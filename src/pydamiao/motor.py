from __future__ import annotations

import threading
from dataclasses import dataclass, field
from time import time

from pydamiao.bus import SerialBus
from pydamiao.protocol import DamiaoProtocol, ParsedMessage
from pydamiao.result import Result
from pydamiao.types import ControlMode, MotorReg, MotorState, MotorType, MotorLimits, MOTOR_LIMITS


@dataclass(slots=True)
class Motor:
    bus: SerialBus
    motor_type: MotorType
    slave_id: int
    master_id: int = 0
    pos: float = field(init=False, default=0.0)
    vel: float = field(init=False, default=0.0)
    torque: float = field(init=False, default=0.0)
    enabled: bool = field(init=False, default=False)
    control_mode: ControlMode | None = field(init=False, default=None)
    param_cache: dict[int, float | int] = field(init=False, default_factory=dict)
    last_update_time: float | None = field(init=False, default=None)
    _state_lock: threading.RLock = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self._state_lock = threading.RLock()
        self.bus.register_motor(self)

    @property
    def alias_ids(self) -> tuple[int, ...]:
        if self.master_id != 0:
            return self.slave_id, self.master_id
        return (self.slave_id,)

    @property
    def limits(self) -> MotorLimits:
        return MOTOR_LIMITS[self.motor_type]

    def get_state(self) -> MotorState:
        with self._state_lock:
            return MotorState(pos=self.pos, vel=self.vel, torque=self.torque)

    def get_position(self) -> float:
        with self._state_lock:
            return self.pos

    def get_velocity(self) -> float:
        with self._state_lock:
            return self.vel

    def get_torque(self) -> float:
        with self._state_lock:
            return self.torque

    def get_param(self, reg_id: MotorReg) -> float | int | None:
        with self._state_lock:
            return self.param_cache.get(int(reg_id))

    def enable(self) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.ENABLE_CMD))
        with self._state_lock:
            self.enabled = True
        return Result.success()

    def disable(self) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.DISABLE_CMD))
        with self._state_lock:
            self.enabled = False
        return Result.success()

    def set_zero(self) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.SET_ZERO_CMD))
        return Result.success()

    def clear_error(self) -> Result[None]:
        return Result.failure("clear_error is not supported by the current protocol implementation", code="unsupported")

    def set_mode(self, mode: ControlMode, timeout: float = 1.0) -> Result[ControlMode]:
        result = self.write_param(MotorReg.CTRL_MODE, int(mode), timeout=timeout)
        if not result.ok:
            return Result.failure(result.error or "Failed to switch mode", code=result.code or "error")

        with self._state_lock:
            self.control_mode = mode
        return Result.success(mode)

    def set_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        torque: float,
    ) -> Result[None]:
        self.bus.send(
            DamiaoProtocol.encode_mit_control(
                self.slave_id,
                self.limits,
                kp=kp,
                kd=kd,
                pos=pos,
                vel=vel,
                torque=torque,
            )
        )
        return Result.success()

    def set_pos_vel(self, pos: float, vel: float) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_pos_vel_control(self.slave_id, pos, vel))
        return Result.success()

    def set_velocity(self, vel: float) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_velocity_control(self.slave_id, vel))
        return Result.success()

    def set_pos_force(self, pos: float, vel: float, current: float) -> Result[None]:
        self.bus.send(DamiaoProtocol.encode_pos_force_control(self.slave_id, pos, vel, current))
        return Result.success()

    def refresh_state(self, timeout: float = 0.5) -> Result[MotorState]:
        result = self.bus.request(
            DamiaoProtocol.encode_refresh_state(self.slave_id),
            matcher=lambda message: message.kind == "status" and self._matches_message(message),
            timeout=timeout,
        )
        if not result.ok:
            return Result.failure(result.error or "Failed to refresh state", code=result.code or "error")
        return Result.success(self.get_state())

    def read_param(self, reg_id: MotorReg, timeout: float = 1.0) -> Result[float | int]:
        result = self.bus.request(
            DamiaoProtocol.encode_read_param(self.slave_id, reg_id),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result.ok:
            return Result.failure(result.error or "Failed to read motor parameter", code=result.code or "error")
        if result.value is None:
            return Result.failure("Motor did not return a parameter value", code="invalid_response")
        return Result.success(result.value.value)

    def write_param(self, reg_id: MotorReg, value: float | int, timeout: float = 1.0) -> Result[float | int]:
        result = self.bus.request(
            DamiaoProtocol.encode_write_param(self.slave_id, reg_id, value),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result.ok:
            return Result.failure(result.error or "Failed to write motor parameter", code=result.code or "error")
        if result.value is None:
            return Result.failure("Motor did not return a parameter value", code="invalid_response")

        returned = result.value.value
        if returned is None:
            return Result.failure("Motor did not return a parameter value", code="invalid_response")

        if isinstance(returned, float):
            if abs(returned - float(value)) > 0.1:
                return Result.failure("Motor parameter verification failed", code="mismatch")
        elif returned != int(value):
            return Result.failure("Motor parameter verification failed", code="mismatch")

        if int(reg_id) == int(MotorReg.CTRL_MODE):
            with self._state_lock:
                self.control_mode = ControlMode(int(returned))
        return Result.success(returned)

    def save_params(self) -> Result[None]:
        self.disable()
        self.bus.send(DamiaoProtocol.encode_save_params(self.slave_id))
        return Result.success()

    def _matches_message(self, message: ParsedMessage) -> bool:
        if self.slave_id in message.route_ids:
            return True
        if self.master_id != 0 and self.master_id in message.route_ids:
            return True
        return message.slave_id == self.slave_id

    def _handle_message(self, message: ParsedMessage) -> None:
        if message.kind == "status":
            pos, vel, torque = DamiaoProtocol.decode_status(message.data, self.limits)
            with self._state_lock:
                self.pos = pos
                self.vel = vel
                self.torque = torque
                self.last_update_time = time()
            return

        if message.kind == "param" and message.reg_id is not None:
            with self._state_lock:
                self.param_cache[message.reg_id] = message.value
                self.last_update_time = time()


class MotorManager:
    def __init__(self, bus: SerialBus) -> None:
        self.bus = bus
        self._motors_by_slave_id: dict[int, Motor] = {}
        self._motors_by_alias: dict[int, Motor] = {}

    @property
    def motors(self) -> dict[int, Motor]:
        return dict(self._motors_by_slave_id)

    def add_motor(self, motor_type: MotorType, slave_id: int, master_id: int = 0) -> Motor:
        self._ensure_aliases_available((slave_id,) if master_id == 0 else (slave_id, master_id))
        motor = Motor(bus=self.bus, motor_type=motor_type, slave_id=slave_id, master_id=master_id)
        self.register(motor)
        return motor

    def register(self, motor: Motor) -> Motor:
        if motor.bus is not self.bus:
            raise ValueError("MotorManager can only register motors that use the same SerialBus")

        self._ensure_aliases_available(motor.alias_ids, current_motor=motor)
        existing = self._motors_by_slave_id.get(motor.slave_id)
        if existing is not None and existing is not motor:
            raise ValueError(f"Motor slave id 0x{motor.slave_id:X} is already registered")

        self._motors_by_slave_id[motor.slave_id] = motor
        for alias_id in motor.alias_ids:
            owner = self._motors_by_alias.get(alias_id)
            if owner is not None and owner is not motor:
                raise ValueError(f"Motor id 0x{alias_id:X} is already registered")
            self._motors_by_alias[alias_id] = motor
        return motor

    def _ensure_aliases_available(self, alias_ids: tuple[int, ...], current_motor: Motor | None = None) -> None:
        for alias_id in alias_ids:
            owner = self._motors_by_alias.get(alias_id)
            if owner is not None and owner is not current_motor:
                raise ValueError(f"Motor id 0x{alias_id:X} is already registered")

    def get(self, motor_id: int) -> Motor | None:
        return self._motors_by_alias.get(motor_id)

    def __getitem__(self, motor_id: int) -> Motor:
        motor = self.get(motor_id)
        if motor is None:
            raise KeyError(motor_id)
        return motor

    def enable_all(self) -> dict[int, Result[None]]:
        return {motor.slave_id: motor.enable() for motor in self._motors_by_slave_id.values()}

    def disable_all(self) -> dict[int, Result[None]]:
        return {motor.slave_id: motor.disable() for motor in self._motors_by_slave_id.values()}

    def refresh_all(self, timeout: float = 0.5) -> dict[int, Result[MotorState]]:
        return {motor.slave_id: motor.refresh_state(timeout=timeout) for motor in self._motors_by_slave_id.values()}
