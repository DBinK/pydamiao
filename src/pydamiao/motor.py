from __future__ import annotations

import threading
import time

from pydamiao.bus import SerialBus
from pydamiao.protocol import DamiaoProtocol, ParsedMessage
from pydamiao.result import Result
from pydamiao.types import ControlMode, MotorReg, MotorState, MotorType, MotorLimits, MOTOR_LIMITS


class Motor:
    """共享总线上的单个达妙电机高层控制对象。"""

    def __init__(
        self,
        bus: SerialBus,
        motor_type: MotorType,
        slave_id: int,
        master_id: int = 0,
    ) -> None:
        """初始化绑定到共享串口总线的电机对象。

        Args:
            bus: 用于通信的共享串口总线。
            motor_type: 用于选择协议限制的电机型号。
            slave_id: 电机接收 id。
            master_id: 某些固件配置下使用的可选反馈 id。
        """
        self.bus = bus
        self.motor_type = motor_type
        self.slave_id = slave_id
        self.master_id = master_id

        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.enabled = False
        self.control_mode: ControlMode | None = None
        self.param_cache: dict[int, float | int] = {}
        self.last_update_time: float | None = None
        self._state_lock = threading.RLock()
        self.bus.register_motor(self)

    @property
    def alias_ids(self) -> tuple[int, ...]:
        """返回所有可能把消息路由到本电机的 id。"""
        if self.master_id != 0:
            return self.slave_id, self.master_id
        return (self.slave_id,)

    @property
    def limits(self) -> MotorLimits:
        """返回当前电机型号对应的协议限制。"""
        return MOTOR_LIMITS[self.motor_type]

    def get_state(self) -> MotorState:
        """返回当前缓存中的最新电机状态。"""
        with self._state_lock:
            return MotorState(pos=self.pos, vel=self.vel, torque=self.torque)

    def get_position(self) -> float:
        """返回当前缓存中的最新位置。"""
        with self._state_lock:
            return self.pos

    def get_velocity(self) -> float:
        """返回当前缓存中的最新速度。"""
        with self._state_lock:
            return self.vel

    def get_torque(self) -> float:
        """返回当前缓存中的最新力矩。"""
        with self._state_lock:
            return self.torque

    def get_param(self, reg_id: MotorReg) -> float | int | None:
        """返回缓存中的寄存器值；如果尚未读取则返回 ``None``。"""
        with self._state_lock:
            return self.param_cache.get(int(reg_id))

    def enable(self) -> Result[None]:
        """使能电机。"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.ENABLE_CMD))
        with self._state_lock:
            self.enabled = True
        return Result()

    def disable(self) -> Result[None]:
        """失能电机，并等待速度降到很小的阈值以下。"""
        deadline = time.monotonic() + 1.0
        command = DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.DISABLE_CMD)

        while True:  # (安全性) 尝试失能电机, 直到速度降到阈值以下 (达妙没有失能/失能的状态反馈)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return Result(error="Motor did not slow down to the target threshold in time", code="disable_timeout")

            result = self._wait_for_feedback(command, timeout=min(0.05, remaining))
            with self._state_lock:
                self.enabled = False

            if result:
                state = self.get_state()
                if abs(state.vel) <= 0.02:
                    return Result()

    def set_zero(self) -> Result[None]:
        """把当前位置设置为电机零点。"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.SET_ZERO_CMD))
        return Result()

    def clear_error(self) -> Result[None]:
        """清除错误 (过热等错误)"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.CLEAN_ERROR_CMD))
        return Result(error="clear_error is not supported by the current protocol implementation", code="unsupported")

    def set_mode(self, mode: ControlMode, timeout: float = 1.0) -> Result[ControlMode]:
        """切换电机控制模式。

        Args:
            mode: 目标控制模式。
            timeout: 等待写入确认的最大时间。
        """
        result = self.write_param(MotorReg.CTRL_MODE, int(mode), timeout=timeout)
        if not result:
            return Result(error=result.error or "Failed to switch mode", code=result.code or "error")

        with self._state_lock:
            self.control_mode = mode
        return Result(mode)

    def set_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        torque: float,
        auto_switch_mode: bool = True,
    ) -> Result[None]:
        """发送 MIT 控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 目标速度，单位 rad/s。
            kp: 位置增益。
            kd: 速度增益。
            torque: 前馈力矩命令。
            auto_switch_mode: 是否自动切换到 MIT 模式。
        """
        mode_result = self._prepare_control_mode(ControlMode.MIT, auto_switch_mode)
        if not mode_result:
            return mode_result
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
        return Result()

    def set_pos_vel(self, pos: float, vel: float, auto_switch_mode: bool = True) -> Result[None]:
        """发送位置速度控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 目标速度，单位 rad/s。
            auto_switch_mode: 是否自动切换到 POS_VEL 模式。
        """
        mode_result = self._prepare_control_mode(ControlMode.POS_VEL, auto_switch_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_pos_vel_control(self.slave_id, pos, vel))
        return Result()

    def set_velocity(self, vel: float, auto_switch_mode: bool = True) -> Result[None]:
        """发送速度控制命令。

        Args:
            vel: 目标速度，单位 rad/s。
            auto_switch_mode: 是否自动切换到 VEL 模式。
        """
        mode_result = self._prepare_control_mode(ControlMode.VEL, auto_switch_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_velocity_control(self.slave_id, vel))
        return Result()

    def set_pos_force(
        self,
        pos: float,
        vel: float,
        current: float,
        auto_switch_mode: bool = True,
    ) -> Result[None]:
        """发送力位混合控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 电机协议要求的辅助速度项。
            current: 电机协议要求的电流或力矩项。
            auto_switch_mode: 是否自动切换到 TORQUE_POS 模式。
        """
        mode_result = self._prepare_control_mode(ControlMode.TORQUE_POS, auto_switch_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_pos_force_control(self.slave_id, pos, vel, current))
        return Result()

    def refresh_state(self, timeout: float = 0.5) -> Result[MotorState]:
        """请求最新状态，并返回更新后的缓存结果。"""
        result = self.bus.request(
            DamiaoProtocol.encode_refresh_state(self.slave_id),
            matcher=lambda message: message.kind == "status" and self._matches_message(message),
            timeout=timeout,
        )
        if not result:
            return Result(error=result.error or "Failed to refresh state", code=result.code or "error")
        return Result(self.get_state())

    def read_param(self, reg_id: MotorReg, timeout: float = 0.5) -> Result[float | int]:
        """读取电机寄存器。

        Args:
            reg_id: 要读取的寄存器 id。
            timeout: 等待响应的最大时间。
        """
        result = self.bus.request(
            DamiaoProtocol.encode_read_param(self.slave_id, reg_id),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result:
            return Result(error=result.error or "Failed to read motor parameter", code=result.code or "error")
        if result.value is None:
            return Result(error="Motor did not return a parameter value", code="invalid_response")
        return Result(result.value.value)

    def write_param(self, reg_id: MotorReg, value: float | int, timeout: float = 1.0) -> Result[float | int]:
        """写入电机寄存器，并校验回读响应。

        Args:
            reg_id: 要写入的寄存器 id。
            value: 要发送给电机的值。
            timeout: 等待响应的最大时间。
        """
        result = self.bus.request(
            DamiaoProtocol.encode_write_param(self.slave_id, reg_id, value),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result:
            return Result(error=result.error or "Failed to write motor parameter", code=result.code or "error")
        if result.value is None:
            return Result(error="Motor did not return a parameter value", code="invalid_response")

        returned = result.value.value
        if returned is None:
            return Result(error="Motor did not return a parameter value", code="invalid_response")

        if isinstance(returned, float):
            if abs(returned - float(value)) > 0.1:
                return Result(error="Motor parameter verification failed", code="mismatch")
        elif returned != int(value):
            return Result(error="Motor parameter verification failed", code="mismatch")

        if int(reg_id) == int(MotorReg.CTRL_MODE):
            with self._state_lock:
                self.control_mode = ControlMode(int(returned))
        return Result(returned)

    def save_params(self) -> Result[None]:
        """把当前参数集保存到电机 flash, 官方文档称不可高频调用, flash 寿命约1万次"""
        self.disable()
        self.bus.send(DamiaoProtocol.encode_save_params(self.slave_id))
        return Result()

    def _prepare_control_mode(self, target_mode: ControlMode, auto_switch_mode: bool) -> Result[None]:
        if self.control_mode == target_mode:
            return Result()
        if not auto_switch_mode:
            return Result(error=f"Motor is not in {target_mode.name} mode", code="mode_mismatch")

        result = self.set_mode(target_mode)
        if not result:
            return Result(error=result.error or "Failed to switch control mode", code=result.code or "error")
        return Result()

    def _wait_for_feedback(self, command: bytes, timeout: float) -> Result[ParsedMessage]:
        """发送命令并等待属于当前电机的任意反馈。"""
        return self.bus.request(
            command,
            matcher=self._matches_message,
            timeout=timeout,
        )

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
                self.last_update_time = time.time()
            return

        if message.kind == "param" and message.reg_id is not None:
            with self._state_lock:
                if message.value is not None:
                    self.param_cache[message.reg_id] = message.value
                    self.last_update_time = time.time()


class MotorManager:
    """管理共享同一条总线的多个电机对象。"""

    def __init__(self, bus: SerialBus) -> None:
        """初始化多电机管理器。

        Args:
            bus: 所有托管电机共用的串口总线。
        """
        self.bus = bus
        self._motors_by_slave_id: dict[int, Motor] = {}
        self._motors_by_alias: dict[int, Motor] = {}

    @property
    def motors(self) -> dict[int, Motor]:
        """返回一份按 slave id 建立的电机映射副本。"""
        return dict(self._motors_by_slave_id)

    def add_motor(self, motor_type: MotorType, slave_id: int, master_id: int = 0) -> Motor:
        """创建、注册并返回一个新的电机对象。"""
        self._ensure_aliases_available((slave_id,) if master_id == 0 else (slave_id, master_id))
        motor = Motor(bus=self.bus, motor_type=motor_type, slave_id=slave_id, master_id=master_id)
        self.register(motor)
        return motor

    def register(self, motor: Motor) -> Motor:
        """把一个已经创建好的电机对象注册到管理器中。"""
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
        """通过 slave id 或 master id 返回电机对象。"""
        return self._motors_by_alias.get(motor_id)

    def __getitem__(self, motor_id: int) -> Motor:
        motor = self.get(motor_id)
        if motor is None:
            raise KeyError(motor_id)
        return motor

    def enable_all(self) -> dict[int, Result[None]]:
        """使能所有已注册电机。"""
        return {motor.slave_id: motor.enable() for motor in self._motors_by_slave_id.values()}

    def disable_all(self) -> dict[int, Result[None]]:
        """失能所有已注册电机。"""
        return {motor.slave_id: motor.disable() for motor in self._motors_by_slave_id.values()}

    def refresh_all(self, timeout: float = 0.5) -> dict[int, Result[MotorState]]:
        """刷新并返回所有已注册电机的状态。"""
        return {motor.slave_id: motor.refresh_state(timeout=timeout) for motor in self._motors_by_slave_id.values()}
