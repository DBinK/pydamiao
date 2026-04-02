import threading
import time

import pytest

from pydamiao import ControlMode, Motor, MotorManager, MotorReg, MotorType, Result, SerialBus
from pydamiao.protocol import DamiaoProtocol
from pydamiao.types import CanResp, MotorFault, MOTOR_LIMITS
from pydamiao.utils import float_to_uint, float_to_uint8s, int_to_uint8s


class FakeSerial:
    def __init__(self, timeout: float = 0.01):
        self.timeout = timeout
        self.is_open = True
        self.writes: list[bytes] = []
        self._buffer = bytearray()
        self._condition = threading.Condition()

    def open(self) -> None:
        self.is_open = True
        with self._condition:
            self._condition.notify_all()

    def close(self) -> None:
        self.is_open = False
        with self._condition:
            self._condition.notify_all()

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        return len(data)

    def read(self, size: int = 1) -> bytes:
        deadline = time.time() + self.timeout
        with self._condition:
            while self.is_open and not self._buffer:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return b""
                self._condition.wait(remaining)

            if not self._buffer:
                return b""

            chunk = bytes(self._buffer[:size])
            del self._buffer[:size]
            return chunk

    def feed(self, data: bytes) -> None:
        with self._condition:
            self._buffer.extend(data)
            self._condition.notify_all()


def make_rx_frame(can_resp: CanResp, can_id: int, payload: bytes) -> bytes:
    return bytes([0xAA, int(can_resp), 0x00]) + can_id.to_bytes(4, "little") + payload + bytes([0x55])


def make_status_payload(
    pos: float,
    vel: float,
    torque: float,
    source_id: int,
    fault: MotorFault = MotorFault.NONE,
    mos_temp: int = 0,
    rotor_temp: int = 0,
) -> bytes:
    limits = MOTOR_LIMITS[MotorType.DM4310]
    pos_uint = int(float_to_uint(pos, -limits.POS_MAX, limits.POS_MAX, 16))
    vel_uint = int(float_to_uint(vel, -limits.VEL_MAX, limits.VEL_MAX, 12))
    torque_uint = int(float_to_uint(torque, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12))

    return bytes(
        [
            ((int(fault) & 0x0F) << 4) | (source_id & 0x0F),
            (pos_uint >> 8) & 0xFF,
            pos_uint & 0xFF,
            (vel_uint >> 4) & 0xFF,
            ((vel_uint & 0x0F) << 4) | ((torque_uint >> 8) & 0x0F),
            torque_uint & 0xFF,
            mos_temp & 0xFF,
            rotor_temp & 0xFF,
        ]
    )


def make_param_payload(slave_id: int, op: int, reg_id: MotorReg, value: float | int) -> bytes:
    head = bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, op, int(reg_id)])
    if MotorReg.is_int_type(int(reg_id)):
        encoded = bytes(int_to_uint8s(int(value)))
    else:
        encoded = bytes(float_to_uint8s(float(value)))
    return head + encoded


def feed_later(fake_serial: FakeSerial, frame: bytes, delay: float = 0.02) -> None:
    def _worker() -> None:
        time.sleep(delay)
        fake_serial.feed(frame)

    threading.Thread(target=_worker, daemon=True).start()


def feed_many_later(fake_serial: FakeSerial, frames: list[bytes], delay: float = 0.02, step: float = 0.02) -> None:
    def _worker() -> None:
        time.sleep(delay)
        for frame in frames:
            fake_serial.feed(frame)
            time.sleep(step)

    threading.Thread(target=_worker, daemon=True).start()


def test_motor_refresh_state_uses_background_receiver():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_status_payload(pos=1.2, vel=-2.0, torque=0.8, source_id=0x06),
        )
        feed_later(fake_serial, frame)

        result = motor.refresh_state(timeout=0.2)
        state = result.unwrap()

        assert result.is_ok
        assert state.pos == pytest.approx(1.2, abs=0.02)
        assert state.vel == pytest.approx(-2.0, abs=0.05)
        assert state.torque == pytest.approx(0.8, abs=0.05)
        assert motor.get_position() == pytest.approx(1.2, abs=0.02)
        assert motor.last_update_time is not None
    finally:
        bus.close()


def test_motor_caches_fault_code_from_status_feedback():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_status_payload(
                pos=0.0,
                vel=0.0,
                torque=0.0,
                source_id=0x06,
                fault=MotorFault.UNDER_VOLTAGE,
            ),
        )
        feed_later(fake_serial, frame)

        result = motor.refresh_state(timeout=0.2)

        assert result.is_ok
        assert motor.fault == MotorFault.UNDER_VOLTAGE
        assert motor.enabled is False
    finally:
        bus.close()


def test_motor_caches_temperatures_from_status_feedback():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_status_payload(
                pos=0.0,
                vel=0.0,
                torque=0.0,
                source_id=0x06,
                mos_temp=48,
                rotor_temp=55,
            ),
        )
        feed_later(fake_serial, frame)

        result = motor.refresh_state(timeout=0.2)

        assert result.is_ok
        assert motor.get_mos_temp() == 48
        assert motor.get_rotor_temp() == 55
    finally:
        bus.close()


def test_motor_read_and_write_param_use_request_response_matching():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        read_frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_param_payload(0x06, 0x33, MotorReg.PMAX, 12.5),
        )
        feed_later(fake_serial, read_frame)
        read_result = motor.read_param(MotorReg.PMAX, timeout=0.2)

        assert read_result.is_ok
        assert read_result.value == pytest.approx(12.5, abs=1e-5)
        assert motor.get_param(MotorReg.PMAX) == pytest.approx(12.5, abs=1e-5)

        write_frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_param_payload(0x06, 0x55, MotorReg.CTRL_MODE, int(ControlMode.POS_VEL)),
        )
        feed_later(fake_serial, write_frame)
        write_result = motor.set_mode(ControlMode.POS_VEL, timeout=0.2)

        assert write_result.is_ok
        assert write_result.value == ControlMode.POS_VEL
        assert motor.control_mode == ControlMode.POS_VEL
    finally:
        bus.close()


def test_control_commands_auto_switch_mode_before_sending():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        mode_frame = make_rx_frame(
            CanResp.RECEIVE_SUCCESS,
            0x16,
            make_param_payload(0x06, 0x55, MotorReg.CTRL_MODE, int(ControlMode.POS_VEL)),
        )
        feed_later(fake_serial, mode_frame)
        result = motor.set_pos_vel(1.0, 2.0)

        assert result.is_ok
        assert motor.control_mode == ControlMode.POS_VEL
        assert fake_serial.writes[0] == DamiaoProtocol.encode_write_param(0x06, MotorReg.CTRL_MODE, int(ControlMode.POS_VEL))
        assert fake_serial.writes[1] == DamiaoProtocol.encode_pos_vel_control(0x06, 1.0, 2.0)
    finally:
        bus.close()


def test_motor_timeout_and_unsupported_paths_return_result_errors():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06)

    try:
        timeout_result = motor.read_param(MotorReg.PMAX, timeout=0.05)
        clear_result = motor.clear_error()

        assert not timeout_result.is_ok
        assert timeout_result.code == "timeout"
        assert clear_result.is_ok
    finally:
        bus.close()


def test_control_commands_are_blocked_when_motor_has_fault():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        feed_later(
            fake_serial,
            make_rx_frame(
                CanResp.RECEIVE_SUCCESS,
                0x16,
                make_status_payload(
                    pos=0.0,
                    vel=0.0,
                    torque=0.0,
                    source_id=0x06,
                    fault=MotorFault.OVER_CURRENT,
                ),
            ),
        )
        motor.refresh_state(timeout=0.2)

        result = motor.set_pos_vel(1.0, 2.0)

        assert not result.is_ok
        assert result.code == "fault"
        assert fake_serial.writes == [
            DamiaoProtocol.encode_refresh_state(0x06),
        ]
    finally:
        bus.close()


def test_disable_retries_until_velocity_is_small_enough():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

    try:
        feed_later(
            fake_serial,
            make_rx_frame(
                CanResp.RECEIVE_SUCCESS,
                0x16,
                make_status_payload(pos=0.0, vel=1.0, torque=0.0, source_id=0x06),
            ),
            delay=0.01,
        )
        feed_later(
            fake_serial,
            make_rx_frame(
                CanResp.RECEIVE_SUCCESS,
                0x16,
                make_status_payload(pos=0.0, vel=0.0005, torque=0.0, source_id=0x06),
            ),
            delay=0.08,
        )

        result = motor.disable()

        assert result.is_ok
        disable_frame = DamiaoProtocol.encode_basic_command(0x06, DamiaoProtocol.DISABLE_CMD)
        assert fake_serial.writes.count(disable_frame) >= 2
        assert set(fake_serial.writes) == {disable_frame}
        assert motor.enabled is False
    finally:
        bus.close()


def test_result_feels_pythonic_to_use():
    success = Result(123)
    failure = Result(error="boom", code="bad")

    assert success.is_ok
    assert success.unwrap() == 123

    assert not failure
    assert not failure.is_ok
    with pytest.raises(RuntimeError, match="boom"):
        failure.unwrap()


def test_motor_manager_supports_direct_registration_and_batch_commands():
    fake_serial = FakeSerial()
    bus = SerialBus(fake_serial, timeout=0.01)
    manager = MotorManager(bus)

    try:
        direct_motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)
        manager.register(direct_motor)
        added_motor = manager.add_motor(MotorType.DM4310, 0x07, 0x17)

        results = manager.enable_all()

        assert manager[0x06] is direct_motor
        assert manager[0x16] is direct_motor
        assert manager.get(0x07) is added_motor
        assert all(result.is_ok for result in results.values())
        assert len(fake_serial.writes) == 2
        assert fake_serial.writes[0] == DamiaoProtocol.encode_basic_command(0x06, DamiaoProtocol.ENABLE_CMD)
        assert fake_serial.writes[1] == DamiaoProtocol.encode_basic_command(0x07, DamiaoProtocol.ENABLE_CMD)
    finally:
        bus.close()
