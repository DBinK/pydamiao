import pytest

from pydamiao.protocol import DamiaoProtocol
from pydamiao.types import CanResp, MotorFault, MotorReg, MotorType, MOTOR_LIMITS
from pydamiao.utils import float_to_uint, float_to_uint8s, int_to_uint8s


def make_rx_frame(can_resp: CanResp, can_id: int, payload: bytes) -> bytes:
    return bytes([0xAA, int(can_resp), 0x00]) + can_id.to_bytes(4, "little") + payload + bytes([0x55])


def make_status_payload(
    motor_type: MotorType,
    pos: float,
    vel: float,
    torque: float,
    source_id: int,
    fault: MotorFault = MotorFault.NONE,
    mos_temp: int = 0,
    rotor_temp: int = 0,
) -> bytes:
    limits = MOTOR_LIMITS[motor_type]
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
    header = bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, op, int(reg_id)])
    if MotorReg.is_int_type(int(reg_id)):
        encoded = bytes(int_to_uint8s(int(value)))
    else:
        encoded = bytes(float_to_uint8s(float(value)))
    return header + encoded


def test_extract_frames_handles_noise_and_partial_packets():
    frame1 = make_rx_frame(
        CanResp.RECEIVE_SUCCESS,
        0x06,
        make_status_payload(MotorType.DM4310, 1.0, 2.0, 3.0, source_id=0x06),
    )
    frame2 = make_rx_frame(
        CanResp.RECEIVE_SUCCESS,
        0x16,
        make_param_payload(0x06, 0x33, MotorReg.MST_ID, 0x16),
    )

    frames, remainder = DamiaoProtocol.extract_frames(b"\x00\xFF" + frame1 + frame2[:8])
    assert frames == [frame1]
    assert remainder.endswith(frame2[:8])

    more_frames, more_remainder = DamiaoProtocol.extract_frames(remainder + frame2[8:])
    assert more_frames == [frame2]
    assert more_remainder == b""


def test_extract_frames_discards_unbounded_noise():
    frames, remainder = DamiaoProtocol.extract_frames(bytes(range(32)))
    assert frames == []
    assert len(remainder) == DamiaoProtocol.RX_FRAME_LENGTH - 1


def test_parse_frame_decodes_param_reply():
    frame = make_rx_frame(
        CanResp.RECEIVE_SUCCESS,
        0x16,
        make_param_payload(0x06, 0x55, MotorReg.CTRL_MODE, 2),
    )

    message = DamiaoProtocol.parse_frame(frame)

    assert message.kind == "param"
    assert message.reg_id == MotorReg.CTRL_MODE
    assert message.value == 2
    assert message.slave_id == 0x06
    assert message.route_ids == (0x16, 0x06)


def test_decode_status_round_trip():
    payload = make_status_payload(MotorType.DM4310, pos=1.25, vel=-3.5, torque=2.5, source_id=0x06)
    pos, vel, torque = DamiaoProtocol.decode_status(payload, MOTOR_LIMITS[MotorType.DM4310])

    assert pos == pytest.approx(1.25, abs=0.01)
    assert vel == pytest.approx(-3.5, abs=0.05)
    assert torque == pytest.approx(2.5, abs=0.05)


def test_parse_frame_decodes_fault_code_from_status():
    frame = make_rx_frame(
        CanResp.RECEIVE_SUCCESS,
        0x16,
        make_status_payload(
            MotorType.DM4310,
            pos=0.0,
            vel=0.0,
            torque=0.0,
            source_id=0x06,
            fault=MotorFault.OVER_CURRENT,
        ),
    )

    message = DamiaoProtocol.parse_frame(frame)

    assert message.kind == "status"
    assert message.fault == MotorFault.OVER_CURRENT


def test_parse_frame_decodes_temperatures_from_status():
    frame = make_rx_frame(
        CanResp.RECEIVE_SUCCESS,
        0x16,
        make_status_payload(
            MotorType.DM4310,
            pos=0.0,
            vel=0.0,
            torque=0.0,
            source_id=0x06,
            mos_temp=53,
            rotor_temp=61,
        ),
    )

    message = DamiaoProtocol.parse_frame(frame)

    assert message.kind == "status"
    assert message.mos_temp == 53
    assert message.rotor_temp == 61
