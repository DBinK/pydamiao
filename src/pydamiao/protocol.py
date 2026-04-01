from __future__ import annotations

from dataclasses import dataclass
from struct import pack

import numpy as np

from pydamiao.types import CanResp, Hex, MotorLimits, MotorReg
from pydamiao.utils import float_to_uint, float_to_uint8s, int_to_uint8s, uint8s_to_float, uint8s_to_uint32, uint_to_float


@dataclass(slots=True, frozen=True)
class ParsedMessage:
    kind: str
    can_resp: CanResp
    can_id: Hex
    data: bytes
    route_ids: tuple[int, ...] = ()
    reg_id: int | None = None
    value: float | int | None = None
    slave_id: int | None = None


class DamiaoProtocol:
    BRIDGE_TX_HEADER = bytes([0x55, 0xAA, 0x1E, 0x03])
    BRIDGE_TX_SEND_COUNT = bytes([0x01, 0x00, 0x00, 0x00])
    BRIDGE_TX_INTERVAL = bytes([0x0A, 0x00, 0x00, 0x00])
    BRIDGE_TX_SUFFIX = bytes([0x00, 0x08, 0x00, 0x00])
    BRIDGE_TX_CRC = bytes([0x00])

    RX_FRAME_HEADER = 0xAA
    RX_FRAME_TAIL = 0x55
    RX_FRAME_LENGTH = 16

    ENABLE_CMD = 0xFC
    DISABLE_CMD = 0xFD
    SET_ZERO_CMD = 0xFE

    POS_VEL_BASE_ID = 0x100
    VEL_BASE_ID = 0x200
    POS_FORCE_BASE_ID = 0x300
    QUERY_ID = 0x7FF

    @classmethod
    def build_bridge_frame(cls, can_id: Hex, payload: bytes) -> bytes:
        if len(payload) != 8:
            raise ValueError("CAN payload must be exactly 8 bytes")

        return b"".join(
            [
                cls.BRIDGE_TX_HEADER,
                cls.BRIDGE_TX_SEND_COUNT,
                cls.BRIDGE_TX_INTERVAL,
                bytes([0x00]),
                pack("<I", can_id),
                cls.BRIDGE_TX_SUFFIX,
                payload,
                cls.BRIDGE_TX_CRC,
            ]
        )

    @classmethod
    def encode_basic_command(cls, motor_id: Hex, command: int) -> bytes:
        payload = bytes([0xFF] * 7 + [command])
        return cls.build_bridge_frame(motor_id, payload)

    @classmethod
    def encode_mit_control(
        cls,
        motor_id: Hex,
        limits: MotorLimits,
        kp: float,
        kd: float,
        pos: float,
        vel: float,
        torque: float,
    ) -> bytes:
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        pos_uint = float_to_uint(pos, -limits.POS_MAX, limits.POS_MAX, 16)
        vel_uint = float_to_uint(vel, -limits.VEL_MAX, limits.VEL_MAX, 12)
        torque_uint = float_to_uint(torque, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)

        payload = bytes(
            [
                (pos_uint >> 8) & 0xFF,
                pos_uint & 0xFF,
                vel_uint >> 4,
                ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
                kp_uint & 0xFF,
                kd_uint >> 4,
                ((kd_uint & 0x0F) << 4) | ((torque_uint >> 8) & 0x0F),
                torque_uint & 0xFF,
            ]
        )
        return cls.build_bridge_frame(motor_id, payload)

    @classmethod
    def encode_pos_vel_control(cls, motor_id: Hex, pos: float, vel: float) -> bytes:
        payload = bytes(float_to_uint8s(pos) + float_to_uint8s(vel))
        return cls.build_bridge_frame(cls.POS_VEL_BASE_ID + motor_id, payload)

    @classmethod
    def encode_velocity_control(cls, motor_id: Hex, vel: float) -> bytes:
        payload = bytes(float_to_uint8s(vel) + (0, 0, 0, 0))
        return cls.build_bridge_frame(cls.VEL_BASE_ID + motor_id, payload)

    @classmethod
    def encode_pos_force_control(cls, motor_id: Hex, pos: float, vel: float, current: float) -> bytes:
        vel_uint = np.uint16(vel)
        current_uint = np.uint16(current)
        payload = bytes(
            float_to_uint8s(pos)
            + (
                int(vel_uint & 0xFF),
                int(vel_uint >> 8),
                int(current_uint & 0xFF),
                int(current_uint >> 8),
            )
        )
        return cls.build_bridge_frame(cls.POS_FORCE_BASE_ID + motor_id, payload)

    @classmethod
    def encode_refresh_state(cls, slave_id: Hex) -> bytes:
        payload = bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00])
        return cls.build_bridge_frame(cls.QUERY_ID, payload)

    @classmethod
    def encode_read_param(cls, slave_id: Hex, reg_id: MotorReg) -> bytes:
        payload = bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, 0x33, int(reg_id), 0x00, 0x00, 0x00, 0x00])
        return cls.build_bridge_frame(cls.QUERY_ID, payload)

    @classmethod
    def encode_write_param(cls, slave_id: Hex, reg_id: MotorReg, value: float | int) -> bytes:
        payload = bytearray([slave_id & 0xFF, (slave_id >> 8) & 0xFF, 0x55, int(reg_id), 0x00, 0x00, 0x00, 0x00])
        if MotorReg.is_int_type(int(reg_id)):
            payload[4:8] = bytes(int_to_uint8s(int(value)))
        else:
            payload[4:8] = bytes(float_to_uint8s(float(value)))
        return cls.build_bridge_frame(cls.QUERY_ID, bytes(payload))

    @classmethod
    def encode_save_params(cls, slave_id: Hex) -> bytes:
        payload = bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00])
        return cls.build_bridge_frame(cls.QUERY_ID, payload)

    @classmethod
    def extract_frames(cls, buffer: bytes) -> tuple[list[bytes], bytes]:
        frames: list[bytes] = []
        frame_length = cls.RX_FRAME_LENGTH
        start = 0
        scan = 0

        while scan <= len(buffer) - frame_length:
            if buffer[scan] == cls.RX_FRAME_HEADER and buffer[scan + frame_length - 1] == cls.RX_FRAME_TAIL:
                frames.append(buffer[scan : scan + frame_length])
                scan += frame_length
                start = scan
                continue
            scan += 1

        if frames:
            return frames, buffer[start:]

        keep = max(frame_length - 1, 0)
        return frames, buffer[-keep:]

    @classmethod
    def parse_frame(cls, frame: bytes) -> ParsedMessage:
        if len(frame) != cls.RX_FRAME_LENGTH:
            raise ValueError("Unexpected rx frame length")
        if frame[0] != cls.RX_FRAME_HEADER or frame[-1] != cls.RX_FRAME_TAIL:
            raise ValueError("Malformed rx frame")

        can_resp = CanResp(frame[1])
        can_id = int.from_bytes(frame[3:7], byteorder="little", signed=False)
        data = bytes(frame[7:15])

        if can_resp == CanResp.RECEIVE_SUCCESS:
            if data[2] in (0x33, 0x55):
                slave_id = (data[1] << 8) | data[0]
                reg_id = data[3]
                value: int | float
                if MotorReg.is_int_type(reg_id):
                    value = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                else:
                    value = uint8s_to_float(data[4], data[5], data[6], data[7])

                route_ids = tuple(dict.fromkeys([motor_id for motor_id in (can_id, slave_id) if motor_id != 0]))
                if not route_ids:
                    route_ids = (slave_id,)
                return ParsedMessage(
                    kind="param",
                    can_resp=can_resp,
                    can_id=can_id,
                    data=data,
                    route_ids=route_ids,
                    reg_id=reg_id,
                    value=value,
                    slave_id=slave_id,
                )

            route_id = can_id if can_id != 0 else data[0] & 0x0F
            return ParsedMessage(
                kind="status",
                can_resp=can_resp,
                can_id=can_id,
                data=data,
                route_ids=(route_id,),
            )

        kind = "heartbeat" if can_resp == CanResp.HEARTBEAT else "can_result"
        route_ids = tuple(motor_id for motor_id in (can_id,) if motor_id != 0)
        return ParsedMessage(kind=kind, can_resp=can_resp, can_id=can_id, data=data, route_ids=route_ids)

    @classmethod
    def decode_status(cls, data: bytes, limits: MotorLimits) -> tuple[float, float, float]:
        pos_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
        vel_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
        torque_uint = np.uint16(((data[4] & 0x0F) << 8) | data[5])

        return (
            float(uint_to_float(pos_uint, -limits.POS_MAX, limits.POS_MAX, 16)),
            float(uint_to_float(vel_uint, -limits.VEL_MAX, limits.VEL_MAX, 12)),
            float(uint_to_float(torque_uint, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)),
        )
