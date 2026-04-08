
from enum import IntEnum

from pydamiao.structs import MotorType
from pydamiao.arm.joint import JointCfg

joint_cfgs = [
    JointCfg(MotorType.DM4310, 0x05, 0x15, "wrist_2", pos_min=-10, pos_max=10),
    JointCfg(MotorType.DM4310, 0x06, 0x16, "wrist_3", pos_min=-30, pos_max=30),
    JointCfg(MotorType.DM4310, 0x04, 0x14, "wrist_1", pos_min=-10, pos_max=10),
    JointCfg(MotorType.DM4340, 0x03, 0x13, "elbow", pos_min=-10, pos_max=10),
    JointCfg(MotorType.DM4340, 0x02, 0x12, "shoulder", pos_min=-10, pos_max=10),
    JointCfg(MotorType.DM4340, 0x01, 0x11, "base", pos_min=-10, pos_max=10),
]

class JointID(IntEnum):
    """与 slave_id 映射 """
    base     = 0x01
    shoulder = 0x02
    elbow    = 0x03
    wrist_1  = 0x04
    wrist_2  = 0x05
    wrist_3  = 0x06
