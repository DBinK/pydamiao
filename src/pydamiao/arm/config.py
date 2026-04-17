
from enum import IntEnum

from pydamiao.structs import MotorType
from pydamiao.arm.joint import JointCfg

joint_cfgs = [                                    
    JointCfg(MotorType.DM4310, 0x06, 0x16, "wrist_3",  direction=-1),  
    JointCfg(MotorType.DM4310, 0x05, 0x15, "wrist_2",  direction=-1),
    JointCfg(MotorType.DM4310, 0x04, 0x14, "wrist_1",  direction=-1),
    JointCfg(MotorType.DM4340, 0x03, 0x13, "elbow",    direction=-1),
    JointCfg(MotorType.DM4340, 0x02, 0x12, "shoulder", direction=-1),
    JointCfg(MotorType.DM4340, 0x01, 0x11, "base",     direction=-1),
]

class JointID(IntEnum):
    """与 slave_id 映射 """
    base     = 0x01
    shoulder = 0x02
    elbow    = 0x03
    wrist_1  = 0x04
    wrist_2  = 0x05
    wrist_3  = 0x06

