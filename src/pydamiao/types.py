# src/pydamiao/types.py

from enum import IntEnum
from typing import NamedTuple, TypeAlias

# 类型别名
Hex: TypeAlias = int

# 电机类型
class MotorType(IntEnum):
    DM4310 = 0
    DM4310_48V = 1
    DM4340 = 2
    DM4340_48V = 3
    DM6006 = 4
    DM8006 = 5
    DM8009 = 6
    DM10010L = 7
    DM10010 = 8
    DMH3510 = 9
    DMH6215 = 10
    DMG6220 = 11

class MotorLimits(NamedTuple):
    POS_MAX: float  # 位置限制 (弧度)
    VEL_MAX: float  # 角速度限制 (弧度/秒)
    TAU_MAX: float  # 力矩限制 (牛·米)

# 电机限制参数 - 每个电机型号对应的 [POS_MAX, VEL_MAX, TAU_MAX]
MOTOR_LIMITS = {
    MotorType.DM4310: MotorLimits(12.5, 30, 10),
    MotorType.DM4310_48V: MotorLimits(12.5, 50, 10),
    MotorType.DM4340: MotorLimits(12.5, 8, 28),
    MotorType.DM4340_48V: MotorLimits(12.5, 10, 28),
    MotorType.DM6006: MotorLimits(12.5, 45, 20),
    MotorType.DM8006: MotorLimits(12.5, 45, 40),
    MotorType.DM8009: MotorLimits(12.5, 45, 54),
    MotorType.DM10010L: MotorLimits(12.5, 25, 200),
    MotorType.DM10010: MotorLimits(12.5, 20, 200),
    MotorType.DMH3510: MotorLimits(12.5, 280, 1),
    MotorType.DMH6215: MotorLimits(12.5, 45, 10),
    MotorType.DMG6220: MotorLimits(12.5, 45, 10),
}

# 电机参数枚举
class MotorReg(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81
    
    @staticmethod
    def is_int_type(reg_id: int) -> bool:
        """
        这是一个规则函数, 判断某个电机寄存器处对应的值需要的类型, 是否为整数, 否则为浮点数
        """
        return (7 <= reg_id <= 10) or (13 <= reg_id <= 16) or (35 <= reg_id <= 36)


# 控制模式枚举
class ControlMode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    TORQUE_POS = 4


# CAN 命令反馈枚举
class CanResp(IntEnum):
    #! 注意: 在官方文档的 CAN接收数据帧格式 中, 本字段被称为 CAN 命令
    HEARTBEAT = 0x00        # 心跳

    RECEIVE_FAILED = 0x01   # 数据接收失败
    RECEIVE_SUCCESS = 0x11  # 数据接收成功，包含有效数据

    SEND_FAILED = 0x02      # 数据发送失败
    SEND_SUCCESS = 0x12     # 数据发送成功

    BAUD_RATE_SET_FAILED = 0x03  # 波特率设置失败
    BAUD_RATE_SET_SUCCESS = 0x13  # 波特率设置成功

    COMM_ERROR = 0xEE       # 通讯错误信息