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
    TORQUE_MAX: float  # 力矩限制 (牛·米)


class MotorState(NamedTuple):
    pos: float
    vel: float
    torque: float

# 电机限制参数 - 每个电机型号对应的 [POS_MAX, VEL_MAX, TORQUE_MAX]
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

# 电机寄存器参数枚举, 官方文档中称为 RID
class MotorReg(IntEnum):
    """
    电机寄存器参数枚举, 官方文档中称为 RID
    从固件的更新日志文档查询: https://gitee.com/kit-miao/motor-firmware
    """
    UV_Value = 0      # 低压保护值, RW, float
    KT_Value = 1      # 扭矩系数, RW, float
    OT_Value = 2      # 过温保护值, RW, float
    OC_Value = 3      # 过流保护值, RW, float
    ACC = 4           # 加速度, RW, float
    DEC = 5           # 减速度, RW, float
    MAX_SPD = 6       # 最大速度, RW, float
    MST_ID = 7        # 反馈ID, RW, uint32
    ESC_ID = 8        # 接收ID, RW, uint32
    TIMEOUT = 9       # 超时警报时间, RW, uint32
    CTRL_MODE = 10    # 控制模式, RW, uint32

    Damp = 11         # 电机粘滞系数, RO, float
    Inertia = 12      # 电机转动惯量, RO, float
    hw_ver = 13       # 保留, RO, uint32
    sw_ver = 14       # 软件版本号, RO, uint32
    SN = 15           # 保留, RO, uint32
    NPP = 16          # 电机极对数, RO, uint32
    Rs = 17           # 电机相电阻, RO, float
    LS = 18           # 电机相电感, RO, float
    Flux = 19         # 电机磁链值, RO, float
    Gr = 20           # 齿轮减速比, RO, float

    PMAX = 21         # 位置映射范围, RW, float
    VMAX = 22         # 速度映射范围, RW, float
    TMAX = 23         # 扭矩映射范围, RW, float
    I_BW = 24         # 电流环控制带宽, RW, float
    KP_ASR = 25       # 速度环Kp, RW, float
    KI_ASR = 26       # 速度环Ki, RW, float
    KP_APR = 27       # 位置环Kp, RW, float
    KI_APR = 28       # 位置环Ki, RW, float
    OV_Value = 29     # 过压保护值, RW, float
    GREF = 30         # 齿轮力矩效率, RW, float
    Deta = 31         # 速度环阻尼系数, RW, float
    V_BW = 32         # 速度环滤波带宽, RW, float
    IQ_c1 = 33        # 电流环增强系数, RW, float
    VL_c1 = 34        # 速度环增强系数, RW, float
    can_br = 35       # CAN波特率代码, RW, uint32
    
    sub_ver = 36      # 子版本号, RO, uint32
    u_off = 50        # u相偏置, RO, float
    v_off = 51        # v相偏置, RO, float
    k1 = 52           # 补偿因子1, RO, float
    k2 = 53           # 补偿因子2, RO, float
    m_off = 54        # 角度偏移, RO, float
    dir = 55          # 方向, RO, float
    p_m = 80          # 电机当前位置, RO, float
    xout = 81         # 输出轴位置, RO, float
    
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

