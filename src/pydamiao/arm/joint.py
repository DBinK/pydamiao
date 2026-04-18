import math
import time
from dataclasses import dataclass

from rich import print as rprint

from pydamiao import Motor, MotorState, Result, SerialBus
from pydamiao.structs import ControlMode, MotorId, MotorType
from pydamiao.utils import limit_min_max


@dataclass
class JointCfg():
    # 关节属性
    motor_type: MotorType
    slave_id: MotorId
    master_id: MotorId
    name: str
    offset: float = 0.0
    direction: float  = 1      # 方向系数, 取值 -1 或 1 , 默认正转为 1
    scale: float = 1.0         # 倍率系数，默认为1.0, 一般用于调整减速比

    # 默认参数
    pos: float = 0.0  # 默认位置
    vel: float = 0.0  # 默认速度

    # MIT 模式参数
    mit_kp: float     = 15.0
    mit_kd: float     = 1.0
    mit_torque: float = 0.0

    # PVT 模式参数
    pvt_vel = 8000
    pvt_current = 2000   # 电流标幺值放大10000倍, 范围 0-10000

    # POS_VEL 模式参数
    pv_vel: float = 50000.0

    # 机器人关节参数
    pos_min: float = -math.pi    # 默认最小位置
    pos_max: float = math.pi     # 默认最大位置  


class Joint:
    def __init__(self, cfg: JointCfg, bus: SerialBus):
        self.cfg = cfg

        self.name = cfg.name
        self.slave_id = cfg.slave_id

        self.offset = cfg.offset
        self.direction = cfg.direction
        self.scale = cfg.scale  # 获取倍率系数

        self.motor = Motor(bus, cfg.motor_type, cfg.slave_id, cfg.master_id, cfg.name)

    # 预处理数据
    def apply_clip(self, pos: float) -> float:
        """应用限幅"""
        return limit_min_max(pos, self.cfg.pos_min, self.cfg.pos_max)

    def apply_offset(self, pos: float) -> float:
        """应用偏移"""
        return pos + self.offset

    def apply_direction(self, pos: float) -> float:
        """应用方向"""
        return pos * self.direction

    def apply_scale(self, pos: float) -> float:
        """应用倍率系数"""
        return pos * self.scale

    def update(self) -> Result[MotorState]:
        """更新关节状态"""
        return self.motor.refresh_state()

    def set_zero_hard(self):
        """设置硬零点, 写入电机 flash (推荐设置软零点, 保护 flash 寿命) """
        self.motor.set_zero()

    def set_zero(self):
        """设置软零点, 适用于 """
        self.offset = -(self.motor.pos * self.direction)  # 设置当前位置为偏移值

    def get_pos(self) -> float:
        """获取当前关节位置"""
        return (self.motor.pos - self.offset) * self.direction / self.scale  # 应用倍率系数

    def get_vel(self) -> float:
        """获取当前关节速度"""
        return self.motor.vel * self.direction / self.scale  # 应用倍率系数

    def get_torque(self) -> float:
        """获取当前关节力矩"""
        return self.motor.torque * self.direction / self.scale  # 应用倍率系数

    def set_mit_pos(self, pos: float) -> Result[None]:
        """设置关节位置"""
        pos = self.apply_direction(pos)  # 添加方向
        pos = self.apply_offset(pos)     # 添加偏移
        pos = self.apply_clip(pos)       # 限幅
        pos = self.apply_scale(pos)      # 应用倍率系数
        return self.motor.set_mit(pos, self.cfg.vel, self.cfg.mit_kp, self.cfg.mit_kd, self.cfg.mit_torque)

    def set_pos_force(self, pos: float, vel: int, current: int) -> Result[None]:
        """设置关节位置"""
        pos = self.apply_direction(pos)  # 添加方向
        pos = self.apply_offset(pos)     # 添加偏移
        pos = self.apply_clip(pos)       # 限幅
        pos = self.apply_scale(pos)      # 应用倍率系数
        return self.motor.set_pos_force(pos, vel, current)

    def set_pos_vel(self, pos: float, vel: float) -> Result[None]:
        """设置关节位置"""
        pos = self.apply_direction(pos)  # 添加方向
        pos = self.apply_offset(pos)     # 添加偏移
        pos = self.apply_clip(pos)       # 限幅
        pos = self.apply_scale(pos)      # 应用倍率系数
        return self.motor.set_pos_vel(pos, vel)


class JointManager:
    def __init__(self, bus: SerialBus):
        self.bus = bus
        self.joints_by_name: dict[str, Joint] = {}
        self.joints_by_slave_id: dict[MotorId, Joint] = {}
    
    ### 关节管理 ###

    def add_joint(self, joint: Joint):
        self.joints_by_name[joint.name] = joint
        self.joints_by_slave_id[joint.slave_id] = joint

    def register(self, joint_cfgs: list[JointCfg]) -> dict[MotorId, Joint]:
        for joint_cfg in joint_cfgs:
            joint = Joint(joint_cfg, self.bus)
            self.add_joint(joint)
        return self.joints_by_slave_id

    def get_joint(self, name: str) -> Joint:
        return self.joints_by_name[name]
    
    def get_joint_by_id(self, slave_id: MotorId) -> Joint:
        return self.joints_by_slave_id[slave_id]
    
    @property
    def joints(self) -> list[Joint]:
        sorted_ids = sorted(self.joints_by_slave_id.keys())
        return [self.joints_by_slave_id[slave_id] for slave_id in sorted_ids]

    ### 关节状态 ###

    def update(self):
        for joint in self.joints_by_name.values():
            joint.update()

    def get_joints_pos(self) -> dict[MotorId, float]:
        return {joint.slave_id: joint.get_pos() for joint in self.joints_by_name.values()}
    
    def get_joints_pos_list(self) -> list[float]:
        return self.pos_dict_to_list(self.get_joints_pos())

    def get_joints_vel(self) -> dict[MotorId, float]:
        return {joint.slave_id: joint.get_vel() for joint in self.joints_by_name.values()}
    
    def get_joints_torque(self) -> dict[MotorId, float]:
        return {joint.slave_id: joint.get_torque() for joint in self.joints_by_name.values()}

    ### 关节控制 ###
    def set_mode(self, mode: ControlMode):
        for joint in self.joints_by_name.values():
            ret = joint.motor.set_mode(mode)
            print(ret)

    def clean_error(self):
        for joint in self.joints_by_name.values():
            joint.motor.clean_error()

    def enable(self):
        for joint in self.joints_by_name.values():
            joint.motor.enable()
    
    def disable(self):
        for joint in self.joints_by_name.values():
            joint.motor.disable()

    def set_zero_hard(self):
        for joint in self.joints_by_name.values():
            joint.set_zero_hard()

    def set_zero(self):
        for joint in self.joints_by_name.values():
            joint.set_zero()

    def set_pos(self, pos_dict: dict[MotorId, float], mode: ControlMode = ControlMode.MIT) -> list[Result[None]]:
        ret = []
        for slave_id, pos in pos_dict.items():
            joint = self.joints_by_slave_id[slave_id]
            
            if mode == ControlMode.MIT:
                ret_pos = joint.set_mit_pos(pos)
            elif mode == ControlMode.POS_FORCE:
                ret_pos = joint.set_pos_force(pos, joint.cfg.pvt_vel, joint.cfg.pvt_current)
            elif mode == ControlMode.POS_VEL:
                ret_pos = joint.set_pos_vel(pos, joint.cfg.pv_vel)
            else:
                raise ValueError(f"Unsupported control mode: {mode}")

            ret.append(ret_pos)
        return ret

    def set_pos_list(self, pos_list: list[float], mode: ControlMode = ControlMode.MIT):
        pos_dict = self.pos_list_to_dict(pos_list)
        return self.set_pos(pos_dict, mode)

    ### 辅助函数 ###
    def pos_list_to_dict(self, pos_list: list[float]) -> dict[MotorId, float]:
        sorted_ids = sorted(self.joints_by_slave_id.keys())
        return {slave_id: pos for slave_id, pos in zip(sorted_ids, pos_list)}

    def pos_dict_to_list(self, pos_dict: dict[MotorId, float]) -> list[float]:
        sorted_ids = sorted(self.joints_by_slave_id.keys())
        return [pos_dict[slave_id] for slave_id in sorted_ids]
    
    
    ### 其它 ###
    def set_teach_mode(self):
        """设置示教模式, 目前实现为 MIT 模式设置全零"""
        self.set_mode(ControlMode.MIT)
        for joint in self.joints_by_name.values():
            joint.motor.set_mit(0,0,0,0,0)
    
    def set_mit_zero(self):
        """用于示教模式下, 持续发送且接收反馈帧, 从而更新电机位置"""
        for joint in self.joints_by_name.values():
            joint.motor.set_mit(0,0,0,0,0)

    def print_joints_cfg(self):
        """用于打印joints的配置信息"""
        for joint in self.joints_by_name.values():
            rprint(joint.cfg)