# src/pydamiao/motor.py
from time import sleep

import numpy as np
from serial import Serial

from pydamiao.types import Hex, ControlMode, MotorType, MotorReg, MotorLimits, MOTOR_LIMITS, CanResp
from pydamiao.utils import (
    int_to_uint8s,
    float_to_uint,
    float_to_uint8s,
    uint8s_to_float,
    uint8s_to_uint32,
    uint_to_float,
)

class Motor:
    """电机类数据容器，用于定义电机对象"""

    def __init__(self, motor_type: MotorType, slave_id: Hex, master_id: Hex):
        """初始化电机对象
        
        Args:
            motor_type: 电机类型
            slave_id: CANID 电机ID
            master_id: 主机ID，建议不要设为0
        """
        self.motor_type = motor_type
        self.slave_id = slave_id
        self.master_id = master_id

        # 电机状态反馈
        self.pos = float(0)
        self.vel = float(0)
        self.torque = float(0)

        self.enabled = False  
        self.control_mode: ControlMode | None = None  
        self.param_cache = {}

    def update_from_controller(self, pos: float, vel: float, torque: float):
        """从控制器被动接受更新数据"""
        self.pos = pos
        self.vel = vel
        self.torque = torque

    def get_position(self):
        """获取电机位置"""
        return self.pos

    def get_velocity(self):
        """获取电机速度"""
        return self.vel

    def get_torque(self):
        """获取电机力矩"""
        return self.torque

    def get_param(self, reg_id: MotorReg):
        """获取电机内部参数，需要提前读取
        
        Args:
            reg_id: 电机寄存器中的值
            
        Returns:
            电机寄存器中的值
        """
        if reg_id in self.param_cache:
            return self.param_cache[reg_id]
        else:
            return None


class MotorController:
    """电机控制器类，用于控制电机"""

    tx_frame = np.array(
        object=[
            0x55, 0xAA, # 帧头
            0x1e,       # 帧长
            0x03,       # 命令
            0x01, 0x00, 0x00, 0x00,  # 发送次数			
            0x0A, 0x00, 0x00, 0x00,  # 时间间隔			
            0x00,                    # ID类型：0x00 标准帧, 0x01扩展帧 
            0x00, 0x00, 0x00, 0x00,  # CAN ID			
            0x00,                    # 帧类型：0x00 数据帧, 0x01远程帧
            0x08,       # len
            0x00,       # idAcc
            0x00,       # dataAcc
            0, 0, 0, 0, 0, 0, 0, 0,  # data[len]	
            0x00        # CRC 校验 (CRC 底层暂时未解析 可以任意数字填充)
        ], 
        dtype=np.uint8)  # fmt: off

    # ==============================================================================
    # 初始化与电机管理
    # ==============================================================================

    def __init__(self, serial_device: Serial):
        """初始化电机控制器
        
        Args:
            serial_device: 串口对象
        """
        self.serial = serial_device
        self.motors_map: dict[Hex, Motor] = dict()
        self.rx_buf = bytes()  # 存储数据
        if self.serial.is_open:  # 打开串口
            print("Serial port is open")
            serial_device.close()
        self.serial.open()

    def add_motor(self, motor: Motor):
        """添加电机到电机控制器
        
        Args:
            motor: 电机对象
            
        Returns:
            True
        """
        self.motors_map[motor.slave_id] = motor
        if motor.master_id != 0:
            self.motors_map[motor.master_id] = motor
        return True


    # ==============================================================================
    # 电机基础控制命令
    # ==============================================================================

    def enable(self, motor: Motor):
        """使能电机
        
        最好在上电后几秒后再使能电机
        
        Args:
            motor: 电机对象
        """
        self.__basic_cmd(motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # 接收来自串口的数据
        motor.enabled = True  # 更新电机使能状态

    def disable(self, motor: Motor):
        """失能电机
        
        Args:
            motor: 电机对象
        """
        self.__basic_cmd(motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()  # 接收来自串口的数据
        motor.enabled = False  # 更新电机使能状态

    def set_zero_position(self, motor: Motor):
        """设置电机零位
        
        Args:
            motor: 电机对象
        """
        self.__basic_cmd(motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()  # 接收来自串口的数据


    # ==============================================================================
    # 各控制模式函数
    # ==============================================================================

    def control_mit(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float
    ):
        """MIT控制模式函数
        
        Args:
            motor: 电机对象
            kp: 比例系数
            kd: 微分系数
            pos_cmd: 期望位置
            vel_cmd: 期望速度
            torque_cmd: 期望力矩
        """
        if motor.slave_id not in self.motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        motor_type = motor.motor_type
        limits = MOTOR_LIMITS[motor_type]
        POS_MAX = limits.POS_MAX
        VEL_MAX = limits.VEL_MAX
        TORQUE_MAX = limits.TORQUE_MAX
        pos_uint = float_to_uint(pos_cmd, -POS_MAX, POS_MAX, 16)
        vel_uint = float_to_uint(vel_cmd, -VEL_MAX, VEL_MAX, 12)
        torque_uint = float_to_uint(torque_cmd, -TORQUE_MAX, TORQUE_MAX, 12)
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        tx_buf[0] = (pos_uint >> 8) & 0xFF
        tx_buf[1] = pos_uint & 0xFF
        tx_buf[2] = vel_uint >> 4
        tx_buf[3] = ((vel_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        tx_buf[4] = kp_uint & 0xFF
        tx_buf[5] = kd_uint >> 4
        tx_buf[6] = ((kd_uint & 0xF) << 4) | ((torque_uint >> 8) & 0xF)
        tx_buf[7] = torque_uint & 0xFF
        self.__send_data(motor.slave_id, tx_buf)
        self.recv()  # 接收来自串口的数据

    def control_mit_delay(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float,
        delay: float,
    ):
        """MIT控制模式函数（带延迟）
        
        Args:
            motor: 电机对象
            kp: 比例系数
            kd: 微分系数
            pos_cmd: 期望位置
            vel_cmd: 期望速度
            torque_cmd: 期望力矩
            delay: 延迟时间，单位秒
        """
        self.control_mit(motor, kp, kd, pos_cmd, vel_cmd, torque_cmd)
        sleep(delay)

    def control_pos_vel(self, motor: Motor, pos_cmd: float, vel_cmd: float):
        """位置速度控制模式
        
        Args:
            motor: 电机对象
            pos_cmd: 期望位置
            vel_cmd: 期望速度
        """
        if motor.slave_id not in self.motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        motor_id = 0x100 + motor.slave_id
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        pos_cmd_u8s = float_to_uint8s(pos_cmd)
        vel_cmd_u8s  = float_to_uint8s(vel_cmd)
        tx_buf[0:4] = pos_cmd_u8s
        tx_buf[4:8] = vel_cmd_u8s 
        self.__send_data(motor_id, tx_buf)
        # time.sleep(0.001)
        self.recv()  # 接收来自串口的数据

    def control_vel(self, motor: Motor, vel_cmd: float):
        """速度控制模式
        
        Args:
            motor: 电机对象
            vel_cmd: 期望速度
        """
        if motor.slave_id not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motor_id = 0x200 + motor.slave_id
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        vel_cmd_u8s = float_to_uint8s(vel_cmd)
        tx_buf[0:4] = vel_cmd_u8s
        self.__send_data(motor_id, tx_buf)
        self.recv()  # 接收来自串口的数据

    def control_pos_force(self, motor: Motor, pos_cmd: float, vel_cmd: float, cur_cmd: float):
        """EMIT控制模式（力位混合模式）
        
        Args:
            motor: 电机对象
            pos_cmd: 期望位置，单位为rad
            vel_cmd: 期望速度，为放大100倍
            cur_cmd: 期望电流标幺值放大10000倍
        """
        if motor.slave_id not in self.motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        motor_id = 0x300 + motor.slave_id
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        pos_cmd_u8s = float_to_uint8s(pos_cmd)
        tx_buf[0:4] = pos_cmd_u8s
        vel_cmd_uint = np.uint16(vel_cmd)
        cur_cmd_uint = np.uint16(cur_cmd)
        tx_buf[4] = vel_cmd_uint & 0xFF
        tx_buf[5] = vel_cmd_uint >> 8
        tx_buf[6] = cur_cmd_uint & 0xFF
        tx_buf[7] = cur_cmd_uint >> 8
        self.__send_data(motor_id, tx_buf)
        self.recv()  # 接收来自串口的数据


    # ==============================================================================
    # 参数管理
    # ==============================================================================

    def refresh_motor_status(self, motor: Motor):
        """获取电机状态
        
        Args:
            motor: 电机对象
        """
        can_id_l = motor.slave_id & 0xFF  # id 低8位
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id 高8位
        tx_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0xCC,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        self.__send_data(0x7FF, tx_buf)  # 发送查询请求
        self.recv()  # 接收来自串口的数据

    def read_motor_param(self, motor: Motor, reg_id: MotorReg):
        """读取电机内部参数（如版本号等）
        
        Args:
            motor: 电机对象
            reg_id: 电机寄存器参数
            
        Returns:
            电机寄存器参数的值
        """
        max_retries = 20       # 最大重试次数
        retry_interval = 0.05  # 重试间隔
        self.__read_motor_param(motor, reg_id)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if motor.slave_id in self.motors_map:
                if reg_id in self.motors_map[motor.slave_id].param_cache:
                    return self.motors_map[motor.slave_id].param_cache[reg_id]
        return None

    def change_motor_param(self, motor: Motor, reg_id: MotorReg, data: float | int):
        """改变电机寄存器参数的值, 仅当前生效, 需要保持参数请用 save_motor_param()
        
        Args:
            motor: 电机对象
            reg_id: 电机寄存器参数
            data: 电机寄存器参数的值
            
        Returns:
            True表示成功，False表示失败
        """
        max_retries = 20
        retry_interval = 0.05  # 重试间隔

        self.__write_motor_param(motor, reg_id, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if (
                motor.slave_id in self.motors_map
                and reg_id in self.motors_map[motor.slave_id].param_cache
            ):
                if (
                    abs(self.motors_map[motor.slave_id].param_cache[reg_id] - data)
                    < 0.1
                ):
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def save_motor_param(self, motor: Motor):
        """保存所有参数到 flash
        
        Args:
            motor: 电机对象
        """
        can_id_l = motor.slave_id & 0xFF  # id 低8位
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id 高8位
        tx_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0xAA,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        self.disable(motor)  # 保存前先失能电机
        self.__send_data(0x7FF, tx_buf)
        sleep(0.001)

    def change_limit_param(self, motor_type: MotorType, PMAX: float, VMAX: float, TMAX: float):
        """改变电机的PMAX VMAX TMAX
        
        Args:
            motor_type: 电机类型
            PMAX: 电机的PMAX
            VMAX: 电机的VMAX
            TMAX: 电机的TMAX
        """
        MOTOR_LIMITS[motor_type] = MotorLimits(PMAX, VMAX, TMAX)
        
    def switch_control_mode(self, motor: Motor, control_mode: ControlMode):
        """切换电机控制模式
        
        Args:
            motor: 电机对象
            control_mode: 电机控制模式，如MIT:ControlMode.MIT MIT模式
            
        Returns:
            True表示成功，False表示失败
        """
        max_retries = 20
        retry_interval = 0.1  # 重试间隔
        # reg_id = 10
        reg_id = MotorReg.CTRL_MODE
        self.__write_motor_param(motor, reg_id, control_mode)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if motor.slave_id in self.motors_map:
                if reg_id in self.motors_map[motor.slave_id].param_cache:
                    if (
                        abs(
                            self.motors_map[motor.slave_id].param_cache[reg_id]
                            - control_mode
                        )
                        < 0.1
                    ):
                        motor.control_mode = control_mode  # 更新控制模式
                        return True
                    else:
                        return False
        return False

    # ==============================================================================
    # 数据收发与处理（底层辅助）
    # ==============================================================================

    def recv(self):
        """从串口接收数据"""
        read_data = self.serial.read_all()   # 把上次没有解析完的剩下的也放进来
        data_recv = b"".join(
            [self.rx_buf, read_data if read_data is not None else b""]
        )
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            can_id = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            can_cmd = CanResp(packet[1])
            self.__process_packet(data, can_id, can_cmd)

    def recv_set_param_data(self):
        """从串口接收设置参数数据"""
        data_recv = self.serial.read_all()
        if data_recv is None:
            data_recv = b""
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            can_id = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            can_cmd = CanResp(packet[1])
            self.__process_set_param_packet(data, can_id, can_cmd)

    def __extract_packets(self, data: bytes) -> list[bytes]:
        """提取数据包"""
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i : i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.rx_buf = data[remainder_pos:]
        return frames

    def __process_packet(self, data: bytes, can_id: Hex, can_cmd: CanResp) -> None:
        """处理数据包, 此处会更新数据到对应 Motor 对象"""
        if can_cmd == CanResp.RECEIVE_SUCCESS:  # CAN 命令 //00 心跳, 0x01 接收失败, 0x11 接收成功, 0x02 发送失败, 0x12 发送成功 0x03 
            # 确定目标电机ID
            if can_id != 0x00:
                target_id = can_id
            else:
                target_id = data[0] & 0x0F

            # 仅当目标电机存在时处理
            if target_id in self.motors_map:
                pos_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                vel_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                torque_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                motor_type_recv = self.motors_map[target_id].motor_type
                limits = MOTOR_LIMITS[motor_type_recv]
                POS_MAX = limits.POS_MAX
                VEL_MAX = limits.VEL_MAX
                TORQUE_MAX = limits.TORQUE_MAX
                recv_pos = uint_to_float(pos_uint, -POS_MAX, POS_MAX, 16)
                recv_vel = uint_to_float(vel_uint, -VEL_MAX, VEL_MAX, 12)
                recv_torque = uint_to_float(torque_uint, -TORQUE_MAX, TORQUE_MAX, 12)
                self.motors_map[target_id].update_from_controller(
                    float(recv_pos), float(recv_vel), float(recv_torque)
                )

    def __process_set_param_packet(self, data: bytes, can_id: Hex, can_cmd: CanResp) -> None:
        """处理设置参数数据包"""
        if can_cmd == CanResp.RECEIVE_SUCCESS and (data[2] == 0x33 or data[2] == 0x55):
            master_id = can_id
            slave_id = (data[1] << 8) | data[0]
            if can_id == 0x00:  # 防止有人把master_id设为0稳一手
                master_id = slave_id

            if master_id not in self.motors_map:
                if slave_id not in self.motors_map:
                    return
                else:
                    master_id = slave_id

            reg_id = data[3]

            # 读取参数得到的数据
            if MotorReg.is_int_type(reg_id):
                # uint32类型
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].param_cache[reg_id] = num
            else:
                # float类型
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].param_cache[reg_id] = num

    def __send_data(self, motor_id: Hex, data: np.ndarray) -> None:
        """发送数据到电机"""
        self.tx_frame[13] = motor_id & 0xFF
        self.tx_frame[14] = (motor_id >> 8) & 0xFF  # id 高8位
        self.tx_frame[21:29] = data
        self.serial.write(bytes(self.tx_frame.T))

    def __basic_cmd(self, motor: Motor, state_cmd: np.uint8):
        """基本命令, 使能/失能/设置零点"""
        tx_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, state_cmd], np.uint8)
        self.__send_data(motor.slave_id, tx_buf)

    def __read_motor_param(self, motor: Motor, reg_id: MotorReg):
        """读取电机寄存器中的值"""
        can_id_l = motor.slave_id & 0xFF  # id 低8位
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id 高8位
        tx_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0x33,
                np.uint8(reg_id),
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        self.__send_data(0x7FF, tx_buf)

    def __write_motor_param(self, motor: Motor, reg_id: MotorReg, data: float | int):
        """ 写入电机寄存器中的值 """
        can_id_l = motor.slave_id & 0xFF  # id 低8位
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id 高8位
        tx_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0x55,
                np.uint8(reg_id),
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        if not MotorReg.is_int_type(reg_id):
            # data 是浮点数
            tx_buf[4:8] = float_to_uint8s(data)
        else:
            # data 是整数
            tx_buf[4:8] = int_to_uint8s(int(data))
        self.__send_data(0x7FF, tx_buf)