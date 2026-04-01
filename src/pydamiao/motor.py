# src/pydamiao/motor.py
from time import sleep

import numpy as np
from serial import Serial

from pydamiao.types import Hex, ControlMode, MotorType, MotorReg, MotorLimits, MOTOR_LIMITS
from pydamiao.utils import (
    int_to_uint8s,
    float_to_uint,
    float_to_uint8s,
    is_in_ranges,
    uint8s_to_float,
    uint8s_to_uint32,
    uint_to_float,
)

class Motor:
    def __init__(self, motor_type: MotorType, slave_id: Hex, master_id: Hex):
        """
        define Motor object 定义电机对象
        :param motor_type: Motor type 电机类型
        :param slave_id: CANID 电机ID
        :param master_id: master_id 主机ID 建议不要设为0
        """
        self.pos_cmd = float(0)
        self.vel_cmd = float(0)
        self.tau_cmd = float(0)

        self.pos = float(0)
        self.vel = float(0)
        self.tau = float(0)

        self.slave_id = slave_id
        self.master_id = master_id

        self.motor_type = motor_type
        self.is_enable = False
        self.control_mode = ControlMode.MIT
        self.param_cache = {}

    def recv_data(self, pos: float, vel: float, tau: float):
        self.pos = pos
        self.vel = vel
        self.tau = tau

    def get_position(self):
        """
        get the position of the motor 获取电机位置
        :return: the position of the motor 电机位置
        """
        return self.pos

    def get_velocity(self):
        """
        get the velocity of the motor 获取电机速度
        :return: the velocity of the motor 电机速度
        """
        return self.vel

    def get_torque(self):
        """
        get the torque of the motor 获取电机力矩
        :return: the torque of the motor 电机力矩
        """
        return self.tau

    def get_param(self, reg_id: MotorReg):
        """
        get the parameter of the motor 获取电机内部的参数，需要提前读取
        :param reg_id: DM_variable 电机参数
        :return: the parameter of the motor 电机参数
        """
        if reg_id in self.param_cache:
            return self.param_cache[reg_id]
        else:
            return None


class MotorControl:
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
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.serial = serial_device
        self.motors_map: dict[Hex, Motor] = dict()
        self.rx_buf = bytes()  # save data
        if self.serial.is_open:  # open the serial port
            print("Serial port is open")
            serial_device.close()
        self.serial.open()

    def add_motor(self, motor: Motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param motor: Motor object 电机对象
        """
        self.motors_map[motor.slave_id] = motor
        if motor.master_id != 0:
            self.motors_map[motor.master_id] = motor
        return True


    # ==============================================================================
    # 基础控制命令
    # ==============================================================================

    def enable(self, motor: Motor):
        """
        enable motor 使能电机
        最好在上电后几秒后再使能电机
        :param motor: Motor object 电机对象
        """
        self.__control_cmd(motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def enable_old(self, motor: Motor, control_mode: ControlMode):
        """
        enable motor old firmware 使能电机旧版本固件，这个是为了旧版本电机固件的兼容性
        可恶的旧版本固件使能需要加上偏移量
        最好在上电后几秒后再使能电机
        :param motor: Motor object 电机对象
        """
        tx_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        enable_id = ((int(control_mode) - 1) << 2) + motor.slave_id
        self.__send_data(enable_id, tx_buf)
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def disable(self, motor: Motor):
        """
        disable motor 失能电机
        :param motor: Motor object 电机对象
        """
        self.__control_cmd(motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def set_zero_position(self, motor: Motor):
        """
        set the zero position of the motor 设置电机0位
        :param motor: Motor object 电机对象
        """
        self.__control_cmd(motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()  # receive the data from serial port


    # ==============================================================================
    # 控制模式函数
    # ==============================================================================

    def control_mit(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, tau_cmd: float
    ):
        """
        MIT Control Mode Function 达妙电机MIT控制模式函数
        :param DM_Motor: Motor object 电机对象
        :param kp: kp
        :param kd:  kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :return: None
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
        TAU_MAX = limits.TAU_MAX
        q_uint = float_to_uint(pos_cmd, -POS_MAX, POS_MAX, 16)
        dq_uint = float_to_uint(vel_cmd, -VEL_MAX, VEL_MAX, 12)
        tau_uint = float_to_uint(tau_cmd, -TAU_MAX, TAU_MAX, 12)
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        tx_buf[0] = (q_uint >> 8) & 0xFF
        tx_buf[1] = q_uint & 0xFF
        tx_buf[2] = dq_uint >> 4
        tx_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        tx_buf[4] = kp_uint & 0xFF
        tx_buf[5] = kd_uint >> 4
        tx_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
        tx_buf[7] = tau_uint & 0xFF
        self.__send_data(motor.slave_id, tx_buf)
        self.recv()  # receive the data from serial port

    def control_mit_delay(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, tau_cmd: float,
        delay: float,
    ):
        """
        MIT Control Mode Function with delay 达妙电机MIT控制模式函数带延迟
        :param DM_Motor: Motor object 电机对象
        :param kp: kp
        :param kd: kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :param delay: delay time 延迟时间 单位秒
        """
        self.control_mit(motor, kp, kd, pos_cmd, vel_cmd, tau_cmd)
        sleep(delay)

    def control_pos_vel(self, motor: Motor, pos_cmd: float, vel_cmd: float):
        """
        control the motor in position and velocity control mode 电机位置速度控制模式
        :param motor: Motor object 电机对象
        :param pos_cmd: desired position 期望位置
        :param vel_cmd: desired velocity 期望速度
        :return: None
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
        self.recv()  # receive the data from serial port

    def control_vel(self, motor: Motor, vel_cmd: float):
        """
        control the motor in velocity control mode 电机速度控制模式
        :param motor: Motor object 电机对象
        :param vel_cmd: desired velocity 期望速度
        """
        if motor.slave_id not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motor_id = 0x200 + motor.slave_id
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        vel_cmd_u8s = float_to_uint8s(vel_cmd)
        tx_buf[0:4] = vel_cmd_u8s
        self.__send_data(motor_id, tx_buf)
        self.recv()  # receive the data from serial port

    def control_pos_force(self, motor: Motor, pos_cmd: float, vel_cmd: float, cur_cmd: float):
        """
        control the motor in EMIT control mode 电机力位混合模式
        :param pos_cmd: desired position rad  期望位置 单位为rad
        :param vel_cmd: desired velocity rad/s  期望速度 为放大100倍
        :param cur_cmd: desired current rang 0-10000 期望电流标幺值放大10000倍
        电流标幺值：实际电流值除以最大电流值，最大电流见上电打印
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
        self.recv()  # receive the data from serial port


    # ==============================================================================
    # 参数管理
    # ==============================================================================


    # ==============================================================================
    # 数据收发与处理（底层辅助）
    # ==============================================================================

    def recv(self):
        """
        receive the data from serial port 从串口接收数据
        """

        read_data = self.serial.read_all()   # 把上次没有解析完的剩下的也放进来
        data_recv = b"".join(
            [self.rx_buf, read_data if read_data is not None else b""]
        )
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        """
        receive the data from serial port from serial port 从串口接收设置参数数据
        """
        data_recv = self.serial.read_all()
        if data_recv is None:
            data_recv = b""
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __extract_packets(self, data: bytes) -> list[bytes]:
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

    def __process_packet(self, data: bytes, can_id: Hex, CMD: Hex) -> None:
        if CMD == 0x11:
            # Determine target motor ID
            if can_id != 0x00:
                target_id = can_id
            else:
                target_id = data[0] & 0x0F

            # Process only if target motor exists
            if target_id in self.motors_map:
                pos_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                vel_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                motor_type_recv = self.motors_map[target_id].motor_type
                limits = MOTOR_LIMITS[motor_type_recv]
                POS_MAX = limits.POS_MAX
                VEL_MAX = limits.VEL_MAX
                TAU_MAX = limits.TAU_MAX
                recv_pos = uint_to_float(pos_uint, -POS_MAX, POS_MAX, 16)
                recv_vel = uint_to_float(vel_uint, -VEL_MAX, VEL_MAX, 12)
                recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                self.motors_map[target_id].recv_data(
                    float(recv_pos), float(recv_vel), float(recv_tau)
                )
    def __process_set_param_packet(self, data: bytes, can_id: Hex, CMD: Hex) -> None:
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
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
        """
        send data to the motor 发送数据到电机
        :param motor_id:
        :param data:
        :return:
        """
        self.tx_frame[13] = motor_id & 0xFF
        self.tx_frame[14] = (motor_id >> 8) & 0xFF  # id high 8 bits
        self.tx_frame[21:29] = data
        self.serial.write(bytes(self.tx_frame.T))

    def __control_cmd(self, motor: Motor, cmd: np.uint8):
        tx_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd], np.uint8)
        self.__send_data(motor.slave_id, tx_buf)

    def __read_motor_param(self, motor: Motor, reg_id: MotorReg):
        can_id_l = motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id high 8 bits
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
        can_id_l = motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id high 8 bits
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
            # data is float
            tx_buf[4:8] = float_to_uint8s(data)
        else:
            # data is int
            tx_buf[4:8] = int_to_uint8s(int(data))
        self.__send_data(0x7FF, tx_buf)


    def refresh_motor_status(self, motor: Motor):
        """
        get the motor status 获得电机状态
        """
        can_id_l = motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id high 8 bits
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
        self.__send_data(0x7FF, tx_buf)
        self.recv()  # receive the data from serial port

    def read_motor_param(self, motor: Motor, reg_id: MotorReg):
        """
        read only the reg_id of the motor 读取电机的内部信息例如 版本号等
        :param motor: Motor object 电机对象
        :param reg_id: DM_variable 电机参数
        :return: 电机参数的值
        """
        max_retries = 20
        retry_interval = 0.05  # retry times
        self.__read_motor_param(motor, reg_id)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if motor.slave_id in self.motors_map:
                if reg_id in self.motors_map[motor.slave_id].param_cache:
                    return self.motors_map[motor.slave_id].param_cache[reg_id]
        return None

    def change_motor_param(self, motor: Motor, reg_id: MotorReg, data: float | int):
        """
        change the reg_id of the motor 改变电机的参数
        :param motor: Motor object 电机对象
        :param reg_id: DM_variable 电机参数
        :param data: 电机参数的值
        :return: True or False ,True means success, False means fail
        """
        max_retries = 20
        retry_interval = 0.05  # retry times

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
        """
        save the all parameter to flash 保存所有电机参数
        :param motor: Motor object 电机对象
        :return:
        """
        can_id_l = motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (motor.slave_id >> 8) & 0xFF  # id high 8 bits
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
        self.disable(motor)  # before save disable the motor
        self.__send_data(0x7FF, tx_buf)
        sleep(0.001)

    def change_limit_param(self, motor_type: MotorType, PMAX: float, VMAX: float, TMAX: float):
        """
        change the PMAX VMAX TMAX of the motor 改变电机的PMAX VMAX TMAX
        :param motor_type:
        :param PMAX: 电机的PMAX
        :param VMAX: 电机的VMAX
        :param TMAX: 电机的TMAX
        :return:
        """
        MOTOR_LIMITS[motor_type] = MotorLimits(PMAX, VMAX, TMAX)
        
    def switch_control_mode(self, motor: Motor, control_mode: ControlMode):
        """
        switch the control mode of the motor 切换电机控制模式
        :param motor: Motor object 电机对象
        :param ControlMode: Control_Type 电机控制模式 example:MIT:Control_Type.MIT MIT模式
        """
        max_retries = 20
        retry_interval = 0.1  # retry times
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
                        return True
                    else:
                        return False
        return False