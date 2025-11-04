# src/pydamiao/motor.py
from time import sleep

import numpy as np
from serial import Serial

from pydamiao.enums import ControlType
from pydamiao.logging import logger
from pydamiao.utils import (
    data_to_uint8s,
    float_to_uint,
    float_to_uint8s,
    is_in_ranges,
    uint8s_to_float,
    uint8s_to_uint32,
    uint_to_float,
)


class Motor:
    def __init__(self, motor_type, slave_id, master_id):
        """
        define Motor object 定义电机对象
        :param motor_type: Motor type 电机类型
        :param slave_id: CANID 电机ID
        :param master_id: master_id 主机ID 建议不要设为0
        """
        self.Pd = float(0)
        self.Vd = float(0)
        self.state_q = float(0)
        self.state_dq = float(0)
        self.state_tau = float(0)
        self.slave_id = slave_id
        self.master_id = master_id
        self.motor_type = motor_type
        self.is_enable = False
        self.now_control_mode = ControlType.MIT
        self.temp_param_dict = {}

    def recv_data(self, q: float, dq: float, tau: float):
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau

    def get_position(self):
        """
        get the position of the motor 获取电机位置
        :return: the position of the motor 电机位置
        """
        return self.state_q

    def get_velocity(self):
        """
        get the velocity of the motor 获取电机速度
        :return: the velocity of the motor 电机速度
        """
        return self.state_dq

    def get_torque(self):
        """
        get the torque of the motor 获取电机力矩
        :return: the torque of the motor 电机力矩
        """
        return self.state_tau

    def get_param(self, RID):
        """
        get the parameter of the motor 获取电机内部的参数，需要提前读取
        :param RID: DM_variable 电机参数
        :return: the parameter of the motor 电机参数
        """
        if RID in self.temp_param_dict:
            return self.temp_param_dict[RID]
        else:
            return None


class MotorControl:
    send_data_frame = np.array(
        object=[0x55, 0xAA, 0x1e, 0x03, 0x01, 
                0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 
                0, 0, 0, 0, 0x00, 0x08, 0x00, 0x00, 
                0, 0, 0, 0, 0, 0, 0, 0, 0x00], 
        dtype=np.uint8)  # fmt: off

    Limit_Param = [
        [12.5, 30, 10],  # 4310
        [12.5, 50, 10],  # 4310_48V
        [12.5, 8, 28],  # 4340
        [12.5, 10, 28],  # 4340_48V
        [12.5, 45, 20],  # 6006
        [12.5, 45, 40],  # 8006
        [12.5, 45, 54],  # 8009
        [12.5, 25, 200],  # 10010L
        [12.5, 20, 200],  # 10010
        [12.5, 280, 1],  # H3510
        [12.5, 45, 10],  # DMG62150
        [12.5, 45, 10],  # DMH6220
    ]

    def __init__(self, serial_device: Serial):
        """
        define MotorControl object 定义电机控制对象
        :param serial_device: serial object 串口对象
        """
        self.serial_ = serial_device
        self.motors_map = dict()
        self.data_save = bytes()  # save data
        if self.serial_.is_open:  # open the serial port
            logger.success("串口已打开")
            serial_device.close()
        self.serial_.open()

    def control_mit(
        self, dm_motor, kp: float, kd: float, q: float, dq: float, tau: float
    ):
        """
        MIT Control Mode Function 达妙电机MIT控制模式函数
        :param dm_motor: Motor object 电机对象
        :param kp: kp
        :param kd:  kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :return: None
        """
        if dm_motor.slave_id not in self.motors_map:
            logger.error("Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        motor_type = dm_motor.motor_type
        Q_MAX = self.Limit_Param[motor_type][0]
        DQ_MAX = self.Limit_Param[motor_type][1]
        TAU_MAX = self.Limit_Param[motor_type][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xFF
        data_buf[1] = q_uint & 0xFF
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        data_buf[4] = kp_uint & 0xFF
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)
        data_buf[7] = tau_uint & 0xFF
        self.__send_data(dm_motor.slave_id, data_buf)
        self.recv()  # receive the data from serial port

    def control_delay(
        self,
        dm_motor,
        kp: float,
        kd: float,
        q: float,
        dq: float,
        tau: float,
        delay: float,
    ):
        """
        MIT Control Mode Function with delay 达妙电机MIT控制模式函数带延迟
        :param dm_motor: Motor object 电机对象
        :param kp: kp
        :param kd: kd
        :param q:  position  期望位置
        :param dq:  velocity  期望速度
        :param tau: torque  期望力矩
        :param delay: delay time 延迟时间 单位秒
        """
        self.control_mit(dm_motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_pos_vel(self, dm_motor, pos_des: float, vel_des: float):
        """
        control the motor in position and velocity control mode 电机位置速度控制模式
        :param dm_motor: Motor object 电机对象
        :param pos_des: desired position 期望位置
        :param vel_des: desired velocity 期望速度
        :return: None
        """
        if dm_motor.slave_id not in self.motors_map:
            logger.error("Control Pos_Vel Error : Motor ID not found")
            return
        motorid = 0x100 + dm_motor.slave_id
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        P_desired_uint8s = float_to_uint8s(pos_des)
        V_desired_uint8s = float_to_uint8s(vel_des)
        data_buf[0:4] = P_desired_uint8s
        data_buf[4:8] = V_desired_uint8s
        self.__send_data(motorid, data_buf)
        # time.sleep(0.001)
        self.recv()  # receive the data from serial port

    def control_vel(self, dm_motor, vel_des):
        """
        control the motor in velocity control mode 电机速度控制模式
        :param dm_motor: Motor object 电机对象
        :param vel_des: desired velocity 期望速度
        """
        if dm_motor.slave_id not in self.motors_map:
            logger.error("control_VEL ERROR : Motor ID not found")
            return
        motorid = 0x200 + dm_motor.slave_id
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Vel_desired_uint8s = float_to_uint8s(vel_des)
        data_buf[0:4] = Vel_desired_uint8s
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def control_pos_force(self, dm_motor, pos_des: float, vel_des, i_des):
        """
        control the motor in EMIT control mode 电机力位混合模式
        :param pos_des: desired position rad  期望位置 单位为rad
        :param vel_des: desired velocity rad/s  期望速度 为放大100倍
        :param i_des: desired current rang 0-10000 期望电流标幺值放大10000倍
        电流标幺值：实际电流值除以最大电流值，最大电流见上电打印
        """
        if dm_motor.slave_id not in self.motors_map:
            logger.error("control_pos_vel ERROR : Motor ID not found")
            return
        motorid = 0x300 + dm_motor.slave_id
        data_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        Pos_desired_uint8s = float_to_uint8s(pos_des)
        data_buf[0:4] = Pos_desired_uint8s
        Vel_uint = np.uint16(vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xFF
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xFF
        data_buf[7] = ides_uint >> 8
        self.__send_data(motorid, data_buf)
        self.recv()  # receive the data from serial port

    def enable(self, dm_motor):
        """
        enable motor 使能电机
        最好在上电后几秒后再使能电机
        :param dm_motor: Motor object 电机对象
        """
        self.__control_cmd(dm_motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def enable_old(self, dm_motor, control_mode):
        """
        enable motor old firmware 使能电机旧版本固件，这个是为了旧版本电机固件的兼容性
        可恶的旧版本固件使能需要加上偏移量
        最好在上电后几秒后再使能电机
        :param dm_motor: Motor object 电机对象
        """
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], np.uint8)
        enable_id = ((int(control_mode) - 1) << 2) + dm_motor.slave_id
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def disable(self, dm_motor):
        """
        disable motor 失能电机
        :param dm_motor: Motor object 电机对象
        """
        self.__control_cmd(dm_motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def set_zero_position(self, dm_motor):
        """
        set the zero position of the motor 设置电机0位
        :param dm_motor: Motor object 电机对象
        """
        self.__control_cmd(dm_motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()  # receive the data from serial port

    def recv(self):
        # 把上次没有解析完的剩下的也放进来
        read_data = self.serial_.read_all()
        data_recv = b"".join(
            [self.data_save, read_data if read_data is not None else b""]
        )
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)

    def recv_set_param_data(self):
        data_recv = self.serial_.read_all()
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_set_param_packet(data, CANID, CMD)

    def __process_packet(self, data, CANID, CMD):
        if CMD == 0x11:
            if CANID != 0x00:
                if CANID in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                    motor_type_recv = self.motors_map[CANID].motor_type
                    Q_MAX = self.Limit_Param[motor_type_recv][0]
                    DQ_MAX = self.Limit_Param[motor_type_recv][1]
                    TAU_MAX = self.Limit_Param[motor_type_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau)
            else:
                master_id = data[0] & 0x0F
                if master_id in self.motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                    motor_type_recv = self.motors_map[master_id].motor_type
                    Q_MAX = self.Limit_Param[motor_type_recv][0]
                    DQ_MAX = self.Limit_Param[motor_type_recv][1]
                    TAU_MAX = self.Limit_Param[motor_type_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.motors_map[master_id].recv_data(recv_q, recv_dq, recv_tau)

    def __process_set_param_packet(self, data, CANID, CMD):
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            master_id = CANID
            slave_id = (data[1] << 8) | data[0]
            if CANID == 0x00:  # 防止有人把master_id设为0稳一手
                master_id = slave_id

            if master_id not in self.motors_map:
                if slave_id not in self.motors_map:
                    return
                else:
                    master_id = slave_id

            RID = data[3]
            # 读取参数得到的数据
            if is_in_ranges(RID):
                # uint32类型
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].temp_param_dict[RID] = num

            else:
                # float类型
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].temp_param_dict[RID] = num

    def add_motor(self, dm_motor):
        """
        add motor to the motor control object 添加电机到电机控制对象
        :param dm_motor: Motor object 电机对象
        """
        self.motors_map[dm_motor.slave_id] = dm_motor
        if dm_motor.master_id != 0:
            self.motors_map[dm_motor.master_id] = dm_motor
        return True

    def __control_cmd(self, dm_motor, cmd: np.uint8):
        data_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd], np.uint8)
        self.__send_data(dm_motor.slave_id, data_buf)

    def __send_data(self, motor_id, data):
        """
        send data to the motor 发送数据到电机
        :param motor_id:
        :param data:
        :return:
        """
        self.send_data_frame[13] = motor_id & 0xFF
        self.send_data_frame[14] = (motor_id >> 8) & 0xFF  # id high 8 bits
        self.send_data_frame[21:29] = data
        self.serial_.write(bytes(self.send_data_frame.T))

    def __read_RID_param(self, dm_motor, RID):
        can_id_l = dm_motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (dm_motor.slave_id >> 8) & 0xFF  # id high 8 bits
        data_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0x33,
                np.uint8(RID),
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, dm_motor, RID, data):
        can_id_l = dm_motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (dm_motor.slave_id >> 8) & 0xFF  # id high 8 bits
        data_buf = np.array(
            [
                np.uint8(can_id_l),
                np.uint8(can_id_h),
                0x55,
                np.uint8(RID),
                0x00,
                0x00,
                0x00,
                0x00,
            ],
            np.uint8,
        )
        if not is_in_ranges(RID):
            # data is float
            data_buf[4:8] = float_to_uint8s(data)
        else:
            # data is int
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switch_control_mode(self, dm_motor, control_mode):
        """
        switch the control mode of the motor 切换电机控制模式
        :param dm_motor: Motor object 电机对象
        :param control_mode: Control_Type 电机控制模式 example:MIT:Control_Type.MIT MIT模式
        """
        max_retries = 20
        retry_interval = 0.1  # retry times
        RID = 10
        self.__write_motor_param(dm_motor, RID, np.uint8(control_mode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if dm_motor.slave_id in self.motors_map:
                if RID in self.motors_map[dm_motor.slave_id].temp_param_dict:
                    if (
                        abs(
                            self.motors_map[dm_motor.slave_id].temp_param_dict[RID]
                            - control_mode
                        )
                        < 0.1
                    ):
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, dm_motor):
        """
        save the all parameter  to flash 保存所有电机参数
        :param dm_motor: Motor object 电机对象
        :return:
        """
        can_id_l = dm_motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (dm_motor.slave_id >> 8) & 0xFF  # id high 8 bits
        data_buf = np.array(
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
        self.disable(dm_motor)  # before save disable the motor
        self.__send_data(0x7FF, data_buf)
        sleep(0.001)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        change the PMAX VMAX TMAX of the motor 改变电机的PMAX VMAX TMAX
        :param Motor_Type:
        :param PMAX: 电机的PMAX
        :param VMAX: 电机的VMAX
        :param TMAX: 电机的TMAX
        :return:
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, dm_motor):
        """
        get the motor status 获得电机状态
        """
        can_id_l = dm_motor.slave_id & 0xFF  # id low 8 bits
        can_id_h = (dm_motor.slave_id >> 8) & 0xFF  # id high 8 bits
        data_buf = np.array(
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
        self.__send_data(0x7FF, data_buf)
        self.recv()  # receive the data from serial port

    def change_motor_param(self, dm_motor, RID, data):
        """
        change the RID of the motor 改变电机的参数
        :param dm_motor: Motor object 电机对象
        :param RID: DM_variable 电机参数
        :param data: 电机参数的值
        :return: True or False ,True means success, False means fail
        """
        max_retries = 20
        retry_interval = 0.05  # retry times

        self.__write_motor_param(dm_motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if (
                dm_motor.slave_id in self.motors_map
                and RID in self.motors_map[dm_motor.slave_id].temp_param_dict
            ):
                if (
                    abs(self.motors_map[dm_motor.slave_id].temp_param_dict[RID] - data)
                    < 0.1
                ):
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, dm_motor, RID):
        """
        read only the RID of the motor 读取电机的内部信息例如 版本号等
        :param dm_motor: Motor object 电机对象
        :param RID: DM_variable 电机参数
        :return: 电机参数的值
        """
        max_retries = 20
        retry_interval = 0.05  # retry times
        self.__read_RID_param(dm_motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if dm_motor.slave_id in self.motors_map:
                if RID in self.motors_map[dm_motor.slave_id].temp_param_dict:
                    return self.motors_map[dm_motor.slave_id].temp_param_dict[RID]
        return None

    # -------------------------------------------------
    # Extract packets from the serial data
    def __extract_packets(self, data):
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
        self.data_save = data[remainder_pos:]
        return frames
