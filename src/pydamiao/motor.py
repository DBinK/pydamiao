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
    """Represents a motor object for control and monitoring."""
    
    def __init__(self, motor_type: MotorType, slave_id: Hex, master_id: Hex):
        """Initializes a Motor object.
        
        Args:
            motor_type: Type of the motor.
            slave_id: CAN ID of the motor.
            master_id: Master ID (host ID, recommended not to be 0).
        """
        self.pos_cmd = float(0)
        self.vel_cmd = float(0)
        self.torque_cmd = float(0)

        self.pos = float(0)
        self.vel = float(0)
        self.torque = float(0)

        self.slave_id = slave_id
        self.master_id = master_id

        self.motor_type = motor_type
        self.is_enable = False
        self.control_mode = ControlMode.MIT
        self.param_cache = {}

    def update_from_controller(self, pos: float, vel: float, torque: float):
        """Updates motor state from controller data.
        
        Args:
            pos: Position value.
            vel: Velocity value.
            torque: Torque value.
        """
        self.pos = pos
        self.vel = vel
        self.torque = torque

    def get_position(self) -> float:
        """Gets the current position of the motor.
        
        Returns:
            Current motor position.
        """
        return self.pos

    def get_velocity(self) -> float:
        """Gets the current velocity of the motor.
        
        Returns:
            Current motor velocity.
        """
        return self.vel

    def get_torque(self) -> float:
        """Gets the current torque of the motor.
        
        Returns:
            Current motor torque.
        """
        return self.torque

    def get_param(self, reg_id: MotorReg):
        """Gets a parameter from the motor's parameter cache.
        
        Args:
            reg_id: Register ID of the parameter.
            
        Returns:
            Parameter value if exists in cache, None otherwise.
        """
        if reg_id in self.param_cache:
            return self.param_cache[reg_id]
        else:
            return None


class MotorController:
    """Controls multiple motors through a serial interface."""
    
    tx_frame = np.array(
        object=[
            0x55, 0xAA, # Frame header
            0x1e,       # Frame length
            0x03,       # Command
            0x01, 0x00, 0x00, 0x00,  # Send count			
            0x0A, 0x00, 0x00, 0x00,  # Time interval			
            0x00,                    # ID type: 0x00 standard frame, 0x01 extended frame 
            0x00, 0x00, 0x00, 0x00,  # CAN ID			
            0x00,                    # Frame type: 0x00 data frame, 0x01 remote frame
            0x08,       # Data length
            0x00,       # idAcc
            0x00,       # dataAcc
            0, 0, 0, 0, 0, 0, 0, 0,  # data[len]	
            0x00        # CRC checksum (not validated at low level)
        ], 
        dtype=np.uint8)

    def __init__(self, serial_device: Serial):
        """Initializes a MotorController object.
        
        Args:
            serial_device: Serial port object for communication.
        """
        self.serial = serial_device
        self.motors_map: dict[Hex, Motor] = dict()
        self.rx_buf = bytes()  # Buffer for received data
        if self.serial.is_open:
            print("Serial port is open")
            serial_device.close()
        self.serial.open()

    def add_motor(self, motor: Motor) -> bool:
        """Adds a motor to the controller.
        
        Args:
            motor: Motor object to add.
            
        Returns:
            True if motor was added successfully.
        """
        self.motors_map[motor.slave_id] = motor
        if motor.master_id != 0:
            self.motors_map[motor.master_id] = motor
        return True

    def enable(self, motor: Motor):
        """Enables the motor.
        
        Note:
            Best to enable motor several seconds after power-on.
            
        Args:
            motor: Motor object to enable.
        """
        self.__basic_cmd(motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()

    def disable(self, motor: Motor):
        """Disables the motor.
        
        Args:
            motor: Motor object to disable.
        """
        self.__basic_cmd(motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()

    def set_zero_position(self, motor: Motor):
        """Sets the zero position of the motor.
        
        Args:
            motor: Motor object to set zero position.
        """
        self.__basic_cmd(motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()

    def control_mit(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float
    ):
        """Controls motor in MIT mode.
        
        Args:
            motor: Motor object to control.
            kp: Proportional gain.
            kd: Derivative gain.
            pos_cmd: Desired position.
            vel_cmd: Desired velocity.
            torque_cmd: Desired torque.
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
        self.recv()

    def control_mit_delay(
        self, motor: Motor, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float,
        delay: float,
    ):
        """Controls motor in MIT mode with delay.
        
        Args:
            motor: Motor object to control.
            kp: Proportional gain.
            kd: Derivative gain.
            pos_cmd: Desired position.
            vel_cmd: Desired velocity.
            torque_cmd: Desired torque.
            delay: Delay time in seconds.
        """
        self.control_mit(motor, kp, kd, pos_cmd, vel_cmd, torque_cmd)
        sleep(delay)

    def control_pos_vel(self, motor: Motor, pos_cmd: float, vel_cmd: float):
        """Controls motor in position and velocity mode.
        
        Args:
            motor: Motor object to control.
            pos_cmd: Desired position.
            vel_cmd: Desired velocity.
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
        self.recv()

    def control_vel(self, motor: Motor, vel_cmd: float):
        """Controls motor in velocity mode.
        
        Args:
            motor: Motor object to control.
            vel_cmd: Desired velocity.
        """
        if motor.slave_id not in self.motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        motor_id = 0x200 + motor.slave_id
        tx_buf = np.array([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], np.uint8)
        vel_cmd_u8s = float_to_uint8s(vel_cmd)
        tx_buf[0:4] = vel_cmd_u8s
        self.__send_data(motor_id, tx_buf)
        self.recv()

    def control_pos_force(self, motor: Motor, pos_cmd: float, vel_cmd: float, cur_cmd: float):
        """Controls motor in EMIT (position-force) mode.
        
        Args:
            motor: Motor object to control.
            pos_cmd: Desired position in radians.
            vel_cmd: Desired velocity (scaled by 100).
            cur_cmd: Desired current (scaled by 10000, normalized to max current).
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
        self.recv()

    def refresh_motor_status(self, motor: Motor):
        """Refreshes motor status by requesting current state.
        
        Args:
            motor: Motor object to refresh.
        """
        can_id_l = motor.slave_id & 0xFF
        can_id_h = (motor.slave_id >> 8) & 0xFF
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
        self.recv()

    def read_motor_param(self, motor: Motor, reg_id: MotorReg):
        """Reads a specific parameter from the motor.
        
        Args:
            motor: Motor object to read from.
            reg_id: Register ID of the parameter to read.
            
        Returns:
            Parameter value if successful, None otherwise.
        """
        max_retries = 20
        retry_interval = 0.05
        self.__read_motor_param(motor, reg_id)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if motor.slave_id in self.motors_map:
                if reg_id in self.motors_map[motor.slave_id].param_cache:
                    return self.motors_map[motor.slave_id].param_cache[reg_id]
        return None

    def change_motor_param(self, motor: Motor, reg_id: MotorReg, data: float | int):
        """Changes a specific parameter of the motor.
        
        Args:
            motor: Motor object to modify.
            reg_id: Register ID of the parameter to change.
            data: New parameter value.
            
        Returns:
            True if successful, False otherwise.
        """
        max_retries = 20
        retry_interval = 0.05

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
        """Saves all motor parameters to flash memory.
        
        Args:
            motor: Motor object whose parameters should be saved.
        """
        can_id_l = motor.slave_id & 0xFF
        can_id_h = (motor.slave_id >> 8) & 0xFF
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
        self.disable(motor)
        self.__send_data(0x7FF, tx_buf)
        sleep(0.001)

    def change_limit_param(self, motor_type: MotorType, PMAX: float, VMAX: float, TMAX: float):
        """Changes the PMAX, VMAX, TMAX limits for a motor type.
        
        Args:
            motor_type: Type of motor to update limits for.
            PMAX: New position maximum.
            VMAX: New velocity maximum.
            TMAX: New torque maximum.
        """
        MOTOR_LIMITS[motor_type] = MotorLimits(PMAX, VMAX, TMAX)
        
    def switch_control_mode(self, motor: Motor, control_mode: ControlMode):
        """Switches the control mode of the motor.
        
        Args:
            motor: Motor object to switch mode.
            control_mode: New control mode.
            
        Returns:
            True if successful, False otherwise.
        """
        max_retries = 20
        retry_interval = 0.1
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

    def recv(self):
        """Receives and processes data from the serial port."""
        read_data = self.serial.read_all()
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
        """Receives and processes parameter data from the serial port."""
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
        """Extracts complete packets from raw data stream.
        
        Args:
            data: Raw byte data from serial port.
            
        Returns:
            List of complete packets.
        """
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
        """Processes received motor status packets.
        
        Args:
            data: Packet data payload.
            can_id: CAN identifier.
            can_cmd: CAN response command.
        """
        if can_cmd == CanResp.RECEIVE_SUCCESS:
            # Determine target motor ID
            if can_id != 0x00:
                target_id = can_id
            else:
                target_id = data[0] & 0x0F

            # Process only if target motor exists
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
        """Processes received parameter setting packets.
        
        Args:
            data: Packet data payload.
            can_id: CAN identifier.
            can_cmd: CAN response command.
        """
        if can_cmd == CanResp.RECEIVE_SUCCESS and (data[2] == 0x33 or data[2] == 0x55):
            master_id = can_id
            slave_id = (data[1] << 8) | data[0]
            if can_id == 0x00:
                master_id = slave_id

            if master_id not in self.motors_map:
                if slave_id not in self.motors_map:
                    return
                else:
                    master_id = slave_id

            reg_id = data[3]

            # Process parameter data
            if MotorReg.is_int_type(reg_id):
                # uint32 type
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].param_cache[reg_id] = num
            else:
                # float type
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
                self.motors_map[master_id].param_cache[reg_id] = num

    def __send_data(self, motor_id: Hex, data: np.ndarray) -> None:
        """Sends data to the motor via serial interface.
        
        Args:
            motor_id: Target motor ID.
            data: Data payload to send.
        """
        self.tx_frame[13] = motor_id & 0xFF
        self.tx_frame[14] = (motor_id >> 8) & 0xFF
        self.tx_frame[21:29] = data
        self.serial.write(bytes(self.tx_frame.T))

    def __basic_cmd(self, motor: Motor, state_cmd: np.uint8):
        """Sends basic commands to the motor.
        
        Args:
            motor: Target motor object.
            state_cmd: Command byte to send.
        """
        tx_buf = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, state_cmd], np.uint8)
        self.__send_data(motor.slave_id, tx_buf)

    def __read_motor_param(self, motor: Motor, reg_id: MotorReg):
        """Sends a parameter read request to the motor.
        
        Args:
            motor: Target motor object.
            reg_id: Register ID to read.
        """
        can_id_l = motor.slave_id & 0xFF
        can_id_h = (motor.slave_id >> 8) & 0xFF
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
        """Sends a parameter write request to the motor.
        
        Args:
            motor: Target motor object.
            reg_id: Register ID to write.
            data: New parameter value.
        """
        can_id_l = motor.slave_id & 0xFF
        can_id_h = (motor.slave_id >> 8) & 0xFF
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