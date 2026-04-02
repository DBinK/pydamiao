
import atexit
import threading
import time

from pydamiao.bus import SerialBus
from pydamiao.protocol import DamiaoProtocol, ParsedMessage
from pydamiao.result import Result
from pydamiao.structs import (
    MOTOR_LIMITS,
    ControlMode,
    MotorFault,
    MotorId,
    MotorLimits,
    MotorReg,
    MotorState,
    MotorType,
    RegId,
)


class Motor:
    """共享总线上的单个达妙电机高层控制对象。"""

    def __init__(
        self,
        bus: SerialBus,
        motor_type: MotorType,
        slave_id: MotorId,
        master_id: MotorId = 0,
        name: str | None = None,
        auto_disable: bool = True,
    ) -> None:
        """初始化绑定到共享串口总线的电机对象。

        Args:
            bus: 用于通信的共享串口总线。
            motor_type: 用于选择协议限制的电机型号。
            slave_id: 电机接收 id。
            master_id: 某些固件配置下使用的可选反馈 id。
            name: 可选的电机名称。
            auto_disable: 是否在电机对象被销毁时自动禁用电机。
        """
        # 通讯总线
        self.bus = bus

        # 对象属性
        self.motor_type = motor_type
        self.slave_id = slave_id
        self.master_id = master_id
        self.name = name
        self.auto_disable = auto_disable

        # 可从电机中读取的状态
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0

        self.mos_temp: int | None = None
        self.rotor_temp: int | None = None

        self.fault: MotorFault | None = None
        self.param_cache: dict[RegId, float | int] = {}  # 可能有未知的寄存器

        # 手动维护的状态
        self.enabled = False
        self.control_mode: ControlMode | None = None
        self.last_update_time: float | None = None

        # 状态锁
        self._state_lock = threading.RLock()

        # 注册电机通讯总线
        self.bus.register_motor(self)

        # 注册退出处理函数
        atexit.register(self._cleanup)

    def _cleanup(self):
        """程序退出时的清理函数, 确保电机失能"""
        try:
            if self.enabled and self.auto_disable:
                self.disable()
        except Exception:
            pass  # 忽略清理过程中的异常，避免影响程序正常退出


    # ===========================================================================
    # 电机对象的 Getter 方法
    # ===========================================================================

    @property
    def alias_ids(self) -> tuple[int, ...]:
        """返回所有可能把消息路由到本电机的 id。"""
        if self.master_id != 0: 
            return self.slave_id, self.master_id
        return (self.slave_id,)

    @property
    def limits(self) -> MotorLimits:
        """返回当前电机型号对应的协议限制。"""
        return MOTOR_LIMITS[self.motor_type]

    def get_state(self) -> MotorState:
        """返回当前缓存中的最新电机状态。"""
        with self._state_lock:
            return MotorState(pos=self.pos, vel=self.vel, torque=self.torque)

    def get_position(self) -> float:
        """返回当前缓存中的最新位置。"""
        with self._state_lock:
            return self.pos

    def get_velocity(self) -> float:
        """返回当前缓存中的最新速度。"""
        with self._state_lock:
            return self.vel

    def get_torque(self) -> float:
        """返回当前缓存中的最新力矩。"""
        with self._state_lock:
            return self.torque

    def get_mos_temp(self) -> int | None:
        """返回当前缓存中的 MOS 温度。"""
        with self._state_lock:
            return self.mos_temp

    def get_rotor_temp(self) -> int | None:
        """返回当前缓存中的转子温度。"""
        with self._state_lock:
            return self.rotor_temp

    def get_param(self, reg_id: RegId) -> float | int | None:
        """返回缓存中的寄存器值；如果尚未读取则返回 ``None``。"""
        with self._state_lock:
            return self.param_cache.get(reg_id)


    # ===========================================================================
    # 基础控制函数
    # ===========================================================================

    def enable(self) -> Result[None]:
        """使能电机。"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.ENABLE_CMD))
        with self._state_lock:
            self.enabled = True
        return Result.ok()

    def disable(self, timeout_sec=3) -> Result[None]:
        """失能电机，并等待速度降到很小的阈值以下。"""
        deadline = time.monotonic() + timeout_sec
        command = DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.DISABLE_CMD)

        while True:  # (安全性) 尝试失能电机, 直到速度降到阈值以下 (达妙没有失能/失能的状态反馈)
            # 超时检测
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return Result.err("Motor did not slow down to the target threshold in time", code="disable_timeout")

            # 继续发送失能命令
            result = self._request_feedback(command, timeout=min(0.05, remaining))
            
            # 检查速度和力矩是否都降到阈值以下
            if result.is_ok and (abs(self.vel) <= 0.02 and abs(self.torque) <= 0.02):
                with self._state_lock:
                    self.enabled = False  # 修改状态
                return Result.ok()

    def set_zero(self) -> Result[None]:
        """把当前位置设置为电机零点。"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.SET_ZERO_CMD))
        return Result.ok()

    def clear_error(self) -> Result[None]:
        """清除错误 (过热等错误)"""
        self.bus.send(DamiaoProtocol.encode_basic_command(self.slave_id, DamiaoProtocol.CLEAN_ERROR_CMD))
        with self._state_lock:
            self.fault = MotorFault.NONE
        return Result.ok()


    # ===========================================================================
    # 运动控制函数
    # ===========================================================================

    def set_mode(self, mode: ControlMode, timeout: float = 1.0) -> Result[ControlMode]:
        """切换电机控制模式。

        Args:
            mode: 目标控制模式。
            timeout: 等待写入确认的最大时间。
        """
        result = self.write_param(MotorReg.CTRL_MODE, int(mode), timeout=timeout)
        if not result:
            return Result.err(result.error or "Failed to switch mode", code=result.code or "error")

        with self._state_lock:
            self.control_mode = mode
        return Result.ok(mode)

    def set_mit(
        self,
        pos: float,
        vel: float,
        kp: float,
        kd: float,
        torque: float,
        auto_mode: bool = True,
    ) -> Result[None]:
        """发送 MIT 控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 目标速度，单位 rad/s。
            kp: 位置增益。
            kd: 速度增益。
            torque: 前馈力矩命令。
            auto_mode: 是否自动切换到 MIT 模式。
        """
        fault_result = self._ensure_no_fault()
        if not fault_result:
            return fault_result
        mode_result = self._prepare_control_mode(ControlMode.MIT, auto_mode)
        if not mode_result:
            return mode_result
        self.bus.send(
            DamiaoProtocol.encode_mit_control(
                self.slave_id,
                self.limits,
                kp=kp,
                kd=kd,
                pos=pos,
                vel=vel,
                torque=torque,
            )
        )
        return Result.ok()

    def set_pos_vel(self, pos: float, vel: float, auto_mode: bool = True) -> Result[None]:
        """发送位置速度控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 目标速度，单位 rad/s。
            auto_mode: 是否自动切换到 POS_VEL 模式。
        """
        fault_result = self._ensure_no_fault()
        if not fault_result:
            return fault_result
        mode_result = self._prepare_control_mode(ControlMode.POS_VEL, auto_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_pos_vel_control(self.slave_id, pos, vel))
        return Result.ok()

    def set_velocity(self, vel: float, auto_mode: bool = True) -> Result[None]:
        """发送速度控制命令。

        Args:
            vel: 目标速度，单位 rad/s。
            auto_mode: 是否自动切换到 VEL 模式。
        """
        fault_result = self._ensure_no_fault()
        if not fault_result:
            return fault_result
        mode_result = self._prepare_control_mode(ControlMode.VEL, auto_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_velocity_control(self.slave_id, vel))
        return Result.ok()

    def set_pos_force(
        self,
        pos: float,
        vel: float,
        current: float,
        auto_mode: bool = True,
    ) -> Result[None]:
        """发送力位混合控制命令。

        Args:
            pos: 目标位置，单位 rad。
            vel: 电机协议要求的辅助速度项。
            current: 电机协议要求的电流或力矩项。
            auto_mode: 是否自动切换到 TORQUE_POS 模式。
        """
        fault_result = self._ensure_no_fault()
        if not fault_result:
            return fault_result
        mode_result = self._prepare_control_mode(ControlMode.TORQUE_POS, auto_mode)
        if not mode_result:
            return mode_result
        self.bus.send(DamiaoProtocol.encode_pos_force_control(self.slave_id, pos, vel, current))
        return Result.ok()

    
    # ===========================================================================
    # 读写 电机状态/寄存器参数
    # ===========================================================================

    def refresh_state(self, timeout: float = 0.5) -> Result[MotorState]:
        """请求最新状态，并返回更新后的缓存结果。"""
        result = self.bus.request(
            DamiaoProtocol.encode_refresh_state(self.slave_id),
            matcher=lambda message: message.kind == "status" and self._matches_message(message),
            timeout=timeout,
        )
        if not result:
            return Result.err(result.error or "Failed to refresh state", code=result.code or "error")
        return Result.ok(self.get_state())

    def read_param(self, reg_id: MotorReg, timeout: float = 0.1) -> Result[float | int]:
        """读取电机寄存器。

        Args:
            reg_id: 要读取的寄存器 id。
            timeout: 等待响应的最大时间。
        """
        result = self.bus.request(
            DamiaoProtocol.encode_read_param(self.slave_id, reg_id),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result:
            return Result.err(result.error or "Failed to read motor parameter", code=result.code or "error")
        if result.value is None:
            return Result.err("Motor did not return a parameter value", code="invalid_response")
        return Result.ok(result.value.value)

    def write_param(self, reg_id: MotorReg, value: float | int, timeout: float = 1.0) -> Result[float | int]:
        """写入电机寄存器，并校验回读响应。

        Args:
            reg_id: 要写入的寄存器 id。
            value: 要发送给电机的值。
            timeout: 等待响应的最大时间。
        """
        result = self.bus.request(
            DamiaoProtocol.encode_write_param(self.slave_id, reg_id, value),
            matcher=lambda message: (
                message.kind == "param"
                and message.reg_id == int(reg_id)
                and self._matches_message(message)
            ),
            timeout=timeout,
        )
        if not result:
            return Result.err(result.error or "Failed to write motor parameter", code=result.code or "error")
        if result.value is None:
            return Result.err("Motor did not return a parameter value", code="invalid_response")

        returned = result.value.value
        if returned is None:
            return Result.err("Motor did not return a parameter value", code="invalid_response")

        if isinstance(returned, float):
            if abs(returned - float(value)) > 0.1:
                return Result.err("Motor parameter verification failed", code="mismatch")
        elif returned != int(value):
            return Result.err("Motor parameter verification failed", code="mismatch")

        if int(reg_id) == int(MotorReg.CTRL_MODE):
            with self._state_lock:
                self.control_mode = ControlMode(int(returned))
        return Result.ok(returned)

    def save_params(self, disable: bool=False) -> Result[None]:
        """ 把当前参数集保存到电机 flash
            Args:
                disable: 是否在保存参数时自动失能电机 (仅失能是可以保存参数)
            Note:
                1. 存储参数只在失能模式下生效。
                2. 存储参数时将一次性保留全部参数。
                3. 该操作将参数写入片内flash中，每次操作时间最大为30ms，请注意留足足够的时间。
                4. flash擦写次数约1万次，请不要频繁发送“存储参数”指令。
        """
        if disable:
            self.disable()
        self.bus.send(DamiaoProtocol.encode_save_params(self.slave_id))
        return Result.ok()


    # ===========================================================================
    # 辅助函数
    # ===========================================================================

    def _prepare_control_mode(self, target_mode: ControlMode, auto_mode: bool) -> Result[None]:
        if self.control_mode == target_mode:
            return Result.ok()
        if not auto_mode:
            return Result.err(f"Motor is not in {target_mode.name} mode", code="mode_mismatch")

        result = self.set_mode(target_mode)
        if not result:
            return Result.err(result.error or "Failed to switch control mode", code=result.code or "error")
        return Result.ok()

    def _ensure_no_fault(self) -> Result[None]:
        with self._state_lock:
            fault = self.fault
        if fault is None or fault == MotorFault.NONE:
            return Result.ok()
        return Result.err(f"Motor is in fault state: {fault.name}", code="fault")

    def _request_feedback(self, command: bytes, timeout: float) -> Result[ParsedMessage]:
        """发送命令并等待属于当前电机的任意反馈。"""
        return self.bus.request(
            command,
            matcher=self._matches_message,
            timeout=timeout,
        )

    def _matches_message(self, message: ParsedMessage) -> bool:
        if self.slave_id in message.route_ids:
            return True
        if self.master_id != 0 and self.master_id in message.route_ids:
            return True
        return message.slave_id == self.slave_id

    def _handle_message(self, message: ParsedMessage) -> None:
        if message.kind == "status":
            pos, vel, torque = DamiaoProtocol.decode_status(message.data, self.limits)
            with self._state_lock:
                self.pos = pos
                self.vel = vel
                self.torque = torque
                self.fault = message.fault
                self.mos_temp = message.mos_temp
                self.rotor_temp = message.rotor_temp
                if message.fault not in (None, MotorFault.NONE):
                    self.enabled = False
                self.last_update_time = time.time()
            return

        if message.kind == "param" and message.reg_id is not None:
            with self._state_lock:
                if message.value is not None:
                    self.param_cache[message.reg_id] = message.value
                    self.last_update_time = time.time()



class MotorManager:
    """管理共享同一条总线的多个电机对象。"""

    def __init__(self, bus: SerialBus) -> None:
        """初始化多电机管理器。

        Args:
            bus: 所有托管电机共用的串口总线。
        """
        self.bus = bus
        self._motors_by_slave_id: dict[MotorId, Motor] = {}
        self._motors_by_name: dict[str, Motor] = {}
    
    @property
    def motors(self) -> dict[MotorId, Motor]:
        """返回一份按 slave id 建立的电机映射副本。"""
        return dict(self._motors_by_slave_id)

    def add_motor(self, motor_type: MotorType, slave_id: MotorId, master_id: MotorId = 0, name: str | None = None) -> Motor:
        """创建、注册并返回一个新的电机对象。"""
        # 检查 slave_id 是否已被占用
        if slave_id in self._motors_by_slave_id:
            raise ValueError(f"Motor slave id 0x{slave_id:X} is already registered")
        
        # 检查名字是否已被占用
        if name is not None and name in self._motors_by_name:
            raise ValueError(f"Motor name '{name}' is already registered")
            
        motor = Motor(bus=self.bus, motor_type=motor_type, slave_id=slave_id, master_id=master_id, name=name)
        self.register(motor)
        return motor

    def register(self, motor: Motor) -> Motor:
        """把一个已经创建好的电机对象注册到管理器中。"""
        if motor.bus is not self.bus:
            raise ValueError("MotorManager can only register motors that use the same SerialBus")

        # 检查 slave_id 是否冲突
        existing = self._motors_by_slave_id.get(motor.slave_id)
        if existing is not None and existing is not motor:
            raise ValueError(f"Motor slave id 0x{motor.slave_id:X} is already registered")

        # 检查名字是否冲突
        if motor.name is not None:
            name_owner = self._motors_by_name.get(motor.name)
            if name_owner is not None and name_owner is not motor:
                raise ValueError(f"Motor name '{motor.name}' is already registered")

        self._motors_by_slave_id[motor.slave_id] = motor
        if motor.name is not None:
            self._motors_by_name[motor.name] = motor
        return motor

    def get(self, motor_id: MotorId) -> Motor | None:
        """通过 slave id 返回电机对象。"""
        return self._motors_by_slave_id.get(motor_id)

    def get_by_name(self, name: str) -> Motor | None:
        """通过名字返回电机对象。"""
        return self._motors_by_name.get(name)

    def __getitem__(self, key: MotorId | str) -> Motor:
        if isinstance(key, str):
            motor = self.get_by_name(key)
            if motor is None:
                raise KeyError(key)
            return motor
        else:
            motor = self.get(key)
            if motor is None:
                raise KeyError(key)
            return motor
    

    # ===========================================================================
    # 多电机状态/参数
    # ===========================================================================   
     
    def get_all_params(self) -> dict[MotorId, dict[RegId, float | int]]:
        """获取所有已注册电机的已缓存的寄存器参数。"""
        return {
            motor.slave_id: motor.param_cache
            for motor in self._motors_by_slave_id.values()
        }

    def get_all_status(self) -> dict[MotorId, MotorState]:
        """返回所有已注册电机缓存的当前状态。"""
        return {
            motor.slave_id: motor.get_state() 
            for motor in self._motors_by_slave_id.values()
        }

    def refresh_all_status(self) -> dict[MotorId, MotorState]:
        """刷新所有已注册电机的状态。"""
        for motor in self._motors_by_slave_id.values():
            motor.refresh_state()
        
        return self.get_all_status()


    # ===========================================================================
    # 基础控制
    # ===========================================================================

    def enable_all(self) -> dict[MotorId, Result[None]]:
        """使能所有已注册电机。"""
        return {
            motor.slave_id: motor.enable()
            for motor in self._motors_by_slave_id.values()
        }

    def disable_all(self) -> dict[MotorId, Result[None]]:
        """失能所有已注册电机。"""
        return {
            motor.slave_id: motor.disable()
            for motor in self._motors_by_slave_id.values()
        }

    def set_zero_all(self) -> dict[MotorId, Result[None]]:
        """将所有已注册电机的零点设置为当前位置。"""
        return {
            motor.slave_id: motor.set_zero()
            for motor in self._motors_by_slave_id.values()
        }

    def clear_error_all(self) -> dict[MotorId, Result[None]]:
        """清除所有已注册电机的错误。"""
        return {
            motor.slave_id: motor.clear_error()
            for motor in self._motors_by_slave_id.values()
        }
