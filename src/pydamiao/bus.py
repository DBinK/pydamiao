
from queue import Empty, Queue
import threading
from typing import TYPE_CHECKING, Callable, Protocol

from serial import Serial, SerialException

from pydamiao.exceptions import ProtocolError, ReceiverThreadError, SerialBusError, SerialPortClosedError
from pydamiao.protocol import DamiaoProtocol, ParsedMessage
from pydamiao.result import Result

if TYPE_CHECKING:
    from pydamiao.motor import Motor


class SerialPortLike(Protocol):
    """`SerialBus` 依赖的最小串口接口。"""

    @property
    def is_open(self) -> bool: ...

    def open(self) -> None: ...
    def close(self) -> None: ...
    def write(self, data: bytes, /) -> int | None: ...
    def read(self, size: int = 1) -> bytes: ...


class SerialBus:
    """管理共享串口连接和后台接收循环。

    总线层负责线程安全发送、后台拆帧，以及为 ``Motor`` 这类高层 API
    提供 request-response 匹配能力。
    """

    def __init__(
        self,
        port: str | SerialPortLike,
        baudrate: int = 921600,
        timeout: float = 0.01,
        read_size: int = 64,
        auto_start: bool = True,
    ) -> None:
        """初始化共享串口总线。

        Args:
            port: 串口名，或已经打开的类串口对象。
            baudrate: 通过串口名打开时使用的波特率。
            timeout: 底层串口对象使用的读超时。
            read_size: 接收线程每次读取的最大字节数。
            auto_start: 是否在初始化后立即启动后台接收线程。
        """
        self.serial = self._open_serial(port, baudrate=baudrate, timeout=timeout)
        self.baudrate = baudrate
        self.timeout = timeout
        self.read_size = read_size

        self.protocol = DamiaoProtocol()
        self._send_lock = threading.Lock()
        self._waiters_lock = threading.Lock()
        self._motors_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._waiters: list[tuple[Callable[[ParsedMessage], bool], Queue[ParsedMessage | None]]] = []
        self._motors_by_alias: dict[int, Motor] = {}
        self._rx_buffer = b""
        self._receiver_error: BaseException | None = None
        self._receiver_thread: threading.Thread | None = None

        if auto_start:
            self.start()

    def _open_serial(self, port: str | SerialPortLike, baudrate: int, timeout: float) -> SerialPortLike:
        if not isinstance(port, str):
            serial_device = port
            setattr(serial_device, "timeout", timeout)
            if not serial_device.is_open:
                serial_device.open()
            return serial_device

        try:
            return Serial(port=port, baudrate=baudrate, timeout=timeout)
        except SerialException as exc:
            raise SerialBusError(f"Failed to open serial port {port!r}: {exc}") from exc

    def start(self) -> None:
        """启动后台接收线程；如果已经启动则直接返回。"""
        self._check_receiver()
        if self._receiver_thread and self._receiver_thread.is_alive():
            return

        self._stop_event.clear()
        self._receiver_thread = threading.Thread(
            target=self._receiver_loop,
            name="pydamiao-serial-receiver",
            daemon=True,
        )
        self._receiver_thread.start()

    def close(self) -> None:
        """停止接收线程并关闭串口。"""
        self._stop_event.set()
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=max(self.timeout * 5, 0.1))
        if self.serial.is_open:
            self.serial.close()

    def register_motor(self, motor: "Motor") -> None:
        """注册电机对象，使接收到的帧可以按 id 路由。"""
        with self._motors_lock:
            for alias_id in motor.alias_ids:
                owner = self._motors_by_alias.get(alias_id)
                if owner is not None and owner is not motor:
                    raise SerialBusError(f"Motor id 0x{alias_id:X} is already registered")
                self._motors_by_alias[alias_id] = motor

    def get_motor(self, motor_id: int) -> "Motor | None":
        """按 slave id 或 master id 返回已注册电机。"""
        with self._motors_lock:
            return self._motors_by_alias.get(motor_id)

    def send(self, data: bytes) -> None:
        """在共享串口链路上发送原始字节。

        Args:
            data: 已经编码完成的桥接帧。

        Raises:
            SerialPortClosedError: 串口未打开时抛出。
            SerialBusError: 底层写入失败时抛出。
            ReceiverThreadError: 接收线程已经失败时抛出。
        """
        self._check_receiver()
        if not self.serial.is_open:
            raise SerialPortClosedError("Serial port is closed")
        with self._send_lock:
            try:
                self.serial.write(data)
            except SerialException as exc:
                raise SerialBusError(f"Failed to write to serial port: {exc}") from exc

    def request(
        self,
        data: bytes,
        matcher: Callable[[ParsedMessage], bool],
        timeout: float = 0.5,
    ) -> Result[ParsedMessage]:
        """发送请求并等待第一条匹配的响应。

        Args:
            data: 已编码完成的请求帧。
            matcher: 用于匹配响应消息的判定函数。
            timeout: 等待匹配响应的最大秒数。

        Returns:
            Result[ParsedMessage]: 成功时返回匹配到的响应，否则返回超时或中断错误。
        """
        response_queue: Queue[ParsedMessage | None] = Queue(maxsize=1)
        with self._waiters_lock:
            self._waiters.append((matcher, response_queue))

        try:
            self.send(data)
            try:
                response = response_queue.get(timeout=timeout)
            except Empty:
                self._check_receiver()
                return Result.err("Timed out waiting for motor response", code="timeout")

            self._check_receiver()
            if response is None:
                return Result.err("Receiver stopped before a matching response arrived", code="interrupted")
            return Result.ok(response)
        finally:
            with self._waiters_lock:
                waiter = (matcher, response_queue)
                if waiter in self._waiters:
                    self._waiters.remove(waiter)


    def _receiver_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = self.serial.read(self.read_size)
                    if not chunk:
                        continue

                    self._rx_buffer += chunk
                    frames, self._rx_buffer = self.protocol.extract_frames(self._rx_buffer)
                    for frame in frames:
                        message = self.protocol.parse_frame(frame)
                        self._dispatch_message(message)
                except ValueError as exc:  # 单帧解析错误不应终止整个接收线程
                    print(exc)  # TODO: 改为用日志记录
                    continue
        except SerialException as exc:
            self._receiver_error = SerialBusError(f"Serial communication failed: {exc}")
            self._notify_waiters_of_failure()
        except Exception as exc:
            self._receiver_error = exc
            self._notify_waiters_of_failure()

    def _dispatch_message(self, message: ParsedMessage) -> None:
        motor = None
        for route_id in message.route_ids:
            motor = self.get_motor(route_id)
            if motor is not None:
                break

        if motor is not None:
            motor._handle_message(message)

        with self._waiters_lock:
            for matcher, response_queue in list(self._waiters):
                if matcher(message) and response_queue.empty():
                    response_queue.put_nowait(message)

    def _notify_waiters_of_failure(self) -> None:
        # 唤醒所有等待中的请求，让它们把接收线程故障显式返回给上层。
        with self._waiters_lock:
            for _, response_queue in self._waiters:
                if response_queue.empty():
                    response_queue.put_nowait(None)

    def _check_receiver(self) -> None:
        if self._receiver_error is not None:
            raise ReceiverThreadError("Serial receiver thread has stopped unexpectedly") from self._receiver_error
