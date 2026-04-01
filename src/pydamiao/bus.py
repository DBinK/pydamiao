# src/pydamiao/bus.py
"""
串口总线层 - 负责底层串口通信
线程安全、支持队列、独立接收线程
"""

import serial
import threading
from typing import Optional, Callable
from collections import deque


class SerialBus:
    """
    串口总线类，负责底层串口收发
    - 线程安全的发送
    - 独立接收线程
    - 数据缓冲队列
    """
    
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.01):
        """
        初始化串口总线
        
        Args:
            port: 串口号 (如 "COM3" 或 "/dev/ttyUSB0")
            baudrate: 波特率，默认 921600
            timeout: 串口读取超时时间，默认 0.01s (实时性要求)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # 串口对象
        self.ser: Optional[serial.Serial] = None
        
        # 发送锁 - 保证多电机并发时串口写入线程安全
        self._send_lock = threading.Lock()
        
        # 接收缓冲区队列
        self._rx_queue = deque()
        self._rx_lock = threading.Lock()
        
        # 接收线程
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False
        
        # 数据回调函数
        self._on_data_callback: Optional[Callable[[bytes], None]] = None
        
    def open(self):
        """打开串口并启动接收线程"""
        if self.ser is None or not self.ser.is_open:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"SerialBus: Opened {self.port} at {self.baudrate}")
            
        # 启动接收线程
        if not self._rx_running:
            self._rx_running = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            
    def close(self):
        """关闭串口并停止接收线程"""
        self._rx_running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"SerialBus: Closed {self.port}")
            
    def set_data_callback(self, callback: Callable[[bytes], None]):
        """
        设置数据接收回调函数
        
        Args:
            callback: 回调函数，接收 bytes 数据
        """
        self._on_data_callback = callback
        
    def send(self, data: bytes) -> bool:
        """
        发送数据（线程安全）
        
        Args:
            data: 要发送的字节数据
            
        Returns:
            True 表示发送成功，False 表示失败
        """
        if not self.ser or not self.ser.is_open:
            return False
            
        with self._send_lock:
            try:
                self.ser.write(data)
                self.ser.flush()
                return True
            except Exception as e:
                print(f"SerialBus send error: {e}")
                return False
                
    def read(self, size: int = 1) -> bytes:
        """
        同步读取数据（不推荐，优先使用回调）
        
        Args:
            size: 读取字节数
            
        Returns:
            读取到的数据
        """
        if not self.ser or not self.ser.is_open:
            return b""
        return self.ser.read(size)
        
    def read_all(self) -> bytes:
        """
        读取所有可用数据（不推荐，优先使用回调）
        
        Returns:
            读取到的数据
        """
        if not self.ser or not self.ser.is_open:
            return b""
        return self.ser.read_all()
        
    def _rx_loop(self):
        """接收线程主循环"""
        while self._rx_running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        # 放入队列
                        with self._rx_lock:
                            self._rx_queue.append(data)
                        
                        # 触发回调
                        if self._on_data_callback:
                            try:
                                self._on_data_callback(data)
                            except Exception as e:
                                print(f"SerialBus callback error: {e}")
                                
            except Exception as e:
                print(f"SerialBus rx_loop error: {e}")
                
            # 避免空转占用 CPU
            threading.Event().wait(0.001)
            
    def get_queued_data(self) -> bytes:
        """
        从队列获取所有累积的数据
        
        Returns:
            所有累积的数据
        """
        with self._rx_lock:
            if not self._rx_queue:
                return b""
            data = b"".join(self._rx_queue)
            self._rx_queue.clear()
            return data
            
    @property
    def is_open(self) -> bool:
        """检查串口是否打开"""
        return self.ser is not None and self.ser.is_open
        
    @property
    def in_waiting(self) -> int:
        """获取接收缓冲区等待读取的字节数"""
        if not self.ser or not self.ser.is_open:
            return 0
        return self.ser.in_waiting
