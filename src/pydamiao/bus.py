# src/pydamiao/bus.py

import serial
import threading
from typing import Callable, Optional, Dict
from collections import deque
import time

from pydamiao.protocol import DamiaoProtocol


class SerialBus:
    """
    Simple thread-safe serial communication layer.
    
    Features:
    - Thread-safe sending with lock
    - Background receiver thread
    - Automatic frame extraction (handles partial/sticky packets)
    - Callback-based message dispatch
    - Optional synchronous request-response
    
    Usage:
        bus = SerialBus("/dev/ttyUSB0", baudrate=921600)
        
        # Register callback for async messages
        def on_message(msg):
            print(f"Received: {msg}")
        bus.register_callback(on_message)
        
        # Send commands
        bus.send(frame_bytes)
        
        # Or send and wait for response
        response = bus.request_sync(frame_bytes, timeout=0.5)
        
        bus.close()
    """
    
    FRAME_HEADER = 0xAA
    FRAME_TAIL = 0x55
    FRAME_LENGTH = 16
    
    def __init__(
        self, 
        port: str, 
        baudrate: int = 921600, 
        timeout: float = 0.01,
        existing_serial: Optional[serial.Serial] = None
    ):
        """
        Initialize the serial bus.
        
        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0", "COM3")
            baudrate: Baud rate, default 921600
            timeout: Read timeout in seconds
            existing_serial: Optional existing serial object to use
        """
        if existing_serial is not None:
            self.ser = existing_serial
            if not self.ser.is_open:
                self.ser.open()
        else:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            if not self.ser.is_open:
                self.ser.open()
        
        self.lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._running = True
        
        # Callbacks for received messages
        self._callbacks: list[Callable[[dict], None]] = []
        
        # Synchronous request tracking
        self._pending: Dict[str, deque] = {}
        self._pending_lock = threading.Lock()
        
        # Start background receiver thread
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()
    
    def send(self, data: bytes) -> bool:
        """
        Send data through the serial bus (thread-safe).
        
        Args:
            data: Bytes to send
            
        Returns:
            True if successful, False otherwise
        """
        try:
            with self.lock:
                self.ser.write(data)
            return True
        except Exception as e:
            print(f"[SerialBus] Send error: {e}")
            return False
    
    def read(self, size: int = 64) -> bytes:
        """Read data from serial (called by receiver thread)."""
        try:
            return self.ser.read(size)
        except:
            return b""
    
    def close(self):
        """Close the serial bus and stop receiver thread."""
        self._running = False
        if self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def register_callback(self, callback: Callable[[dict], None]):
        """Register a callback for received messages."""
        self._callbacks.append(callback)
    
    def unregister_callback(self, callback: Callable[[dict], None]):
        """Unregister a message callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)
    
    def _receive_loop(self):
        """Background thread for receiving and parsing frames."""
        while self._running:
            try:
                data = self.read(64)
                if data:
                    self._rx_buffer.extend(data)
                    frames = self._extract_frames()
                    
                    for frame in frames:
                        parsed = DamiaoProtocol.decode_frame(frame)
                        if parsed:
                            self._dispatch(parsed)
            except Exception as e:
                print(f"[SerialBus] Receiver error: {e}")
                time.sleep(0.01)
    
    def _extract_frames(self) -> list[bytes]:
        """Extract complete frames from buffer."""
        frames = []
        i = 0
        
        while i <= len(self._rx_buffer) - self.FRAME_LENGTH:
            if (self._rx_buffer[i] == self.FRAME_HEADER and 
                self._rx_buffer[i + self.FRAME_LENGTH - 1] == self.FRAME_TAIL):
                frame = bytes(self._rx_buffer[i:i + self.FRAME_LENGTH])
                frames.append(frame)
                i += self.FRAME_LENGTH
            else:
                i += 1
        
        # Keep remaining data in buffer
        self._rx_buffer = self._rx_buffer[i:]
        return frames
    
    def _dispatch(self, msg: dict):
        """Dispatch parsed message to callbacks and pending requests."""
        # Notify callbacks
        for callback in self._callbacks:
            try:
                callback(msg)
            except Exception as e:
                print(f"[SerialBus] Callback error: {e}")
        
        # Check for pending synchronous requests
        msg_type = msg.get('type')
        motor_id = msg.get('motor_id')
        reg_id = msg.get('reg_id')
        
        # Build lookup keys
        keys_to_try = []
        if msg_type is not None:
            keys_to_try.append(f"{msg_type}:{motor_id}:{reg_id}")
            if motor_id is not None:
                keys_to_try.append(f"{msg_type}:{motor_id}:None")
            keys_to_try.append(f"{msg_type}:None:None")
        
        with self._pending_lock:
            for key in keys_to_try:
                if key in self._pending:
                    queue = self._pending[key]
                    queue.append(msg)
                    break
    
    def request_sync(
        self, 
        data: bytes, 
        msg_type: Optional[int] = None,
        motor_id: Optional[int] = None,
        reg_id: Optional[int] = None,
        timeout: float = 0.5
    ) -> Optional[dict]:
        """
        Send a request and wait for response synchronously.
        
        Args:
            data: Data to send
            msg_type: Expected message type for response matching
            motor_id: Expected motor ID for response matching
            reg_id: Expected register ID for response matching
            timeout: Timeout in seconds
            
        Returns:
            Response dict or None if failed/timeout
        """
        key = f"{msg_type}:{motor_id}:{reg_id}"
        queue = deque()
        
        with self._pending_lock:
            self._pending[key] = queue
        
        try:
            # Send the request
            if not self.send(data):
                return None
            
            # Wait for response
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self._pending_lock:
                    if queue:
                        return queue.popleft()
                time.sleep(0.001)
            
            return None  # Timeout
        finally:
            with self._pending_lock:
                if key in self._pending:
                    del self._pending[key]
