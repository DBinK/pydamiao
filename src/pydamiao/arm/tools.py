import time

def wait_until(target_time: float):
    """高精度等待直到目标时间"""
    while True:
        dt = target_time - time.perf_counter()
        if dt <= 0:
            return
        if dt > 0.002:
            time.sleep(0)  # 让出 CPU
        # dt < 2ms 时进入 busy-wait


class Timer:
    def __init__(self, duration: float, auto_start=True):
        self.duration = duration
        self._start_time: float | None = None
        self._end_time: float | None = None
        if auto_start:
            self.reset()

    def reset(self):
        self._start_time = time.perf_counter()
        self._end_time = self._start_time + self.duration
        return self

    @property
    def done(self) -> bool:
        if self._start_time is None or self._end_time is None:
            self.reset()
        # 修复点 1：此时 self._end_time 已通过 reset 确保不是 None
        assert self._end_time is not None
        return time.perf_counter() >= self._end_time

    def elapsed(self) -> float | None:
        if self.done:
            return None
        # 修复点 2：done 属性已保证了 _start_time 存在
        assert self._start_time is not None
        return time.perf_counter() - self._start_time

    def __iter__(self):
        self.reset()
        return self

    def __next__(self):
        val = self.elapsed()
        if val is None:
            raise StopIteration
        return val

class Rate:
    def __init__(self, hz: float, duration: float | None = None):
        self.period = 1.0 / hz
        self.duration = duration
        self._start_time: float | None = None
        self._next_time: float | None = None
        self._end_time: float | None = None

    def reset(self):
        now = time.perf_counter()
        self._start_time = now
        self._next_time = now
        self._end_time = (now + self.duration) if self.duration is not None else None
        return self

    def sleep(self) -> float | None:
        if self._start_time is None or self._next_time is None:
            self.reset()
        
        # 修复点 3：显式断言或重新获取局部变量以通过类型检查
        start_time = self._start_time
        next_time = self._next_time
        assert start_time is not None and next_time is not None

        wait_until(next_time)
        
        now = time.perf_counter()
        if self._end_time is not None and now >= self._end_time:
            return None

        elapsed = now - start_time
        # 修复点 4：更新下一帧时间
        self._next_time = next_time + self.period
        return elapsed

    def __iter__(self):
        self.reset()
        return self

    def __next__(self):
        val = self.sleep()
        if val is None:
            raise StopIteration
        return val

if __name__ == "__main__":
    
    # --- 测试 Timer (for 循环) ---
    print("--- 测试 Timer (for 循环) ---")

    # 迭代器会自动调用 reset()，从 0 开始
    for t in Timer(1.0):
        print(f"for: {t:.2f}")
        time.sleep(0.2)


    # --- 测试 Timer (while 循环) ---
    print("\n--- 测试 Timer (while 循环) ---")

    timer = Timer(1.0)
    while not timer.done:
        t = timer.elapsed()
        if t is not None:
            print(f"while: {t:.2f}")
            time.sleep(0.2)


    # --- 测试 Rate (for 循环) ---
    print("\n--- 测试 Rate (for 循环, 10Hz, 1s) ---")

    for t in Rate(10, duration=1.0):
        print(f"for: {t:.2f}")


    # --- 测试 Rate (while 循环) ---
    print("\n--- 测试 Rate (while 循环, 5Hz, 1s) ---")

    rate = Rate(5, duration=1.0)
    while True:
        t = rate.sleep()
        if t is None:
            break
        print(f"while: {t:.2f}")
        # 模拟工作耗时（只要小于 period 0.2s，频率就是稳定的）
        time.sleep(0.05)


    # --- 测试 Rate (无限循环) ---
    print("\n--- 测试 Rate (无限循环, 演示前 3 次) ---")

    inf_rate = Rate(10) # 不传 duration
    count = 0
    while True:
        t = inf_rate.sleep()
        print(f"infinite: {t:.2f}")
        count += 1
        if count >= 30: 
            print("Stopping infinite test manually.")
            break