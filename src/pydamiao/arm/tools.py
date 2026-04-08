import time


def wait_until(target_time: float):
    """
    等待直到 target_time 秒（基于 perf_counter）

    三段策略：
    - 远：sleep(0) 让出CPU
    - 近：busy-wait 保证精度
    """
    while True:
        now = time.perf_counter()
        dt = target_time - now

        if dt <= 0:
            return

        if dt > 0.002:
            time.sleep(0)   # 让出CPU
        else:
            pass            # busy-wait



class RateLoop:
    def __init__(self, hz, duration):
        self.period = 1.0 / hz
        self.start = time.perf_counter()
        self.end = self.start + duration
        self.next_time = self.start

    def __iter__(self):
        return self

    def __next__(self) -> float:
        """循环迭代, 返回已用时间(秒)"""

        # 等待到下一帧（关键改动）
        wait_until(self.next_time)

        now = time.perf_counter()

        # 判断是否结束
        if now >= self.end:
            print("Timeout, exiting")
            raise StopIteration

        # 推进时间轴（关键）
        self.next_time += self.period

        return now - self.start


class Timeout:
    def __init__(self, seconds):
        self.start = time.monotonic()
        self.end = self.start + seconds

    def __iter__(self):
        return self

    def __next__(self):
        now = time.monotonic()
        if now >= self.end:
            print("Timeout, exiting")
            raise StopIteration
        # time.sleep(0.01)
        return now - self.start  # 返回已用时间