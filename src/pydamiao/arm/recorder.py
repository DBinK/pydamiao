from pathlib import Path
import time
import numpy as np

class JointsRecorder:
    def __init__(self) -> None:
        self.data = []
    
    def record(self, pos_list: list[float]):
        timestamp  = time.time()
        frame = pos_list + [timestamp]
        self.data.append(frame)
    
    def save(self, npy_path: Path | str):
        """保存数据。"""
        np.save(npy_path, self.data)


class JointsReader:
    def __init__(self, npy_path: Path | str):
        """
        npy_path: numpy 文件路径
        """
        self.npy_path = Path(npy_path)
        self.data = np.load(self.npy_path)

    def get_all(self):
        """返回全部数据"""
        return self.data

    def get_positions(self):
        """返回关节角（去掉时间戳）"""
        return self.data[:, :-1]

    def get_timestamps(self):
        """返回时间戳"""
        return self.data[:, -1]

    def __len__(self):
        return len(self.data)
    
    def __iter__(self):
        for row in self.data:
            yield row[:-1], row[-1]


if __name__ == "__main__":

    from pydamiao.arm.tools import Timeout

    recorder = JointsRecorder()
    npy_path = Path("tmp/test.npy")

    try:
        for elapsed in Timeout(5):
            pos_list = np.random.uniform(-3.14, 3.14, size=6)
            recorder.record(pos_list.tolist())
            time.sleep(0.1)
    
    finally:
        recorder.save(npy_path)