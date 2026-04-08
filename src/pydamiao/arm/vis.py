import numpy as np
from pydamiao.structs import MotorId
import rerun as rr
import time

# 初始化 Rerun 会话，指定应用名称
rr.init("pyarmx")
# rr.save("tmp/data.rrd")  # 保存到文件

# 启动本地可视化查看器
rr.spawn()

def rrlog_joints(joints: dict[MotorId, float]):
    rr.set_time("time", timestamp=time.time())
    for joint_id, pos in joints.items():
        rr.log(f"j_{joint_id}", rr.Scalars(float(pos)))

def npylog_joints(joints_list: list[float]):
    timestamp = time.time()
    
    
        
if __name__ == "__main__":
    for i in range(100):
        rrlog_joints({
            1: np.random.uniform(-3.14, 3.14),
            6: np.random.uniform(-3.14, 3.14),
            4: np.random.uniform(-3.14, 3.14),
            9: np.random.uniform(-3.14, 3.14),
        })
        time.sleep(0.1)