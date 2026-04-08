
import math
from pathlib import Path
import time

from rich import print as rprint

from pydamiao.arm.config import JointID, joint_cfgs
from pydamiao.arm.joint import JointManager
from pydamiao.arm.recorder import JointsReader, JointsRecorder
from pydamiao.bus import SerialBus

from pydamiao.arm.tools import RateLoop
from pydamiao.structs import ControlMode


bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
manager = JointManager(bus)


manager.register(joint_cfgs)

manager.clean_error()
# manager.set_zero()

# manager.print_joints_cfg()
# breakpoint()

manager.enable()
# manager.set_mode(ControlMode.POS_FORCE)
manager.set_mode(ControlMode.POS_VEL)

# 控制示例

if __name__ == "__main__":


    npy_path = Path("tmp/test.npy")

    reader = JointsReader(npy_path)
    loop = RateLoop(200, duration=999)  # 时间给很大

    # mode = ControlMode.POS_FORCE
    mode = ControlMode.POS_VEL

    while True:

        for (pos, ts), _ in zip(reader, loop):
            print("send:", pos)
            manager.set_pos_list(pos, mode)

        for (pos, ts), _ in zip(reversed(reader), loop):
            print("send:", pos)
            manager.set_pos_list(pos, mode)

        print("done")