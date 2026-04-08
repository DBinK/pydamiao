
import math
from pathlib import Path
import time

from rich import print as rprint
from looptick import LoopTick

from pydamiao.arm.config import JointID, joint_cfgs
from pydamiao.arm.joint import JointManager
from pydamiao.arm.recorder import JointsRecorder
from pydamiao.bus import SerialBus

from pydamiao.arm.vis import rrlog_joints
from pydamiao.arm.tools import Timeout


bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
manager = JointManager(bus)


manager.register(joint_cfgs)

manager.clean_error()
# manager.set_zero()
manager.enable()

manager.set_teach_mode()

# 控制示例

if __name__ == "__main__":

    from pydamiao.arm.tools import RateLoop
    from looptick import LoopTick

    recorder = JointsRecorder()
    npy_path = Path("tmp/test.npy")

    loop = LoopTick()

    try:
        for elapsed in RateLoop(100, duration=15):

            # manager.update()
            pos_dict = manager.get_joints_pos()
            pos_list = manager.pos_dict_to_list(pos_dict)

            rprint(loop.tick_ms())
            
            # rrlog_joints(manager.pos_list_to_dict(pos_list))
            recorder.record(pos_list)
    
    finally:
        recorder.save(npy_path)