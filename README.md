

<p align="center">
  <img src="https://github.com/user-attachments/assets/04fbae2c-53e7-46ee-bb77-a79b0d9d8597" style="width: 40%; height: auto;">
</p>

<div align="center">
  <!-- Keep these links. Translations will automatically update with the README. -->
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=en">English</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=ja">日本語</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=de">Deutsch</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=es">Español</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=fr">français</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=ko">한국어</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=pt">Português</a> | 
  <a href="https://www.zdoc.app/DBinK/pydamiao?lang=ru">Русский</a>
</div>

# pydamiao

一个非官方的达妙电机 Python 库。

`pydamiao` 基于官方 SDK 的工作流做了更 Pythonic 的封装, 使其开发体验更好。`pydamiao` 提供串口通信、电机管理、参数读写和常见控制模式的高层接口，适合快速集成到自己的项目中。


## 特性

- 基于 `pyserial` 的简单串口总线封装
- 提供 `Motor` 和 `MotorManager` 高层接口
- 同时支持单电机和多电机场景
- 支持 `MIT`、`VEL`、`POS_VEL`、`POS_FORCE` 等常见控制模式
- 可能会失败的接口, 使用类似 Rust 的 `Result` 返回格式, 显式处理错误, 避免大量 `None` 判断

## 安装

从 [PyPI](https://pypi.org/project/pydamiao/) 安装
```bash
uv pip install pydamiao 
```
或
```bash
pip install pydamiao 
```

从源码安装：

```bash
git clone https://github.com/DBinK/pydamiao.git
cd pydamiao
pip install -e .
```

## 快速开始

```python
import math
import time

from pydamiao import Motor, MotorManager, ControlMode, MotorType, MotorReg, SerialBus

# 初始化所有对象
bus = SerialBus("COM3", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

motor1 = Motor(bus, MotorType.DM4310, 0x06, 0x12)
motor2 = Motor(bus, MotorType.DM4310, 0x05, 0x12)

# 注册电机到管理器
manager.register(motor1)
manager.register(motor2)

# 统一控制电机
manager.clean_error_all()
manager.disable_all()
manager.set_mode_all(ControlMode.MIT)

# 查看电机参数
for id, motor in manager.motors.items():
    print(f"电机 ID {id}:")
    print("CTRL_MODE:", motor.read_param(MotorReg.CTRL_MODE).value)
    print("MST_ID:", motor.read_param(MotorReg.MST_ID).value)

# 单独控制电机
if not motor1.set_mode(ControlMode.POS_VEL).is_ok:
    print("motor1 切换到 POS_VEL 失败")

if not motor2.set_mode(ControlMode.VEL).is_ok:
    print("motor2 切换到 VEL 失败")

# 运动测试
for _ in range(1000):
    q = math.sin(time.time())
    motor1.set_pos_vel(q * 8, 3)
    motor2.set_velocity(8 * q)
    time.sleep(0.001)

# 程序结束前, 记得失能所有电机 (虽然有自动失能兜底, 但是推荐养成好习惯)
manager.disable_all()
bus.close()
```

## API 概览

- `SerialBus`：共享串口通信总线
- `Motor`：单个电机的高层控制对象
- `MotorManager`：同一总线上的多电机管理器
- `ControlMode`、`MotorType`、`MotorReg`：协议相关枚举和辅助类型

## 使用文档 和 示例

文档正在补充中... 

可先查看示例 [`examples`](./examples) 目录, 包含了本库几乎全部用法:

- [`base.py`](./examples/base.py): 基础用法, 包含常用 API 的使用
- [`motor_single.py`](./examples/motor_single.py): 单电机控制
- [`motor_muitl.py`](./examples/motor_muitl.py): 多电机控制
- [`calibration.py`](./examples/calibration.py): 零点校准 (多电机)
- [`read_reg.py`](./examples/read_reg.py): 读取电机寄存器中的值

## 项目状态

项目仍在持续迭代中，正式稳定版本发布前 API 可能会继续调整。

## 许可证

MIT，详见 [`LICENSE`](./LICENSE)。
