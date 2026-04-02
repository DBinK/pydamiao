# pydamiao

一个非官方的达妙电机 Python 库。

`pydamiao` 基于官方 SDK 的工作流做了更 Python 风格的封装，提供串口通信、电机管理、参数读写和常见控制模式的高层接口，适合快速集成到自己的项目中。

[English](./README.md)

## 特性

- 基于 `pyserial` 的简单串口总线封装
- 提供 `Motor` 和 `MotorManager` 高层接口
- 同时支持单电机和多电机场景
- 内置参数读写辅助方法
- 支持 `MIT`、`VEL`、`POS_VEL` 等常见控制模式

## 安装

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

from pydamiao import ControlMode, MotorManager, MotorType, SerialBus

bus = SerialBus("COM3", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

motor1 = manager.add_motor(MotorType.DM4310, 0x06, 0x12)
motor2 = manager.add_motor(MotorType.DM4310, 0x05, 0x12)

if not motor1.set_mode(ControlMode.POS_VEL).is_ok:
    raise RuntimeError("motor1 切换到 POS_VEL 失败")
if not motor2.set_mode(ControlMode.VEL).is_ok:
    raise RuntimeError("motor2 切换到 VEL 失败")

motor1.save_params().unwrap()
motor2.save_params().unwrap()
for result in manager.enable_all().values():
    result.unwrap()

for _ in range(1000):
    q = math.sin(time.time())
    motor1.set_pos_vel(q * 8, 3)
    motor2.set_velocity(8 * q)
    time.sleep(0.001)

bus.close()
```

## API 概览

- `SerialBus`：共享串口通信总线
- `Motor`：单个电机的高层控制对象
- `MotorManager`：同一总线上的多电机管理器
- `ControlMode`、`MotorType`、`MotorReg`：协议相关枚举和辅助类型

## 示例

可运行示例见 [`examples`](./examples) 目录。

## 项目状态

项目仍在持续迭代中，正式稳定版本发布前 API 可能会继续调整。

## 许可证

MIT，详见 [`LICENSE`](./LICENSE)。
