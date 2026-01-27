[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=flat&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/DBinK/pydamiao)

# 达妙电机 Python 库
---
一个用于达妙电机的非官方 Python 库  
基于官方 SDK 源码二次开发，通过更“Pythonic”的方式进行封装，支持 uv 现代工具链，计划发布至 PyPI 以便一键安装与集成。

项目仍在快速迭代中，部分功能与文档可能尚不完善。可查看 **[zread.ai](https://zread.ai/DBinK/pydamiao)** 自动为本项目生成的说明和使用文档

## 快速开始

从 PyPi 安装（暂不支持）:

```bash
pip install pydamiao
```

或者

从源码中安装:

```bash
git clone https://github.com/DBinK/pydamiao.git  # 克隆项目
pip install -e ./                                # 以可编辑模式安装到项目
```


下面是一个完整的示例，帮助你开始控制两个达妙电机

```python
import math
import time
import serial
from pydamiao.enums import DamiaoMotorType, ControlType, MotorVariable
from pydamiao.motor import Motor, MotorControl
 
# 1. 创建电机对象
motor1 = Motor(DamiaoMotorType.DM4310, 0x01, 0x11)
motor2 = Motor(DamiaoMotorType.DM4310, 0x05, 0x15)
 
# 2. 设置串口通信
serial_device = serial.Serial("COM8", 921600, timeout=0.5)
motor_control = MotorControl(serial_device)
 
# 3. 将电机添加到控制器
motor_control.add_motor(motor1)
motor_control.add_motor(motor2)
 
# 4. 配置控制模式
motor_control.switch_control_mode(motor1, ControlType.POS_VEL)
motor_control.switch_control_mode(motor2, ControlType.VEL)
 
# 5. 保存参数并启用电机
motor_control.save_motor_param(motor1)
motor_control.save_motor_param(motor2)
motor_control.enable(motor1)
motor_control.enable(motor2)
 
# 6. 控制电机
for i in range(10000):
    q = math.sin(time.time())
    motor_control.control_pos_vel(motor1, q * 8, 30)
    motor_control.control_vel(motor2, 8 * q)
    time.sleep(0.001)
 
# 7. 清理
serial_device.close()
```

