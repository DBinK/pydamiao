# pydamiao

Unofficial Python library for Damiao motors.

`pydamiao` is a small, Python-friendly wrapper around the Damiao motor protocol and SDK workflow. It provides a clean API for serial communication, motor management, parameter access, and common control modes.

[简体中文](./README.zh-CN.md)

## Features

- Simple serial bus abstraction based on `pyserial`
- High-level `Motor` and `MotorManager` APIs
- Support for single-motor and multi-motor setups
- Built-in helpers for parameter read/write
- Common control modes such as `MIT`, `VEL`, and `POS_VEL`

## Installation

From source:

```bash
git clone https://github.com/DBinK/pydamiao.git
cd pydamiao
pip install -e .
```

## Quick Start

```python
import math
import time

from pydamiao import ControlMode, MotorManager, MotorType, SerialBus

bus = SerialBus("COM3", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

motor1 = manager.add_motor(MotorType.DM4310, 0x06, 0x12)
motor2 = manager.add_motor(MotorType.DM4310, 0x05, 0x12)

if not motor1.set_mode(ControlMode.POS_VEL).is_ok:
    raise RuntimeError("failed to switch motor1 to POS_VEL")
if not motor2.set_mode(ControlMode.VEL).is_ok:
    raise RuntimeError("failed to switch motor2 to VEL")

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

## API Overview

- `SerialBus`: shared serial communication bus
- `Motor`: high-level control for one motor
- `MotorManager`: coordinate multiple motors on the same bus
- `ControlMode`, `MotorType`, `MotorReg`: protocol enums and helpers

## Examples

See the [`examples`](./examples) directory for runnable scripts.

## Status

The project is under active development. APIs may still change before the first stable release.

## License

MIT. See [`LICENSE`](./LICENSE).
