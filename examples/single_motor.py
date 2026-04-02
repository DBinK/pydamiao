"""
Single motor usage example with the new architecture.

This example demonstrates:
- Creating a SerialBus for communication
- Using MotorManager to register motors
- Reading/writing parameters
- MIT control mode
- State feedback via async callbacks
"""

import math
import time

from pydamiao import (
    SerialBus,
    MotorManager,
    MotorType,
    ControlMode,
    MotorReg,
)


def main():
    # Create serial bus (automatically starts receiver thread)
    # Replace "/dev/ttyUSB0" with your actual port (e.g., "COM9" on Windows)
    bus = SerialBus(port="/dev/ttyUSB0", baudrate=921600, timeout=0.01)
    
    # Create motor manager
    manager = MotorManager(bus)
    
    # Add a single motor
    motor1 = manager.add_motor(MotorType.DM4310, slave_id=0x06, master_id=0x16)
    
    print(f"Motor registered: {motor1}")
    
    # Switch to position-velocity mode
    result = motor1.write_param(MotorReg.CTRL_MODE, ControlMode.POS_VEL)
    if result.ok:
        print("Switch to POS_VEL mode success")
    else:
        print(f"Switch mode failed: {result.error}")
    
    # Read some parameters
    sub_ver = motor1.read_param(MotorReg.sub_ver)
    print(f"sub_ver: {sub_ver.value if sub_ver.ok else sub_ver.error}")
    
    Gr = motor1.read_param(MotorReg.Gr)
    print(f"Gr: {Gr.value if Gr.ok else Gr.error}")
    
    PMAX = motor1.read_param(MotorReg.PMAX)
    print(f"PMAX: {PMAX.value if PMAX.ok else PMAX.error}")
    
    MST_ID = motor1.read_param(MotorReg.MST_ID)
    print(f"MST_ID: {MST_ID.value if MST_ID.ok else MST_ID.error}")
    
    VMAX = motor1.read_param(MotorReg.VMAX)
    print(f"VMAX: {VMAX.value if VMAX.ok else VMAX.error}")
    
    TMAX = motor1.read_param(MotorReg.TMAX)
    print(f"TMAX: {TMAX.value if TMAX.ok else TMAX.error}")
    
    # Save parameters to flash
    motor1.save_params()
    
    # Enable motor
    motor1.enable()
    print("Motor enabled")
    
    # Brief test: enable then disable
    time.sleep(0.1)
    motor1.disable()
    print("Motor disabled")
    
    # Control loop demonstration
    print("\nStarting control loop...")
    i = 0
    try:
        while i < 1000:
            q = math.sin(time.time())
            i += 1
            
            # Request state update (asynchronous - will be updated via callback)
            motor1.refresh_state()
            
            # Print current state (from cache, updated by background receiver)
            print(
                f"Motor1: POS={motor1.position:.3f}, "
                f"VEL={motor1.velocity:.3f}, "
                f"TORQUE={motor1.torque:.3f}"
            )
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Cleanup
        motor1.disable()
        bus.close()
        print("Shutdown complete")


if __name__ == "__main__":
    main()
