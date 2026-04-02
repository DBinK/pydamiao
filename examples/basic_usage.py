#!/usr/bin/env python3
"""
Basic usage example for PyDamiao library.

Demonstrates:
- Creating a motor manager with context manager
- Adding motors
- Control commands with auto mode switching
- Reading cached state
- Parameter read/write
"""

from pydamiao import SerialBus, MotorManager, MotorType, ControlMode, MotorReg


def main():
    # Use context manager for automatic cleanup
    with MotorManager(SerialBus("/dev/ttyUSB0", baudrate=921600)) as manager:
        # Add motors
        motor1 = manager.add_motor(MotorType.DM4310, slave_id=0x06)
        motor2 = manager.add_motor(MotorType.DM4310, slave_id=0x07)
        
        # Enable all motors
        results = manager.enable_all()
        for motor_id, success in results.items():
            if success:
                print(f"Motor {motor_id} enabled")
            else:
                print(f"Failed to enable motor {motor_id}")
        
        # Control commands automatically switch mode
        # No need to call set_mode() manually!
        motor1.set_pos_vel(pos=1.0, vel=0.5)  # Auto-switches to POS_VEL mode
        motor2.set_velocity(vel=2.0)  # Auto-switches to VEL mode
        
        # Read cached state (updated asynchronously by background thread)
        print(f"Motor 1 position: {motor1.position:.3f} rad")
        print(f"Motor 1 velocity: {motor1.velocity:.3f} rad/s")
        print(f"Motor 1 torque: {motor1.torque:.3f} N·m")
        
        # Request state refresh (response updates cache asynchronously)
        motor1.refresh_state()
        
        # Synchronous parameter access (blocks until response)
        pmax = motor1.read_param(MotorReg.PMAX)
        if pmax is not None:
            print(f"Motor 1 PMAX: {pmax}")
        else:
            print("Failed to read PMAX")
        
        # Write parameter
        if motor1.write_param(MotorReg.MAX_SPD, 10.0):
            print("Successfully wrote MAX_SPD")
        
        # MIT control mode (auto-switches to MIT mode)
        motor1.set_mit(kp=100, kd=2.0, pos=0.5, vel=0.1, torque=0.05)
        
        # Batch operations
        manager.refresh_all_states()
        
        # Access motors by ID
        manager[0x06].set_zero()
        
    # Context manager automatically disables all motors and closes serial


if __name__ == "__main__":
    main()
