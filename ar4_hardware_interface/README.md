# ar4_hardware_interface

This module contains the hardware interface for the AR4 robot.


## Initial Set-up

1. Connect the teensy usb to the computer
2. Run:
```bash
./docker/run.sh -s ar4_hardware
```

## Run the hardware interface

```
ros2 launch ar4_hardware_interface main.launch.py calibrate:=True
```

The Teensy must run the [`teensy_arm`](ar4_firmware/teensy_arm/README.md) firmware. To bring up the
whole real-robot stack (hardware interface, MoveIt and RViz), use
[`ar4_realbot_bringup`](../ar4_realbot_bringup/README.md) instead.

## Launch moveit
```
ros2 launch ar4_moveit_config demo.launch.py
```

## AR4 Gripper

The servo gripper is driven by an Arduino Nano (`ar4_firmware/firmware_gripper_arduino.ino`) on
`arduino_serial_port` (default `/dev/ttyUSB0`). `config/gripper_driver.yaml` maps the finger travel
(`finger_joint_1`, -0.014 m open to 0 closed) onto the servo angles. The `gripper_controller`
(`GripperActionController`) takes goals on `/gripper_controller/gripper_cmd`.

`include_gripper:=False` skips the gripper hardware and its controller, for an arm without the Nano.
It affects only the hardware: the robot description and the MoveIt config still include the gripper,
so its joints have no state and MoveIt can't plan with them.

## Controllers

The hardware interface exposes position command interfaces for `joint_1` to `joint_6`. The launch
file loads two controllers on them (configured in `ar4_moveit_config/config/ros2_controllers.yaml`):

- `arm_controller` (`joint_trajectory_controller/JointTrajectoryController`), **active** by
  default. MoveIt executes its plans through it.
- `forward_position_controller` (`forward_command_controller/ForwardCommandController`), loaded
  **inactive** because it claims the same joints. It takes one target per message on
  `/forward_position_controller/commands` (`std_msgs/Float64MultiArray`, radians, joints 1 to 6),
  for streaming positions from teleoperation or a learned policy.

Switch between them with:

```
ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
ros2 control switch_controllers --deactivate forward_position_controller --activate arm_controller
```

The firmware limits the joint speed and acceleration, so streamed targets are followed smoothly
but with some lag.

## Joint Encoder Calibration

You can skip the automatic encoder calibration sequence with the launch
argument `calibrate:=false`. Since the original encoders (AMT-102V) are
relative encoders, they need to be calibrated against the limit switches when
powered on. If you are restarting the software but not the Teensy, the encoders
will have remained powered on and do not need to be recalibrated. Alternatively,
you can change the REST_ENC_POSITIONS values in the Teensy sketch so that the
encoders are initialised to those values on startup, provided that your arm is
also always initialised in that position as well. Before starting any movements,
it is always good to check in RViz that the model is in a sensible position to
verify that the encoders are properly calibrated.
