# Description

Bringup package for the AR4 software stack on the real robot: the hardware interface
([`ar4_hardware_interface`](../ar4_hardware_interface/README.md)), its controllers, MoveIt and RViz.

The Teensy must run the
[`teensy_arm`](../ar4_hardware_interface/ar4_firmware/teensy_arm/README.md) firmware and be connected
directly to the computer. To build the package, run

```bash
colcon build --packages-up-to ar4_realbot_bringup
. install/setup.bash
```

To run the real robot, source and run:

```bash
ros2 launch ar4_realbot_bringup main.launch.py
```

#### Launch file arguments

- `calibrate`:
    - Calibrate the joints against their limit switches on startup. **The arm moves.** Use
      `calibrate:=False` if the Teensy has stayed powered since the last calibration. (default: `True`)
- `serial_port`:
    - Serial port of the Teensy. (default: `/dev/ttyACM0`)
- `ar_model`:
    - Model of the arm. (default: `ar4`)

See the [hardware interface README](../ar4_hardware_interface/README.md#controllers) for the
controllers it loads and how to switch to streaming joint positions.
