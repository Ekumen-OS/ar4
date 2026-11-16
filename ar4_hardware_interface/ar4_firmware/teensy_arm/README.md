# AR4 Teensy 4.1 arm firmware

Firmware for the Teensy 4.1 that drives the AR4 (MK3) steppers, reads the joint encoders and limit switches, and talks
to `ar4_hardware_interface` over USB serial. It is based on the
[ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver) firmware (protocol version `0.0.1`).

## Flashing

Requires the [Teensy board package](https://www.pjrc.com/teensy/td_download.html) and the `AccelStepper` library
(`Encoder` ships with the Teensy package). With `arduino-cli`:

```bash
arduino-cli core install teensy:avr --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli lib install AccelStepper
arduino-cli compile --fqbn teensy:avr:teensy41 ar4_hardware_interface/ar4_firmware/teensy_arm
arduino-cli upload --fqbn teensy:avr:teensy41 -p <port> ar4_hardware_interface/ar4_firmware/teensy_arm
```

`<port>` is the Teensy as listed by `arduino-cli board list`. On Linux, the
[Teensy udev rules](https://www.pjrc.com/teensy/00-teensy.rules) must be installed for uploading.

Connect the Teensy directly to the computer, not through a USB hub shared with other devices: the encoders are powered
from the Teensy, and on a shared hub their counts can freeze while several motors run.

## Serial protocol

Commands end with `\n`. Joint values are in degrees.

| Command | Reply | Description |
| --- | --- | --- |
| `STA<version>` | `STA<1\|0>B<version>` | Handshake; the firmware errors out if the version does not match. |
| `JP` | `JPA<j1>B<j2>...F<j6>` | Read the joint positions from the encoders. |
| `MTA<j1>B<j2>...F<j6>` | `JPA...` | Move to the given joint positions; replies with the current positions. |
| `SSA<speed1>B<accel1>...L<accel6>` | `JPA...` | Set the max speed (deg/s) and acceleration (deg/s²) per joint. |
| `JC` | `JCA<enc1>B<enc2>...F<enc6>` | Calibrate all joints against their limit switches and move to the rest pose. |
| `JC<mask>` | `JCA...` | Calibrate only the joints marked `1` in a 6-digit mask, e.g. `JC000100` calibrates J4 only. |

### Calibration messages

During `JC` the firmware prints progress lines prefixed with `DBCAL`, so a calibration can be followed and debugged:

- `DBCAL start pins ...`: limit switch states when calibration starts.
- `DBCAL J<n> pin <0|1> t=<ms>`: raw limit switch changes (up to 20 per joint), to spot electrical noise.
- `DBCAL J<n> switch reached t=<ms> enc=<count>`: a joint reached its limit switch.
- `DBCAL status t=<ms> waiting J<n>:enc=<count> ...`: every 2 s, the encoder counts of the joints still searching.
- `DBCAL backed off switches, moving to rest`, then `DBCAL rest status J<n>:<steps>/<target> ...` every 2 s.
- `DBCAL rest reached`, followed by the `JC...` reply.

The `DB` prefix ends a driver exchange early, so run the calibration from a serial terminal and start the ROS bringup
with `calibrate:=False` when using this firmware.
