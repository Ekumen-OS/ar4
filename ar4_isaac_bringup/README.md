# Description

Bringup package for the AR4 software stack on the Isaac simulator.

```bash
colcon build --packages-up-to ar4_isaac_bringup
. install/setup.bash
```

To run the simulation, source and run:

```bash
ros2 launch ar4_isaac_bringup main.launch.py
```

#### Launch file arguments

To see a detailed list of the arguments, run:

```bash
ros2 launch ar4_isaac_bringup main.launch.py --show-args
```
