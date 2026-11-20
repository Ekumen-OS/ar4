# Description

Bringup package for the AR4 software stack on the MuJoCo simulator. To build the package, run

```bash
colcon build --packages-up-to ar4_mujoco_bringup
. install/setup.bash
```

To run the simulation, source and run:

```bash
ros2 launch ar4_mujoco_bringup main.launch.py
```

## Instructions to update the MuJoCo robot model

The MuJoCo model of the AR4 robot can be imported from the URDF description in the `ar4_description` package. The process requires running the two scripts in the `scripts/` folder in sequence.

1. First run `stage1_create_urdf_from_xacro.sh` from **within** the development container. This script will generate the URDF file from the xacro description.
2. Then run `stage2_create_mjcf_from_urdf.sh` from **outside** the development container (this script needs to create a new containerized environment running Ubuntu 24.04). This script will generate the MuJoCo model from the URDF file.
3. For the gripper mimic function to work correctly, this code needs to be added into the [mj_ar4.mjcf](../ar4_mujoco_sim/mjcf/robot/mj_ar4.mjcf) file

```
  <equality>
    <joint joint1="finger_joint_1" joint2="finger_joint_2" solimp="0.95 0.99 0.001" solref="0.005 1"/>
  </equality>

```

At this point you'll see the `urdf/` and `mjcf/` folders in the `ar4_mujoco_sim` package as updated. Commit those changes.
