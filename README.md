# AR4

The [Annin Robotics AR4](https://www.anninrobotics.com/) robot is a 6DOF desktop size industrial robot that is a free, open plan low cost robot. This repository contains the necessary ROS 2 packages to plan and execute motions in Gazebo sim. It provides the arm description, simulation controllers and Moveit configuration needed to command it.

<p align="center">
  <img src="docs/ar4.png" width=500 />
</p>

## Package Summary

- [`ar4_gazebo`](./ar4_gazebo): Gazebo simulation for the ar4 arm.
- [`ar4_description`](./ar4_description): Contains the URDF of the arm.
- [`ar4_hardware_interface`](./ar4_hardware_interface): Contains the software required to connect the computer with the real robot.
- [`ar4_moveit_config`](./ar4_moveit_config): Contains configuration and launch files to run and command the arm.

## Installation

### Prerequisites

It is a requirement to have `docker engine` with the `docker compose plugin` already installed in the host machine.

See: [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

### Running the dev container

Build and run the container for the use case you are interested in

#### Gazebo
```bash
./docker/run.sh -s ar4_gazebo
```

#### Hardware

* [Hardware Interface Instructions](ar4_hardware_interface/README.md)

## Build the packages

Build the packages

```
colcon build
```

Source the built packages

```
source install/setup.bash
```

---

### Launch

After building and sourcing the packages, run the specific launch file for your use case:

#### Gazebo

```
ros2 launch ar4_gazebo moveit.launch.py
```

![Ar4 Gazebo](docs/ar4.png)

---

### Controlling the AR4 with MoveIt

To plan and command the arm to execute a motion, this launch file will also start MoveIt automatically. Once launched, you should see RViz showing the robot visualization and the MotionPlanning panel on the left.

There are two ways of selecting a target position for the arm using `RViz`:
1. Selecting a random valid position.
2. Moving the end effector to a desired position.


####  1. Selecting random valid position
This will select a random position for the arm that would not cause a collision with itself or objects around it, calculated from the semantic information of the robot.

[Random valid position video](https://github.com/user-attachments/assets/be9406d2-6589-456c-8ae6-edbbe067b701)


####  2. Moving end effector to a desired position
This allows you to select a goal position for the end effector, which is currently the last link in the arm as no gripper is being used. This is done by dragging and dropping where the end effector should move to.

[User selected position video](https://github.com/user-attachments/assets/a3057320-02ba-4898-8c07-b08d86ec0dcf)


## Licenses

All packages in this repository except for `ar4_description` and `ar4_hardware_interface` are distributed under a **BSD 3-Clause** License.

`ar4_description` is a derivative work from the [ar4_ros_driver](https://github.com/ycheng517/ar4_ros_driver/tree/main/annin_ar4_description) repository, which is distributed under a **MIT License**.
