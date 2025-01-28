# AR4

The Annin Robotics AR4 robot is a 6DOF desktop size industrial robot that is a free, open plan low cost robot. This repository contains the necessary ROS 2 packages to plan and execute motions in Gazebo sim. It provides the arm description, simulation controllers and Moveit configuration needed to command it.

<p align="center">
  <img src="docs/ar4.png" width=500 />
</p>

## Package Summary

- [`ar4_control`](./ar4_control): Contains the configuration for the simulation ros2_control controller.
- [`ar4_gazebo`](./ar4_gazebo): Gazebo simulation for the ar4 arm.
- [`ar4_description`](./ar4_description): Contains the URDF of the arm.
- [`ar4_moveit_config`](./ar4_moveit_config): Contains configuration and launch files to run and command the arm.

## Installation


### Docker

#### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed.

* Make sure you have the drivers installed:

```sh
nvidia-smi
```

* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

#### Building image and running container

Build and run the dev container:

```sh
./docker/run.sh
```

#### Joining an open session

To join a running instance of the container:

```
./docker/join.sh
```