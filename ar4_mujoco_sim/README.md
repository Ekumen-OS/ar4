# Description
A Gazebo simulation of the AR4 Package.

To build the package, run
`colcon build`

To run the simulation, source and run
`ros2 launch ar4_mujoco_sim ar4_sim.launch.py`

#### Launch file arguments
- 'rsp':
    - Run [`robot state publisher`](https://github.com/ros/robot_state_publisher) node. (default: 'false')
- 'rviz':
    - Start RViz. (default: 'false')
