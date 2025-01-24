# Description
A Gazebo simulation of the AR4 Package.

To build the package, run
`colcon build`

To run the simulation, source and run
`ros2 launch ar4_gazebo ar4_sim.launch.py`

#### Launch file arguments
- 'jsp_gui':
    - Run [`joint state publisher gui`](https://github.com/ros/joint_state_publisher/tree/ros2) node. (default: 'false')
- 'rsp':
    - Run [`robot state publisher`](https://github.com/ros/robot_state_publisher) node. (default: 'false')
- 'rviz':
    - Start RViz. (default: 'false')
