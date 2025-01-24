#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


pkg_ar4_description = get_package_share_directory('ar4_description')
pkg_ar4_gazebo = get_package_share_directory('ar4_gazebo')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value='true',
                                             description='Use simulation (Gazebo) clock if true')

    # Launch ar4 robot
    ar4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ar4_gazebo, 'launch', 'spawn_ar4_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ar4_description, 'config', 'ar4_vis.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription(
        [
            # Arguments and Nodes
            rviz_arg,
            use_sim_time_arg,
            ar4_launch,
            gazebo,
            rviz,
        ]
    )

    return ld
