#!/usr/bin/env python3

# Launch file for the AR4 arm simulation.

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Declare the arguments for the LaunchDescription, visible with --show-args
ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
    DeclareLaunchArgument(
        'rsp', default_value='true', description='Run robot state publisher node.'
    ),
    DeclareLaunchArgument(
        'jsp', default_value='true', description='Run joint state publisher node.'
    ),
]

# Get the path of the necessary packages
pkg_ar4_description = get_package_share_directory('ar4_description')
ar4_urdf_file = xacro.process_file(
    os.path.join(pkg_ar4_description, 'urdf', 'ar4.urdf.xacro')
).toprettyxml(indent='  ')

description_launch_file = PathJoinSubstitution(
    [pkg_ar4_description, 'launch', 'rviz2.launch.py']
)


# Declare nodes
def declare_nodes():
    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ar4',
        output='both',
        parameters=[{'robot_description': ar4_urdf_file}],
        condition=IfCondition(LaunchConfiguration('rsp')),
    )
    # Joint state publisher
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='ar4',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp')),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ar4_description, 'config', 'ar4_vis.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return rsp, jsp, rviz


# Includes
robot_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([description_launch_file])
)


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    for node in declare_nodes():
        ld.add_action(node)
    return ld
