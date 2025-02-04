import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    ar4_gazebo_pkg = 'ar4_gazebo'
    ar4_moveit_config_pkg = 'ar4_moveit_config'

    # Declare launch argument for enabling/disabling the ros bridge
    use_ros_bridge = LaunchConfiguration('use_ros_bridge')
    
    declare_use_ros_bridge = DeclareLaunchArgument(
        'use_ros_bridge', default_value='true', description='Enable ROS bridge')

    # Launch ar4_in_empty_world.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare(ar4_gazebo_pkg).find(ar4_gazebo_pkg), 'launch', 'ar4_in_empty_world.launch.py')
        ])
    )

    # Launch gz_ros_bridge.launch.py after Gazebo if enabled
    gz_ros_bridge_launch = TimerAction(
        period=5.0,  # Adjust delay as needed
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(FindPackageShare(ar4_gazebo_pkg).find(ar4_gazebo_pkg), 'launch', 'gz_ros_bridge.launch.py')
                ])
            )
        ],
        condition=launch.conditions.IfCondition(use_ros_bridge)
    )

    # Launch demo.launch.py after gz_ros_bridge
    moveit_launch = TimerAction(
        period=10.0,  # Adjust delay as needed
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(FindPackageShare(ar4_moveit_config_pkg).find(ar4_moveit_config_pkg), 'launch', 'demo.launch.py')
                ])
            )
        ]
    )

    return LaunchDescription([
        declare_use_ros_bridge,
        gazebo_launch,
        gz_ros_bridge_launch,
        moveit_launch
    ])
