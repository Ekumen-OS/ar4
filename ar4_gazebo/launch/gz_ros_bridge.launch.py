#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_ar4_gazebo = get_package_share_directory('ar4_gazebo')
    bridge_config_file_path = os.path.join(pkg_ar4_gazebo, 'config', 'bridge_config.yaml')

    bridge_process = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_file_path}'
        ],
        shell=True,
        output='screen',
    )

    return LaunchDescription([bridge_process])
