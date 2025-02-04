#!/usr/bin/env python3

# Copyright 2025 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_ar4_gazebo = get_package_share_directory('ar4_gazebo')
    bridge_config_file_path = os.path.join(
        pkg_ar4_gazebo, 'config', 'bridge_config.yaml'
    )

    bridge_process = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_file_path}',
        ],
        shell=True,
        output='screen',
    )

    return LaunchDescription([bridge_process])
