#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright 2025 Ekumen, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    headless = LaunchConfiguration('headless')  # noqa F841
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Launch Isaac in headless mode',
    )

    pkg_ar4_isaac = get_package_share_directory('ar4_isaac')
    run_sim_path = os.path.join(pkg_ar4_isaac, 'scripts', 'run_sim.py')

    isaac_process = ExecuteProcess(
        cmd=[
            '/isaac-sim/python.sh',
            run_sim_path,
            '--robot_model_path',
            os.path.join(pkg_ar4_isaac, 'usda', 'ar4.usda'),
        ],
        shell=True,
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    isaac_process_headless = ExecuteProcess(
        cmd=[
            '/isaac-sim/python.sh',
            run_sim_path,
            '--robot_model_path',
            os.path.join(pkg_ar4_isaac, 'usda', 'ar4.usda'),
            '--headless',
        ],
        shell=True,
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless)
    ld.add_action(isaac_process)
    ld.add_action(isaac_process_headless)

    return ld
