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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
import xacro


pkg_ar4_description = get_package_share_directory('ar4_description')
pkg_ar4_gazebo = get_package_share_directory('ar4_gazebo')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


def get_robot_description(use_ros_control):
    # Parse robot description from xacro
    robot_description_file = os.path.join(
        pkg_ar4_gazebo, 'urdf', 'ar4_gazebo.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description_file,
        mappings={
            'use_gazebo_ros_control': use_ros_control,
        },
    )
    robot_description = robot_description_config.toprettyxml(indent='  ')
    # Passing absolute path to the robot description due to Gazebo issues finding
    # ar4_description pkg path.
    robot_description = robot_description.replace(
        'package://ar4_description/', f'file://{pkg_ar4_description}/'
    )
    return robot_description


def generate_launch_description():
    """Nodes launched.

    robot_state_publisher
    robot_state_publisher_control
    jsp_gui
    spawn.
    """
    rsp_arg = DeclareLaunchArgument(
        'rsp', default_value='false', description='Run robot state publisher node.'
    )
    jsp_gui_arg = DeclareLaunchArgument(
        'jsp_gui',
        default_value='false',
        description='Run joint state publisher gui node.',
    )
    use_ros_control = LaunchConfiguration('use_ros_control')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_ros_control = DeclareLaunchArgument(
        name='use_ros_control',
        default_value='true',
        description='True to use the gazebo_ros_control plugin',
    )

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': get_robot_description('false'),
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(PythonExpression(["'", use_ros_control, "' == 'false'"])),
    )

    # Robot state publisher
    robot_state_publisher_control = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': get_robot_description('true'),
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(use_ros_control),
    )

    # Joint state publisher
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='ar4',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'ar4',
            '-topic',
            'robot_description',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'joint_state_broadcaster',
        ],
        output='screen',
        condition=IfCondition(use_ros_control),
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2',
            'control',
            'load_controller',
            '--set-state',
            'active',
            'arm_controller',
        ],
        output='screen',
        condition=IfCondition(use_ros_control),
    )

    ld = LaunchDescription(
        [
            # # Arguments and Nodes
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            # Arguments and Nodes
            declare_use_ros_control,
            jsp_gui_arg,
            rsp_arg,
            jsp_gui,
            robot_state_publisher,
            robot_state_publisher_control,
            spawn,
            use_sim_time_argument,
        ]
    )

    return ld
