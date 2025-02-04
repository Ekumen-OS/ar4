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
