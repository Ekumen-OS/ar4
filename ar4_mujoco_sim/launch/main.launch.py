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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

import os


def get_robot_description():
    return


def generate_launch_description():
    this_pkg_share = get_package_share_directory("ar4_mujoco_sim")

    mjcf_equiv_robot_description_file = os.path.join(
        this_pkg_share,
        "xacro",
        "mj_ar4.urdf.xacro",
    )

    robot_description_urdf = xacro.process_file(
        mjcf_equiv_robot_description_file
    ).toprettyxml(indent="  ")

    rsp_robot_description_file = os.path.join(
        get_package_share_directory("ar4_description"),
        "urdf",
        "ar4.urdf.xacro",
    )

    rsp_robot_description_urdf = xacro.process_file(
        rsp_robot_description_file
    ).toprettyxml(indent="  ")

    mujoco_model = os.path.join(
        this_pkg_share,
        "mjcf",
        "scene.xml",
    )

    controller_config_file = os.path.join(
        get_package_share_directory("ar4_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    mujoco_ros2_control_params = {
        "robot_description": robot_description_urdf,
        "mujoco_model_path": mujoco_model,
    }

    mujoco_sim_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            mujoco_ros2_control_params,
            controller_config_file,
        ],
    )

    # this nodes is launched with the HAL-specific nodes because we need to load
    # the simulation specific version
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": rsp_robot_description_urdf,
            }
        ],
    )

    spawn_joint_state_broadcaster_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            mujoco_sim_node,
            robot_state_publisher_node,
            spawn_joint_state_broadcaster_controller,
            spawn_joint_trajectory_controller,
        ]
    )
