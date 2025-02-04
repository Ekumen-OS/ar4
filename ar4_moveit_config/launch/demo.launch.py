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

from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    """Launch a self-contained demo.

    launch_package_path is optional to use different launch and config packages.

    Includes:
     * move_group
     * moveit_rviz
    """
    moveit_config = MoveItConfigsBuilder(
        "ar4", package_name="ar4_moveit_config"
    ).to_moveit_configs()

    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    return ld
