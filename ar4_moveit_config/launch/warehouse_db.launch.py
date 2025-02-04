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

"""Can be used to store persistent planning scenes and robot states.

By default warehouse_ros_mongo is set to be used, but that package is not currently available.
More info:
https://github.com/ros-planning/warehouse_ros_mongo/issues/75#issuecomment-1594971107
todo: Fix this launch file and move_group to use warehouse_ros_sqlite instead.
(might need changes here and move_group). More info about db change:
https://github.com/ros-planning/warehouse_ros_sqlite
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ar4", package_name="ar4_moveit_config"
    ).to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
