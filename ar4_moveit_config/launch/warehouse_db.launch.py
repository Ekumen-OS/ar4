from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch

"""Can be used to store persistent planning scenes and robot states.
By default warehouse_ros_mongo is set to be used, but that package is not currently available. More info:
https://github.com/ros-planning/warehouse_ros_mongo/issues/75#issuecomment-1594971107
todo: Fix this launch file and move_group to use warehouse_ros_sqlite instead.
(might need changes here and move_group). More info about db change:
https://github.com/ros-planning/warehouse_ros_sqlite
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
