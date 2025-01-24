from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch

"""Launches the robot state publisher.
Note: If ar4_gazebo package is used to spawn the ar4 model, the robot state publisher
will be already spawned.
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
