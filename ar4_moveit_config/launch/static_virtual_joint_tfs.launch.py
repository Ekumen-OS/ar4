from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

"""Spawns a static_transform_publisher for every virtual joint in the ar4.srdf configuration file.
Note: If ar4_gazebo package is used to spawn the ar4 model, a virtual joint will already be published.
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
