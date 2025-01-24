from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch

"""Used to open the the Moveit2 setup assistant for this package.
This could be used to modify the created package.
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
