from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription

from launch_ros.actions import Node

"""Used to launch the Moveit fake controller (a FollowJointTrajectory controller)
configured by the Moveit Setup Assistant.
Note: If ar4_gazebo package is used to spawn the ar4 model, the robot state publisher
will be already spawned.
"""

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ld = LaunchDescription()
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld
