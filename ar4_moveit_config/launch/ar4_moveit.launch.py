from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ar4", package_name="ar4_moveit_config").to_moveit_configs()
    spawn_controllers = ExecuteProcess(
        cmd=['ros2', 'launch', 'moveit_config', 'spawn_controllers.launch.py'],
        output='screen',
    )
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/move_group.launch.py")
        ),
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_controllers,
                    on_exit=[move_group_launch],
                )
            ),
        ]
    )
