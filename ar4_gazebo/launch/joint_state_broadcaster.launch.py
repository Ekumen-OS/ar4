from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_state_broadcaster',
            executable='joint_state_broadcaster',
            name='joint_state_broadcaster',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
        ),
    ])