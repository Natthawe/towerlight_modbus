from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='towerlight_modbus',
            executable='final_emergency_towerlight_node',
            name='final_emergency_towerlight_node',
            output='screen',
            # respawn=True,
        ),
    ])
