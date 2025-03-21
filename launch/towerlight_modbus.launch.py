from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='towerlight_modbus',
            executable='optimize_towerlight_node',
            name='optimize_towerlight_node',
            output='screen',
            respawn=True,
        ),
    ])
