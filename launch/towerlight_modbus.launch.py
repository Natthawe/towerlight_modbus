from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='towerlight_modbus',
            executable='towerlight_modbus_node',
            name='towerlight_modbus_node',
            output='screen',
        ),
    ])
