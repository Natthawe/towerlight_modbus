from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('towerlight_modbus')
    params_file = os.path.join(pkg_share, 'config', 'modbus_params.yaml')

    return LaunchDescription([
        Node(
            package='towerlight_modbus',
            executable='final_emergency_towerlight_node',
            name='final_emergency_towerlight_node',
            output='screen',
            parameters=[params_file],
        )
    ])
