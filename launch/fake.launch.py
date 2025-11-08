from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('towerlight_modbus')
    params_file = os.path.join(pkg_share, 'config', 'fake.yaml')

    return LaunchDescription([
        Node(
            package='towerlight_modbus',
            executable='fake_node',
            name='fake_node_params',
            output='screen',
            parameters=[params_file],
        )
    ])
