# launch/rviz.launch.py
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('safety_shield_node')
    cfg_dir = os.path.join(pkg, 'rviz')
    rviz_cfg = os.path.join(cfg_dir, 'safety_shield.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        )
    ])
