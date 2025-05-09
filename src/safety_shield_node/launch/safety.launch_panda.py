# launch/safety.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('safety_shield_node')
    cfg_dir   = os.path.join(pkg_share, 'config')

    # Full paths to all your YAMLs
    traj_cfg  = os.path.join(cfg_dir, 'trajectory_parameters_panda.yaml')
    robot_cfg = os.path.join(cfg_dir, 'robot_parameters_panda.yaml')
    mocap_cfg = os.path.join(cfg_dir, 'mujoco_mocap.yaml')
    params    = os.path.join(cfg_dir, 'safety_shield_params_panda.yaml')

    return LaunchDescription([
        Node(
            package='safety_shield_node',
            executable='safety_shield_node',
            name='safety_shield_node',
            output='screen',
            parameters=[
                params,  # loads all generic parameters (including shield_type, init.qpos, etc.)
                {       # then override just the file‐paths
                  'trajectory_config': traj_cfg,
                  'robot_config'     : robot_cfg,
                  'mocap_config'     : mocap_cfg,
                }
            ],
        )
    ])
