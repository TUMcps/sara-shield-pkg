# launch/safety.launch.py
import os

from launch               import LaunchDescription
from launch.actions       import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions   import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('safety_shield_node')
    cfg_dir   = os.path.join(pkg_share, 'config')

    traj_cfg  = os.path.join(cfg_dir, 'trajectory_parameters_schunk.yaml')
    robot_cfg = os.path.join(cfg_dir, 'robot_parameters_schunk.yaml')
    mocap_cfg = os.path.join(cfg_dir, 'human_reach_TUM_lab.yaml')
    params    = os.path.join(cfg_dir, 'safety_shield_params.yaml')

    # --- the node itself ----------------------------------------------------
    shield_node = Node(
        package='safety_shield_node',
        executable='safety_shield_node_hard_break',
        name='safety_shield_node_hard_break',
        output='screen',
        parameters=[
            params,
            {
                'trajectory_config': traj_cfg,
                'robot_config'     : robot_cfg,
                'mocap_config'     : mocap_cfg,
            },
        ],
    )

    # --- when the node exits, stop the whole launch service -----------------
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=shield_node,
            on_exit=[Shutdown()]          # ← triggers an orderly shutdown
        )
    )

    return LaunchDescription([shield_node, exit_handler])