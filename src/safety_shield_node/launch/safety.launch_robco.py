import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('safety_shield_node')
    cfg_dir = os.path.join(pkg_share, 'config')

    traj_cfg = os.path.join(cfg_dir, 'trajectory_parameters_robco.yaml')
    robot_cfg = os.path.join(cfg_dir, 'robot_parameters_robco.yaml')
    mocap_cfg = os.path.join(cfg_dir, 'human_reach_TUM_lab.yaml')
    params = os.path.join(cfg_dir, 'safety_shield_params_robco.yaml')

    # Launch argument to choose executable
    exec_arg = DeclareLaunchArgument(
        name='safety_node_exec',
        default_value='safety_shield_node',
        description='Executable for the safety shield node (e.g., safety_shield_node or safety_shield_node_hard_break_recover)'
    )

    # Main node using the selected executable
    shield_node = Node(
        package='safety_shield_node',
        executable=LaunchConfiguration('safety_node_exec'),
        name=LaunchConfiguration('safety_node_exec'),
        output='screen',
        parameters=[
            params,
            {
                'trajectory_config': traj_cfg,
                'robot_config': robot_cfg,
                'mocap_config': mocap_cfg,
            },
        ],
    )

    # Shutdown when node exits
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=shield_node,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        exec_arg,
        shield_node,
        exit_handler
    ])
