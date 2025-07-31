import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('safety_shield_node'),
        'urdf',
        'panda',
        'panda.urdf'
    ])

    # Read URDF content via xacro
    robot_description_content = Command(['xacro ', urdf_file])

    # Get package/share/config directories
    pkg_share = get_package_share_directory('safety_shield_node')
    cfg_dir = os.path.join(pkg_share, 'config')
    rviz_dir = os.path.join(pkg_share, 'rviz')

    # Config files
    traj_cfg = os.path.join(cfg_dir, 'trajectory_parameters_panda.yaml')
    robot_cfg = os.path.join(cfg_dir, 'robot_parameters_panda.yaml')
    mocap_cfg = os.path.join(cfg_dir, 'human_reach_TUM_lab.yaml')
    params = os.path.join(cfg_dir, 'safety_shield_params_panda.yaml')
    rviz_cfg = os.path.join(rviz_dir, 'panda.rviz')

    # Launch arguments
    return LaunchDescription([

        # Use GUI for joint_state_publisher
        DeclareLaunchArgument(
            name='use_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
        ),

        # Select which safety node executable to use
        DeclareLaunchArgument(
            name='safety_node_exec',
            default_value='safety_shield_node',
            description='Executable for safety shield node (e.g., safety_shield_node or safety_shield_node_hard_break_recover)'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }],
            remappings=[
                ('/joint_states', '/current_joint_states')
            ],
            output='screen'
        ),

        # Joint State Publisher GUI
        Node(
            condition=IfCondition(LaunchConfiguration('use_gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Joint State Publisher (headless)
        Node(
            condition=UnlessCondition(LaunchConfiguration('use_gui')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Safety Shield Node (customizable executable and name)
        Node(
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
                }
            ],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])