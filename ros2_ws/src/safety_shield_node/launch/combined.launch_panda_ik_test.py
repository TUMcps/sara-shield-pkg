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

    # Get directories
    pkg_share = get_package_share_directory('safety_shield_node')
    ik_pkg_share = get_package_share_directory('movit_panda_ik')
    cfg_dir = os.path.join(pkg_share, 'config')
    rviz_dir = os.path.join(pkg_share, 'rviz')

    # IK config
    srdf_file = os.path.join(ik_pkg_share, 'config', 'panda.srdf')
    kinematics_file = os.path.join(ik_pkg_share, 'config', 'kinematics.yaml')
    with open(srdf_file, 'r') as infp:
        srdf_content = infp.read()
    with open(kinematics_file, 'r') as f:
        kin_yaml = yaml.safe_load(f)
    kinematics_params = kin_yaml['cartesian_to_joint_node']['ros__parameters']

    # Config files
    traj_cfg = os.path.join(cfg_dir, 'trajectory_parameters_panda.yaml')
    robot_cfg = os.path.join(cfg_dir, 'robot_parameters_panda.yaml')
    mocap_cfg = os.path.join(cfg_dir, 'human_reach_TUM_lab.yaml')
    params = os.path.join(cfg_dir, 'safety_shield_params_panda.yaml')
    rviz_cfg = os.path.join(rviz_dir, 'panda_ik.rviz')

    return LaunchDescription([

        # --- Launch Args ---
        DeclareLaunchArgument(
            name='use_gui',
            default_value='true',
            description='Enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='safety_node_exec',
            default_value='safety_shield_node',
            description='Which safety node executable to run'
        ),
        DeclareLaunchArgument(
            name='enable_ik_node',
            default_value='true',
            description='Whether to launch the IK node'
        ),

        # --- Robot State Publisher ---
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
        
        # --- Safety Shield Node ---
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

        # --- IK Node (conditional) ---
        Node(
            condition=IfCondition(LaunchConfiguration('enable_ik_node')),
            package='movit_panda_ik',
            executable='cartesian_to_joint_node',
            name='cartesian_to_joint_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content},
                {'robot_description_semantic': srdf_content},
                {'robot_description_kinematics': kinematics_params}
            ]
        ),

        # --- RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
