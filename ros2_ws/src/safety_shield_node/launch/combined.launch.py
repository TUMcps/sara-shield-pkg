import yaml
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    # Get robot name from launch configuration
    robot_name = LaunchConfiguration('robot_name').perform(context)
    safety_node_exec = LaunchConfiguration('safety_node_exec').perform(context)
    sync_robot_position_str = LaunchConfiguration('sync_robot_position').perform(context)
    sync_robot_position = (sync_robot_position_str.lower() == 'true')

    # Get package/share directories
    pkg_share = get_package_share_directory('safety_shield_node')

    # Build file paths dynamically with robot-specific filenames
    urdf_file = os.path.join(pkg_share, 'urdf', robot_name, f'{robot_name}.urdf')

    # Robot-specific config files with robot name in filename
    traj_cfg = os.path.join(pkg_share, 'config', f'trajectory_parameters_{robot_name}.yaml')
    robot_cfg = os.path.join(pkg_share, 'config', f'robot_parameters_{robot_name}.yaml')
    params = os.path.join(pkg_share, 'config', f'safety_shield_params_{robot_name}.yaml')

    # Common config files (no robot name)
    mocap_cfg = os.path.join(pkg_share, 'config', 'human_reach_TUM_lab.yaml')

    # Select RViz config based on use_ik and robot_name
    rviz_cfg = os.path.join(pkg_share, 'rviz', f'{robot_name}.rviz')

    # Read URDF content via xacro
    robot_description_content = Command(['xacro ', urdf_file])

    # Prepare nodes list
    nodes = [
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

        # Safety Shield Node
        Node(
            package='safety_shield_node',
            executable=safety_node_exec,
            name=safety_node_exec,
            output='screen',
            parameters=[
                params,
                {
                    'robot_name': robot_name,
                    'trajectory_config': traj_cfg,
                    'robot_config': robot_cfg,
                    'mocap_config': mocap_cfg,
                    'sync_robot_position': sync_robot_position,
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
        )
    ]
    return nodes


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='panda',
        description='Robot name (e.g., panda, ur3, ur5, etc.)'
    )

    sync_robot_position_arg = DeclareLaunchArgument(
        name='sync_robot_position',
        default_value='false',
        description='Synchronize robot initial position flag'
    )

    use_gui_arg = DeclareLaunchArgument(
        name='use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    safety_node_exec_arg = DeclareLaunchArgument(
        name='safety_node_exec',
        default_value='safety_shield_node',
        description='Executable for safety shield node'
    )

    
    return LaunchDescription([
        robot_name_arg,
        use_gui_arg,
        safety_node_exec_arg,
        sync_robot_position_arg,
        OpaqueFunction(function=launch_setup)
    ])
