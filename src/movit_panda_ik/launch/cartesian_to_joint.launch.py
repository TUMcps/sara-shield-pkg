from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'movit_panda_ik'
    share_dir = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(share_dir, 'config', 'panda.urdf.xacro')
    srdf_file = os.path.join(share_dir, 'config', 'panda.srdf')
    kinematics_file = os.path.join(share_dir, 'config', 'kinematics.yaml')

    # Load SRDF contents as string
    with open(srdf_file, 'r') as infp:
        srdf_content = infp.read()

    # Load kinematics.yaml as dict
    with open(kinematics_file, 'r') as f:
        kin_yaml = yaml.safe_load(f)

    # Extract just the parameters under ros__parameters
    kinematics_params = kin_yaml['cartesian_to_joint_node']['ros__parameters']

    # 👇 Print to confirm it's loading the correct structure
    print("=== Kinematics Params Loaded ===")
    print(kinematics_params)
    print("================================")
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', xacro_file])
            }]
        ),
        Node(
            package='movit_panda_ik',
            executable='cartesian_to_joint_node',
            name='cartesian_to_joint_node',
            output='screen',
            parameters=[
                {'robot_description': Command(['xacro ', xacro_file])},
                {'robot_description_semantic': srdf_content},
                {'robot_description_kinematics': kinematics_params}
            ]

        )
    ])
