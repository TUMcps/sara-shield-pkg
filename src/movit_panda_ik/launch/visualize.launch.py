import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare("movit_panda_ik").find("movit_panda_ik")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "panda.urdf.xacro"])
    srdf_file = PathJoinSubstitution([pkg_share, "config", "panda.srdf"])
    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")
    rviz_config_file = os.path.join(pkg_share, "rviz", "panda.rviz")

    robot_description = Command(["xacro ", urdf_file])
    robot_description_semantic = Command(["cat ", srdf_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            name="use_gui",
            default_value="false",
            description="Flag to enable joint_state_publisher_gui"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False
            }],
            remappings=[("/joint_states", "/current_joint_states")]
        ),

        # Optional: joint_state_publisher_gui (if enabled manually)
        Node(
            condition=IfCondition(LaunchConfiguration("use_gui")),
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),

        # RViz for MoveIt
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[{
                "robot_description": robot_description,
                "robot_description_semantic": robot_description_semantic,
                "kinematics_yaml": kinematics_file,
                "use_sim_time": False
            }]
        )
    ])
