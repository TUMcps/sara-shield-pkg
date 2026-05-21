"""
Launch file for testing a robot in RViz with joint state publisher and robot state publisher.
This launch file uses a URDF file for the robot description and allows for checking the urdf visualization and creating the robot.rviz file
Steps:
1. inlcude urdf and meshes inside urdf folder
2. update the link to meshes in the urdf file
3. load the urdf with this file
4. check the visualization in RViz
5. save the RViz configuration as robot.rviz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the URDF file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('safety_shield_node'),
        'urdf',
        'ur_robots', # path to urdf
        'ur30.urdf' # name of urdf file
    ])
    
    # Read URDF content
    robot_description_content = Command(['xacro ', urdf_file])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gui',
            default_value='true',
            description='Flag to enable joint_state_publisher_gui'
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
            output='screen'
        ),
        
        # Joint State Publisher GUI (when use_gui=true)
        Node(
            condition=IfCondition(LaunchConfiguration('use_gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # Joint State Publisher (when use_gui=false)
        Node(
            condition=UnlessCondition(LaunchConfiguration('use_gui')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # RViz2 (with saved config)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('safety_shield_node'),
                'rviz',
                'schunk.rviz'
            ])],
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
