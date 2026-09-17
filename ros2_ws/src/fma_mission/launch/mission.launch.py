"""Course manager with its required stop-command companion; no drive bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=PathJoinSubstitution([
            FindPackageShare('fma_mission'), 'config', 'waypoints.yaml'])),
        Node(package='fma_mission', executable='mission_safety', output='screen'),
        Node(package='fma_mission', executable='mission_manager', output='screen',
             parameters=[{'waypoints_file': LaunchConfiguration('waypoints_file')}]),
    ])
