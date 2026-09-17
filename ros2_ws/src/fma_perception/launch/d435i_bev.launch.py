"""Perception only: never starts a camera driver, controller or STM32 bridge."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('fma_perception'))/'config/d435i_bev.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        Node(package='fma_perception', executable='lane_detector', name='lane_detector',
             parameters=[LaunchConfiguration('config')],
             remappings=[('/front/color/image_raw', LaunchConfiguration('image_topic')),
                         ('/front/color/camera_info', LaunchConfiguration('camera_info_topic'))]),
    ])
