"""Dry-run by default; hardware command path starts only on explicit enable_drive."""
from launch import LaunchDescription
import math

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_width(context):
    # Reject before any nodes (including the hardware group) can start.
    width = float(LaunchConfiguration('lane_min_width_m').perform(context))
    enabled = ParameterValue(LaunchConfiguration('enable_drive'), value_type=bool).evaluate(context)
    if not math.isfinite(width) or width <= 0:
        raise ValueError('lane_min_width_m must be finite and positive')
    if enabled and width != 3.50:
        raise ValueError('lane_min_width_m override is DRY-RUN only; DRIVE requires 3.50 m')
    return []


def generate_launch_description():
    enabled = LaunchConfiguration('enable_drive')
    return LaunchDescription([
        DeclareLaunchArgument('enable_drive', default_value='false'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('lane_min_width_m', default_value='3.50',
                             description='Minimum lane width in meters; override is DRY-RUN only'),
        OpaqueFunction(function=validate_width),
        Node(package='fma_control', executable='lane_follow', output='screen', parameters=[{
            'enable_drive': ParameterValue(enabled, value_type=bool),
            'device': LaunchConfiguration('device'),
            'lane_min_width_m': ParameterValue(LaunchConfiguration('lane_min_width_m'), value_type=float),
        }]),
    ])
