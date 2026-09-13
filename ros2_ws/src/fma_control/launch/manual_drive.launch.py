"""Manual-only ROS path. Does not start teleop, perception or mission nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/fma_stm32'),
        DeclareLaunchArgument('receive_only', default_value='false'),
        Node(package='fma_control', executable='command_arbiter', parameters=[{
            # Manual timeout must not hand control back to autonomous inputs.
            'lane_topic': '/manual_only/unused_lane',
            'mission_topic': '/manual_only/unused_mission',
        }]),
        Node(package='fma_vehicle', executable='vehicle_controller'),
        Node(package='fma_vehicle', executable='stm32_bridge_node', parameters=[{
            'port': LaunchConfiguration('port'),
            'receive_only': ParameterValue(LaunchConfiguration('receive_only'), value_type=bool),
        }]),
    ])
