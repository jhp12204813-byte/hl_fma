from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_STM32_PORT = (
    '/dev/serial/by-id/'
    'usb-STMicroelectronics_STM32_STLink_0671FF505055877267173020-if02'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value=DEFAULT_STM32_PORT,
        ),
        DeclareLaunchArgument(
            'receive_only',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'allow_lane_pwm',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'allow_mission_pwm',
            default_value='true',
        ),

        # Command sources (manual/lane/mission) are arbitrated here.
        Node(
            package='fma_control',
            executable='command_arbiter',
            name='command_arbiter',
            output='screen',
            parameters=[{
                'allow_lane_pwm': ParameterValue(
                    LaunchConfiguration('allow_lane_pwm'),
                    value_type=bool,
                ),
                'allow_mission_pwm': ParameterValue(
                    LaunchConfiguration('allow_mission_pwm'),
                    value_type=bool,
                ),
            }],
        ),

        # Converts final DriveCommand into VehicleCommand.
        Node(
            package='fma_vehicle',
            executable='vehicle_controller',
            name='vehicle_controller',
            output='screen',
        ),

        # The ONLY process that owns the STM32 serial port.
        Node(
            package='fma_vehicle',
            executable='stm32_bridge_node',
            name='stm32_bridge_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'receive_only': ParameterValue(
                    LaunchConfiguration('receive_only'),
                    value_type=bool,
                ),
            }],
        ),
    ])
