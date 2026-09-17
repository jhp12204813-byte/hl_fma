"""Low-speed field stop test; explicit drive only, configurable raw PWM, latched STOP."""
from launch import LaunchDescription
import math

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def validate_width(context):
    # Reject before any nodes (including the hardware group) can start.
    width = float(LaunchConfiguration('lane_min_width_m').perform(context))
    if not math.isfinite(width) or width <= 0:
        raise ValueError('lane_min_width_m must be finite and positive')
    distance = float(LaunchConfiguration('stop_trigger_distance_m').perform(context))
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('stop_trigger_distance_m must be finite and positive')
    from fma_perception.stop_line_detector import StopLineConfig
    thickness = float(LaunchConfiguration('stop_min_thickness_m').perform(context))
    if not math.isfinite(thickness) or not 0 < thickness <= StopLineConfig().max_thickness_m:
        raise ValueError('stop_min_thickness_m outside detector range')
    control_max = float(LaunchConfiguration('single_side_control_max_m').perform(context))
    pair_max = float(LaunchConfiguration('pair_control_max_m').perform(context))
    if not math.isfinite(pair_max) or not 1.8 <= pair_max < 4.0:
        raise ValueError('pair_control_max_m must be within [1.8, 4.0) m')
    if not math.isfinite(control_max) or not 1.8 <= control_max <= 5.20:
        raise ValueError('single_side_control_max_m must be within 1.8..5.20 m')
    pwm = int(LaunchConfiguration('drive_pwm').perform(context))
    if not 0 <= pwm <= 799:
        raise ValueError('drive_pwm must be an integer within 0..799')
    approach = int(LaunchConfiguration('stop_approach_pwm').perform(context))
    timeout = float(LaunchConfiguration('stop_approach_timeout_sec').perform(context))
    if not 0 <= approach <= 799 or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('invalid stop approach PWM or timeout')
    return []


def generate_launch_description(competition=False):
    enabled = LaunchConfiguration('enable_drive')
    extra_defaults = {}
    if competition:
        from fma_perception.competition_lane_tracker import MetricLaneConfig
        extra_defaults = {**vars(MetricLaneConfig()), 'single_drive_pwm': 64, 'degraded_pwm': 40,
                          'pair_return_blend_sec': .3, 'obstacle_stop_topic': '/safety/obstacle_stop',
                          'c920_lock_exposure': True, 'c920_manual_exposure': 156,
                          'c920_gain': 0}
    def validate_competition(context):
        if competition:
            MetricLaneConfig(**{name: type(value)(LaunchConfiguration(name).perform(context))
                                for name, value in vars(MetricLaneConfig()).items()})
            single = int(LaunchConfiguration('single_drive_pwm').perform(context))
            degraded = int(LaunchConfiguration('degraded_pwm').perform(context))
            blend = float(LaunchConfiguration('pair_return_blend_sec').perform(context))
            if not 0 <= degraded <= single <= 799 or not math.isfinite(blend) or blend <= 0:
                raise ValueError('invalid competition PWM/blend settings')
            exposure = int(LaunchConfiguration('c920_manual_exposure').perform(context))
            gain = int(LaunchConfiguration('c920_gain').perform(context))
            if not 3 <= exposure <= 2047:
                raise ValueError('c920_manual_exposure must be within 3..2047')
            if not 0 <= gain <= 255:
                raise ValueError('c920_gain must be within 0..255')
        return []
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=str(value)) for name, value in extra_defaults.items()],
        DeclareLaunchArgument('enable_drive', default_value='false'),
        DeclareLaunchArgument('max_frame_age_sec', default_value='0.35'),
        DeclareLaunchArgument('drive_pwm', default_value='120' if competition else '40'),
        DeclareLaunchArgument('stop_approach_pwm', default_value='40'),
        DeclareLaunchArgument('stop_approach_timeout_sec', default_value='3.0'),
        DeclareLaunchArgument('device', default_value='/dev/video2' if competition else '/dev/video14'),
        DeclareLaunchArgument('lane_min_width_m', default_value='3.00' if competition else '3.50',
                             description='Field test minimum lane width; production default remains 3.50 m'),
        DeclareLaunchArgument('stop_trigger_distance_m', default_value='0.40' if competition else '0.60'),
        DeclareLaunchArgument('allow_single_side_test', default_value='false'),
        DeclareLaunchArgument('single_side_control_max_m', default_value='3.30'),
        DeclareLaunchArgument('pair_control_max_m', default_value='3.0' if competition else '3.5'),
        DeclareLaunchArgument('stop_min_thickness_m', default_value='0.30' if competition else '0.04'),
        OpaqueFunction(function=validate_width),
        OpaqueFunction(function=validate_competition),
        Node(package='fma_control', executable='competition_lane' if competition else 'lane_stop_test', output='screen', parameters=[{
            **{name: ParameterValue(LaunchConfiguration(name), value_type=type(value))
               for name, value in extra_defaults.items()},
            'enable_drive': ParameterValue(enabled, value_type=bool),
            'max_frame_age_sec': ParameterValue(LaunchConfiguration('max_frame_age_sec'), value_type=float),
            'drive_pwm': ParameterValue(LaunchConfiguration('drive_pwm'), value_type=int),
            'device': LaunchConfiguration('device'),
            'stop_approach_pwm': ParameterValue(LaunchConfiguration('stop_approach_pwm'), value_type=int),
            'stop_approach_timeout_sec': ParameterValue(LaunchConfiguration('stop_approach_timeout_sec'), value_type=float),
            'single_side_control_max_m': ParameterValue(LaunchConfiguration('single_side_control_max_m'), value_type=float),
            'pair_control_max_m': ParameterValue(LaunchConfiguration('pair_control_max_m'), value_type=float),
            'allow_single_side_test': ParameterValue(LaunchConfiguration('allow_single_side_test'), value_type=bool),
            'stop_min_thickness_m': ParameterValue(LaunchConfiguration('stop_min_thickness_m'), value_type=float),
            'stop_trigger_distance_m': ParameterValue(LaunchConfiguration('stop_trigger_distance_m'), value_type=float),
            'lane_min_width_m': ParameterValue(LaunchConfiguration('lane_min_width_m'), value_type=float),
        }]),
    ])
