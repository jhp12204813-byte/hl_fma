import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_share = get_package_share_directory('fma_mission')

    route_file = os.path.join(
        mission_share,
        'config',
        'competition_route.yaml',
    )

    return LaunchDescription([
        # Course progress / mission state.
        Node(
            package='fma_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'waypoints_file': route_file,
            }],
        ),

        # GPS polyline/lookahead controller.
        # Output: /cmd/gps_raw
        Node(
            package='fma_localization',
            executable='dense_gps_controller',
            name='dense_gps_controller',
            output='screen',
            parameters=[{
                'enabled': True,
                'route_file': route_file,
                'search_forward_segments': 10,
            }],
        ),

        # GPS-primary lane-departure correction.
        # /cmd/gps_raw + /perception/lane -> /cmd/gps
        Node(
            package='fma_control',
            executable='lane_guard',
            name='lane_guard',
            output='screen',
            parameters=[{
                'enabled': True,
            }],
        ),

        # Mission/controller readiness fail-safe.
        Node(
            package='fma_mission',
            executable='mission_safety',
            name='mission_safety',
            output='screen',
        ),

        # Final command selection.
        # Vehicle controller / STM32 are intentionally NOT launched here.
        Node(
            package='fma_control',
            executable='command_arbiter',
            name='command_arbiter',
            output='screen',
        ),
    ])
