"""Control policy independent of course progress; no maneuver implementation."""

CONTROL_MODES = ('LANE', 'GPS', 'STOP', 'OBSTACLE', 'REVERSE', 'FINISH')

# Course policy only. Controller readiness is enforced separately by mission_safety.
MISSION_CONTROL_MODES = {
    'START': 'STOP',
    'NORMAL_DRIVE': 'LANE',
    'RAMP': 'LANE',
    'INTERSECTION_STRAIGHT_1': 'LANE',
    'S_CURVE': 'OBSTACLE',
    'INTERSECTION_STRAIGHT_2': 'LANE',
    'PERPENDICULAR_PARKING': 'STOP',
    'INTERSECTION_LEFT': 'LANE',
    'CHILD_DUMMY': 'OBSTACLE',
    'PARALLEL_PARKING': 'STOP',
    'INTERSECTION_RIGHT': 'LANE',
    'SIGNAL_CAR': 'LANE',
    'LANE_CHANGE': 'LANE',
    'FINISH': 'FINISH',
}


def control_mode_for_mission(
        mission, phase=None, *, proceed=False, normal_drive_mode='LANE'):
    if mission == 'NORMAL_DRIVE':
        return normal_drive_mode if normal_drive_mode in ('LANE', 'GPS') else 'STOP'
    if mission in ('PERPENDICULAR_PARKING', 'PARALLEL_PARKING'):
        return {'APPROACH': 'LANE', 'ALIGN': 'LANE', 'REVERSE': 'REVERSE',
                'PARKED': 'STOP', 'COMPLETE': 'STOP'}.get(phase, 'STOP')
    if mission.startswith('INTERSECTION_'):
        if phase == 'EXIT':
            return 'LANE'
        # Intersections may still wait for perception/feedback.
        if phase is not None:
            return 'LANE' if proceed else 'STOP'
    return MISSION_CONTROL_MODES.get(mission, 'STOP')


def gps_fallback_allowed(*, lane_valid, lane_state, gps_controller_ready=False):
    """Reserved eligibility check, not wired to automatic mode transitions.

    SINGLE/DEGRADED remain usable; missing data alone is not a fallback request.
    A future controller must explicitly declare readiness before GPS is allowed.
    """
    return (gps_controller_ready and lane_valid is False and lane_state == 'NONE')
