import pytest

from fma_mission.control_mode import (
    CONTROL_MODES, control_mode_for_mission, gps_fallback_allowed)


def test_modes_and_unknown_mission():
    assert CONTROL_MODES == ('LANE', 'GPS', 'STOP', 'OBSTACLE', 'REVERSE', 'FINISH')
    assert control_mode_for_mission('UNKNOWN') == 'STOP'


@pytest.mark.parametrize('state', ['PAIR', 'SINGLE_LEFT', 'SINGLE_RIGHT', 'DEGRADED'])
@pytest.mark.parametrize('valid', [True, False])
def test_usable_lane_never_requests_gps(state, valid):
    assert not gps_fallback_allowed(
        lane_valid=valid, lane_state=state, gps_controller_ready=True)


def test_gps_fallback_requires_explicit_invalid_none_and_controller():
    assert not gps_fallback_allowed(lane_valid=False, lane_state='NONE')
    assert not gps_fallback_allowed(
        lane_valid=True, lane_state='NONE', gps_controller_ready=True)
    assert not gps_fallback_allowed(
        lane_valid=None, lane_state=None, gps_controller_ready=True)
    assert gps_fallback_allowed(
        lane_valid=False, lane_state='NONE', gps_controller_ready=True)


@pytest.mark.parametrize('mission,phase,proceed,expected', [
    ('START', None, False, 'STOP'),
    ('NORMAL_DRIVE', None, False, 'LANE'),
    ('NORMAL_DRIVE', 'COMPLETE', False, 'LANE'),
    ('RAMP', 'ENTRY', False, 'LANE'),
    ('S_CURVE', 'GUIDE', True, 'OBSTACLE'),
    ('CHILD_DUMMY', 'GUIDE', True, 'OBSTACLE'),
    ('PERPENDICULAR_PARKING', 'APPROACH', False, 'LANE'),
    ('PERPENDICULAR_PARKING', 'ALIGN', False, 'LANE'),
    ('PERPENDICULAR_PARKING', 'REVERSE', False, 'REVERSE'),
    ('PERPENDICULAR_PARKING', 'PARKED', False, 'STOP'),
    ('PERPENDICULAR_PARKING', 'COMPLETE', False, 'STOP'),
    ('PARALLEL_PARKING', 'APPROACH', False, 'LANE'),
    ('PARALLEL_PARKING', 'REVERSE', False, 'REVERSE'),
    ('PARALLEL_PARKING', 'COMPLETE', False, 'STOP'),
    ('FINISH', 'EXIT', True, 'FINISH'),
])
def test_mission_phase_policy(mission, phase, proceed, expected):
    assert control_mode_for_mission(mission, phase, proceed=proceed) == expected


@pytest.mark.parametrize('mission', ['INTERSECTION_STRAIGHT_1', 'INTERSECTION_STRAIGHT_2',
    'INTERSECTION_LEFT', 'INTERSECTION_RIGHT'])
def test_perception_gated_missions(mission):
    assert control_mode_for_mission(mission, 'ENTRY') == 'STOP'
    assert control_mode_for_mission(mission, 'ENTRY', proceed=True) == 'LANE'
    assert control_mode_for_mission(mission, 'EXIT') == 'LANE'


@pytest.mark.parametrize('phase', [None, 'ENTRY', 'GUIDE', 'EXIT'])
def test_signal_car_keeps_lane_follow_while_waiting_for_signal(phase):
    assert control_mode_for_mission(
        'SIGNAL_CAR',
        phase,
        proceed=False,
    ) == 'LANE'

    assert control_mode_for_mission(
        'SIGNAL_CAR',
        phase,
        proceed=True,
    ) == 'LANE'


def test_normal_drive_can_select_gps_route_policy():
    assert control_mode_for_mission(
        'NORMAL_DRIVE', normal_drive_mode='GPS') == 'GPS'
    assert control_mode_for_mission(
        'NORMAL_DRIVE', normal_drive_mode='LANE') == 'LANE'
    assert control_mode_for_mission(
        'NORMAL_DRIVE', normal_drive_mode='STOP') == 'STOP'
