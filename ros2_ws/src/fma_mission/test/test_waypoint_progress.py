from dataclasses import replace
from pathlib import Path

import pytest
import yaml
from fma_interfaces.msg import MissionState
from fma_mission.waypoint_progress import load_waypoints, WaypointProgress, SOFT_NUMBERS
from fma_mission.control_mode import control_mode_for_mission

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/waypoints.yaml'


def progress():
    p = WaypointProgress(load_waypoints(CONFIG))
    p.start()
    return p


def arrive(p):
    return p.gps(p.target.latitude, p.target.longitude)


def until(p, number):
    for _ in range(100):
        if p.target.number == number:
            return
        arrive(p)
    pytest.fail('Progress blocked')


def test_final_exact_course():
    ws = load_waypoints(CONFIG)
    assert [w.number for w in ws] == list(range(1, 91))
    coords = [[None if x == 'null' else float(x) for x in line.split()]
              for line in (ROOT / 'test/fixtures/final_coordinates.txt').read_text().splitlines()]
    assert [[w.latitude, w.longitude] for w in ws] == coords
    assert [w.id for w in ws] == (ROOT / 'test/fixtures/final_ids.txt').read_text().splitlines()
    assert {w.number for w in ws if not w.hard_point} == SOFT_NUMBERS
    assert all(w.activation_radius_m == 3 for w in ws)


@pytest.mark.parametrize('number,change', [(56, {'latitude': None}), (57, {'latitude': 37.0}),
    (31, {'hard_point': True}), (68, {'hard_point': True}), (37, {'waypoint_type': 'mission_exit'}),
    (5, {'number': 4}), (22, {'mission': 'RAMP'}), (31, {'waypoint_type': 'route'})])
def test_invalid_schema(tmp_path, number, change):
    data = yaml.safe_load(CONFIG.read_text())
    data['waypoints'][number-1].update(change)
    file = tmp_path / 'bad.yaml'
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_waypoints(file)


def test_null_skipped_without_reached_event():
    p = progress()
    until(p, 56)
    arrive(p)
    assert p.target.number == 58
    assert [(w.number, event) for w, event in p.events if w.number == 57] == [(57, 'skipped')]
    assert p.waypoints[56].latitude is None and p.waypoints[56].longitude is None


@pytest.mark.parametrize('entry,exit', [(30, 33), (67, 76)])
def test_soft_guides_can_be_missed(entry, exit):
    p = progress()
    until(p, entry)
    arrive(p)
    assert not p.target.hard_point
    w = p.waypoints[exit-1]
    p.gps(w.latitude, w.longitude)
    assert p.target.number == exit+1 and p.state == 'NORMAL_DRIVE'
    assert all((w.number, 'skipped') in [(v.number, e) for v, e in p.events]
               for w in p.waypoints[entry:exit-1])


def test_no_global_hard_skip_and_approach_not_entry():
    p = progress()
    w = p.waypoints[21]
    p.gps(w.latitude, w.longitude)
    assert p.target.number == 1
    until(p, 3)
    arrive(p)
    assert p.state == 'NORMAL_DRIVE'
    arrive(p)
    assert p.state == 'RAMP'


@pytest.mark.parametrize('policy', ['gps', 'status', 'gps_and_status'])
@pytest.mark.parametrize('status_first', [True, False])
def test_completion_policy_transition(policy, status_first):
    p = progress()
    p.waypoints = tuple(replace(w, completion_policy=policy) if w.number == 35 else w for w in p.waypoints)
    until(p, 35)
    arrive(p)
    arrive(p)  # 36, phase MID; next target 37 cannot set APPROACH yet.
    assert p.state == 'INTERSECTION_STRAIGHT_2' and p.phase == 'MID'
    w = p.target
    if status_first:
        p.complete(MissionState.INTERSECTION_STRAIGHT_2, True)
        if policy != 'status':
            assert p.state == 'INTERSECTION_STRAIGHT_2'
        p.gps(w.latitude, w.longitude)
    else:
        p.gps(w.latitude, w.longitude)
        if policy != 'gps':
            assert p.state == 'INTERSECTION_STRAIGHT_2'
        p.complete(MissionState.INTERSECTION_STRAIGHT_2, True)
    assert p.state == 'PERPENDICULAR_PARKING' and p.phase == 'APPROACH'
    for _ in range(10):
        p.gps(w.latitude, w.longitude)
        assert not p.complete(MissionState.INTERSECTION_STRAIGHT_2, True)
    assert p.target.number == 38
    assert sum(w.number == 37 for w, event in p.events) == 1


def test_phase_is_reached_point_and_parking_complete_stop():
    p = progress()
    until(p, 38)
    assert p.phase == 'APPROACH'
    arrive(p)
    assert p.target.number == 39 and p.phase == 'ALIGN'
    assert control_mode_for_mission(p.state, p.phase) == 'LANE'
    arrive(p)
    assert p.phase == 'REVERSE' and control_mode_for_mission(p.state, p.phase) == 'REVERSE'
    arrive(p)
    assert p.phase == 'COMPLETE'
    assert p.state == 'NORMAL_DRIVE'
    assert p.target.number == 41
    assert control_mode_for_mission(
        p.state,
        p.phase,
        normal_drive_mode=p.target.recommended_control_mode) == 'GPS'
    arrive(p)
    assert p.phase is None and control_mode_for_mission(p.state, p.phase) == 'LANE'


def test_finish_terminal_no_waypoint_91():
    p = progress()
    until(p, 90)
    arrive(p)
    assert p.state == 'FINISH' and not p.active
    for _ in range(10):
        assert not p.complete(MissionState.SIGNAL_CAR, True)
        assert arrive(p) is None
        p.start()
    assert p.target.number == 90 and control_mode_for_mission(p.state, p.phase) == 'FINISH'


def test_status_cannot_skip_hard_guide():
    p = progress()
    p.waypoints = tuple(replace(w, completion_policy='status') if w.number == 51 else w for w in p.waypoints)
    until(p, 51)
    arrive(p)
    p.complete(MissionState.INTERSECTION_LEFT, True)
    assert p.target.number == 52 and p.state == 'INTERSECTION_LEFT'
    for _ in range(3):
        arrive(p)
    assert p.target.number == 56 and p.state == 'NORMAL_DRIVE'


@pytest.mark.parametrize('entry,end', [(4,5),(22,24),(30,33),(35,37),(37,40),
                                     (51,55),(67,76),(81,82),(83,86),(89,90)])
@pytest.mark.parametrize('policy', ['gps', 'status', 'gps_and_status'])
def test_all_mission_completion_policies(entry, end, policy):
    p = progress()
    p.waypoints = tuple(replace(w, completion_policy=policy) if w.number == entry else w for w in p.waypoints)
    until(p, entry)
    arrive(p)
    mission = p.state
    until(p, end)
    if policy == 'gps':
        arrive(p)
    elif policy == 'status':
        assert p.complete(getattr(MissionState, mission), True)
    else:
        arrive(p)
        assert p.state == mission
        assert p.complete(getattr(MissionState, mission), True)
    assert p.state != mission
    if end == 90:
        assert p.state == 'FINISH'
    else:
        assert p.target.number == end + 1


def test_invalid_policy(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data['completion_policies']['RAMP'] = 'automatic'
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_waypoints(path)


def test_reached_later_soft_guide_skips_only_preceding_soft():
    p = progress()
    until(p, 68)
    w = p.waypoints[72]
    p.gps(w.latitude, w.longitude)
    assert p.target.number == 74 and p.state == 'CHILD_DUMMY' and p.phase == 'GUIDE'
    assert control_mode_for_mission(p.state, p.phase) == 'OBSTACLE'


def test_school_test_10_route_waypoints_finish():
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'test_cheong.yaml'
    )

    waypoints = load_waypoints(path)
    assert len(waypoints) == 10
    assert [w.number for w in waypoints] == list(range(4, 14))
    assert all(w.waypoint_type == 'route' for w in waypoints)
    assert all(w.recommended_control_mode == 'GPS' for w in waypoints)

    p = WaypointProgress(waypoints)
    p.start()
    assert p.state == 'NORMAL_DRIVE'

    for number in range(4, 14):
        w = p.target
        assert w.number == number
        p.gps(w.latitude, w.longitude, True)

    assert p.state == 'FINISH'
    assert p.target.number == 13
