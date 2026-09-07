"""Offline course validation and mocked ROS callbacks; no hardware or DDS."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from builtin_interfaces.msg import Time
from fma_interfaces.msg import MissionState
from sensor_msgs.msg import NavSatFix
from fma_mission import mission_manager_node as node_module
from fma_mission.waypoint_progress import WaypointProgress, load_waypoints, haversine_m

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'waypoints.yaml'
COORDINATES = [
    (37.28903239, 127.10739052), (37.28888662, 127.10727216),
    (37.28852634, 127.10698829), (37.28878058, 127.10699775),
    (37.28868581, 127.10725953), (37.28850779, 127.10711554),
    (37.28867295, 127.10662554), (37.28858243, 127.10747054),
    (37.28870301, 127.10735800), (37.28883966, 127.10732418),
    (37.28877002, 127.10767208)]


def test_course_exact_coordinates_and_enum():
    waypoints = load_waypoints(CONFIG)
    assert len(waypoints) == 11
    assert [(w.latitude, w.longitude) for w in waypoints] == COORDINATES
    assert [w.id for w in waypoints] == [f'WP{i:02}' for i in range(1, 12)]
    assert all(w.activation_radius_m == 3.0 for w in waypoints)
    assert waypoints[9].missions == ('SIGNAL_CAR', 'LANE_CHANGE')
    assert waypoints[10].type == 'finish'
    names = ('START NORMAL_DRIVE RAMP INTERSECTION_STRAIGHT_1 S_CURVE '
             'INTERSECTION_STRAIGHT_2 PERPENDICULAR_PARKING INTERSECTION_LEFT '
             'CHILD_DUMMY PARALLEL_PARKING INTERSECTION_RIGHT SIGNAL_CAR LANE_CHANGE FINISH')
    assert [getattr(MissionState, n) for n in names.split()] == list(range(14))


def test_haversine():
    assert haversine_m(0, 0, 0, 0) == 0
    assert haversine_m(0, 0, 0, 1) == pytest.approx(111194.9266, abs=.001)
    assert haversine_m(0, 179.999, 0, -179.999) == pytest.approx(222.38985, abs=.001)


@pytest.fixture
def progress():
    p = WaypointProgress(load_waypoints(CONFIG))
    assert p.state == 'START'
    p.start()
    return p


def arrive(p):
    return p.gps(p.target.latitude, p.target.longitude)


def test_only_current_target_and_radius(progress):
    for lat, lon in COORDINATES[1:]:
        progress.gps(lat, lon)
        assert progress.state == 'NORMAL_DRIVE' and progress.target_index == 0
    assert arrive(progress) == 0
    assert progress.state == 'RAMP'
    for lat, lon in COORDINATES:
        assert progress.gps(lat, lon) is None
        assert progress.state == 'RAMP'


def test_inside_outside_and_exact_radius(progress):
    w = progress.target
    distance = haversine_m(w.latitude + .00001, w.longitude, w.latitude, w.longitude)
    progress.waypoints = (replace(w, activation_radius_m=distance - .001),) + progress.waypoints[1:]
    progress.gps(w.latitude + .00001, w.longitude)
    assert progress.state == 'NORMAL_DRIVE'
    progress.waypoints = (replace(w, activation_radius_m=distance),) + progress.waypoints[1:]
    progress.gps(w.latitude + .00001, w.longitude)
    assert progress.state == 'RAMP'


@pytest.mark.parametrize('lat,lon,valid', [(float('nan'), 127., True),
    (37., float('inf'), True), (91., 127., True), (37., -181., True),
    (37.28903239, 127.10739052, False)])
def test_invalid_gps(progress, lat, lon, valid):
    assert progress.gps(lat, lon, valid) is None
    assert progress.state == 'NORMAL_DRIVE' and progress.target_index == 0


def test_completion_and_entire_sequence(progress):
    assert not progress.complete(MissionState.RAMP, True)
    for index in range(9):
        assert progress.target_index == index
        arrive(progress)
        mission = getattr(MissionState, progress.state)
        assert not progress.complete(MissionState.FINISH, True)
        assert not progress.complete(mission, False)
        assert progress.complete(mission, True)
        assert not progress.complete(mission, True)  # duplicate cannot advance twice
        assert progress.target_index == index + 1 and progress.state == 'NORMAL_DRIVE'
    assert progress.target.id == 'WP10'
    arrive(progress)
    assert progress.state == 'SIGNAL_CAR'
    assert progress.complete(MissionState.SIGNAL_CAR, True)
    assert progress.state == 'LANE_CHANGE' and progress.target.id == 'WP10'
    assert not progress.complete(MissionState.SIGNAL_CAR, True)
    assert progress.complete(MissionState.LANE_CHANGE, True)
    assert progress.target.id == 'WP11' and progress.state == 'NORMAL_DRIVE'
    arrive(progress)
    assert progress.state == 'FINISH' and not progress.active
    for mission in range(14):
        assert not progress.complete(mission, True)
        assert progress.gps(*COORDINATES[0]) is None
        progress.start()
        assert progress.state == 'FINISH' and progress.target_index == 10


@pytest.mark.parametrize('old,new', [('3.0', '-1.0'), ('WP01', 'WP02'),
                                    ('RAMP', 'FINISH'), ('type: finish', 'type: mission')])
def test_invalid_yaml(tmp_path, old, new):
    path = tmp_path / 'bad.yaml'
    path.write_text(CONFIG.read_text().replace(old, new, 1))
    with pytest.raises(ValueError):
        load_waypoints(path)


@pytest.fixture
def node(monkeypatch):
    pub, sub, clock = MagicMock(), MagicMock(), MagicMock()
    clock.now.return_value.to_msg.return_value = Time(sec=123)
    monkeypatch.setattr(node_module.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(node_module, 'get_package_share_directory', lambda name: str(CONFIG.parent.parent))
    monkeypatch.setattr(node_module.Node, 'declare_parameter',
                        lambda self, name, value, descriptor: SimpleNamespace(value=value))
    monkeypatch.setattr(node_module.Node, 'create_publisher', pub)
    monkeypatch.setattr(node_module.Node, 'create_subscription', sub)
    monkeypatch.setattr(node_module.Node, 'create_timer', MagicMock())
    monkeypatch.setattr(node_module.Node, 'get_clock', lambda self: clock)
    monkeypatch.setattr(node_module.Node, 'get_logger', lambda self: MagicMock())
    n = node_module.MissionManagerNode()
    assert pub.call_args.args[:2] == (MissionState, '/mission/current')
    assert [c.args[1] for c in sub.call_args_list] == ['/gps/fix', '/mission/status']
    return n


def test_node_startup_gps_status_and_headers(node):
    msg = node.publisher.publish.call_args.args[0]
    assert msg.current_mission == MissionState.START and not msg.active
    node.tick()
    assert node.publisher.publish.call_args.args[0].current_mission == MissionState.NORMAL_DRIVE
    fix = NavSatFix(latitude=COORDINATES[0][0], longitude=COORDINATES[0][1])
    fix.status.status = -1
    node.on_gps(fix)
    assert node.progress.state == 'NORMAL_DRIVE'
    fix.status.status = 0
    node.on_gps(fix)
    msg = node.publisher.publish.call_args.args[0]
    assert msg.current_mission == MissionState.RAMP and msg.active and not msg.completed
    assert msg.header.stamp == Time(sec=123) and msg.header.frame_id == ''
    node.on_status(MissionState(current_mission=MissionState.S_CURVE, completed=True))
    assert node.progress.state == 'RAMP'
    node.on_status(MissionState(current_mission=MissionState.RAMP, completed=True))
    assert node.progress.target.id == 'WP02' and node.progress.state == 'NORMAL_DRIVE'
