from fma_interfaces.msg import MissionState, Obstacle

from fma_mission.dummy_behavior import (
    DummyAction,
    decide_dummy_behavior,
)


def obstacle(distance=2.0, y=0.2, confidence=0.9, in_path=True):
    msg = Obstacle()
    msg.x_m = distance
    msg.y_m = y
    msg.distance_m = distance
    msg.bearing_rad = 0.0
    msg.in_path = in_path
    msg.confidence = confidence
    return msg


def decide(mission, obstacles):
    return decide_dummy_behavior(
        mission,
        True,
        obstacles,
        avoid_trigger_distance_m=3.0,
        stop_trigger_distance_m=3.0,
        min_confidence=0.5,
    )


def test_same_obstacle_avoids_in_s_curve():
    result = decide(MissionState.S_CURVE, [obstacle()])
    assert result.action is DummyAction.AVOID
    assert result.distance_m == 2.0


def test_same_obstacle_stops_in_child_dummy():
    result = decide(MissionState.CHILD_DUMMY, [obstacle()])
    assert result.action is DummyAction.STOP
    assert result.distance_m == 2.0


def test_other_mission_does_nothing():
    result = decide(MissionState.NORMAL_DRIVE, [obstacle()])
    assert result.action is DummyAction.NONE


def test_out_of_path_obstacle_ignored():
    result = decide(MissionState.S_CURVE, [obstacle(in_path=False)])
    assert result.action is DummyAction.NONE


def test_low_confidence_obstacle_ignored():
    result = decide(
        MissionState.CHILD_DUMMY,
        [obstacle(confidence=0.2)],
    )
    assert result.action is DummyAction.NONE


def test_inactive_mission_does_nothing():
    result = decide_dummy_behavior(
        MissionState.S_CURVE,
        False,
        [obstacle()],
        avoid_trigger_distance_m=3.0,
        stop_trigger_distance_m=3.0,
        min_confidence=0.5,
    )
    assert result.action is DummyAction.NONE


def test_nearest_relevant_obstacle_used():
    result = decide(
        MissionState.S_CURVE,
        [obstacle(2.5), obstacle(1.2)],
    )
    assert result.action is DummyAction.AVOID
    assert result.distance_m == 1.2
