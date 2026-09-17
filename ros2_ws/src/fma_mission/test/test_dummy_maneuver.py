import math

from fma_interfaces.msg import MissionState

from fma_mission.dummy_maneuver import (
    DummyManeuverController,
    ManeuverConfig,
    ManeuverPhase,
    ObstacleSample,
    PassSide,
    choose_pass_side,
)


def obstacle(y_m=0.0):
    return ObstacleSample(
        distance_m=1.5,
        y_m=y_m,
        confidence=0.9,
        in_path=True,
    )


def counts_for(distance_m, config):
    return math.ceil(
        distance_m / config.encoder_m_per_count
    ) + 1


def test_s_curve_avoids_without_stop():
    c = DummyManeuverController()

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=100,
    )

    assert out.phase is ManeuverPhase.AVOID_OUT
    assert out.command_active is True
    assert out.stop is False
    assert out.drive_pwm == 120


def test_child_dummy_stays_stopped_while_blocked():
    c = DummyManeuverController()

    out = c.update(
        mission=MissionState.CHILD_DUMMY,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=100,
    )

    assert out.phase is ManeuverPhase.STOP_HOLD
    assert out.stop is True
    assert out.drive_pwm == 0

    # Even after a long time and with confirmed zero speed,
    # CHILD_DUMMY must never avoid while the path is still blocked.
    out = c.update(
        mission=MissionState.CHILD_DUMMY,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=30.0,
        encoder_count=100,
        vehicle_stopped=True,
    )

    assert out.phase is ManeuverPhase.STOP_HOLD
    assert out.command_active is True
    assert out.stop is True
    assert out.drive_pwm == 0


def test_child_dummy_cleared_during_stop_resumes_without_avoidance():
    c = DummyManeuverController()

    out = c.update(
        mission=MissionState.CHILD_DUMMY,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=100,
    )

    assert out.phase is ManeuverPhase.STOP_HOLD

    # Dummy/cart is removed from the driving path.
    # Require 5 consecutive clear updates.
    for t in (1.0, 1.5, 2.0, 2.5):
        out = c.update(
            mission=MissionState.CHILD_DUMMY,
            mission_active=True,
            obstacle=None,
            now_sec=t,
            encoder_count=100,
            vehicle_stopped=True,
        )
        assert out.phase is ManeuverPhase.STOP_HOLD
        assert out.stop is True

    out = c.update(
        mission=MissionState.CHILD_DUMMY,
        mission_active=True,
        obstacle=None,
        now_sec=3.01,
        encoder_count=100,
        vehicle_stopped=True,
    )

    assert out.phase is ManeuverPhase.IDLE
    assert out.command_active is False

def test_obstacle_left_passes_right():
    cfg = ManeuverConfig()

    assert choose_pass_side(
        0.4,
        cfg,
    ) is PassSide.RIGHT


def test_obstacle_right_passes_left():
    cfg = ManeuverConfig()

    assert choose_pass_side(
        -0.4,
        cfg,
    ) is PassSide.LEFT


def test_center_uses_preferred_right():
    cfg = ManeuverConfig()

    assert choose_pass_side(
        0.0,
        cfg,
    ) is PassSide.RIGHT


def test_avoid_out_changes_after_encoder_distance():
    cfg = ManeuverConfig()
    c = DummyManeuverController(cfg)

    start = 100

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=start,
    )

    assert out.phase is ManeuverPhase.AVOID_OUT
    assert out.drive_pwm == 120

    out_counts = counts_for(
        cfg.avoid_out_distance_m,
        cfg,
    )

    # Before 1.5 m, keep avoidance steering.
    before_counts = math.floor(
        cfg.avoid_out_distance_m
        / cfg.encoder_m_per_count
    )

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=1.0,
        encoder_count=start + before_counts,
    )

    assert out.phase is ManeuverPhase.AVOID_OUT

    # After 1.5 m, straighten for PASS_STRAIGHT.
    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=2.0,
        encoder_count=start + out_counts,
    )

    assert out.phase is ManeuverPhase.PASS_STRAIGHT
    assert out.steering_rad == 0.0



def test_complete_avoidance_and_rearm_after_clear_frames():
    cfg = ManeuverConfig()
    c = DummyManeuverController(cfg)

    start = 1000

    c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=start,
    )

    out_counts = counts_for(
        cfg.avoid_out_distance_m,
        cfg,
    )

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=1.0,
        encoder_count=start + out_counts,
    )

    assert out.phase is ManeuverPhase.PASS_STRAIGHT

    pass_counts = counts_for(
        cfg.pass_straight_distance_m,
        cfg,
    )

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=2.0,
        encoder_count=start + out_counts + pass_counts,
    )

    assert out.phase is ManeuverPhase.AVOID_RETURN

    return_counts = counts_for(
        cfg.avoid_return_distance_m,
        cfg,
    )

    final_encoder = (
        start
        + out_counts
        + pass_counts
        + return_counts
    )

    out = c.update(
        mission=MissionState.S_CURVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=3.0,
        encoder_count=final_encoder,
    )

    assert out.phase is ManeuverPhase.WAIT_CLEAR
    assert out.command_active is False

    for i in range(cfg.clear_frames):
        out = c.update(
            mission=MissionState.S_CURVE,
            mission_active=True,
            obstacle=None,
            now_sec=1.6 + i * 0.1,
            encoder_count=final_encoder,
        )

    assert out.phase is ManeuverPhase.IDLE
    assert out.command_active is False


def test_wrong_mission_never_commands_vehicle():
    c = DummyManeuverController()

    out = c.update(
        mission=MissionState.NORMAL_DRIVE,
        mission_active=True,
        obstacle=obstacle(),
        now_sec=0.0,
        encoder_count=0,
    )

    assert out.phase is ManeuverPhase.IDLE
    assert out.command_active is False
