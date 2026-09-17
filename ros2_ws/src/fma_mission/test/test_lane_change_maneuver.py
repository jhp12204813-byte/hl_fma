from fma_mission.lane_change_maneuver import (
    COUNTER_STEER,
    DONE,
    FAILED,
    KEEP,
    LEFT,
    REACQUIRE,
    SHIFT_LEFT,
    LaneChangeConfig,
    LaneChangeManeuver,
)


def config():
    return LaneChangeConfig(
        forward_speed_mps=0.12,
        shift_steering_rad=0.10,
        counter_steering_rad=0.08,
        shift_duration_sec=1.0,
        counter_duration_sec=0.8,
        max_maneuver_sec=6.0,
    )


def test_keep_finishes_without_maneuver():
    maneuver = LaneChangeManeuver(config())

    assert maneuver.start(KEEP, 0.0) == DONE
    assert maneuver.command() is None


def test_left_phase_order():
    maneuver = LaneChangeManeuver(config())

    assert maneuver.start(LEFT, 0.0) == SHIFT_LEFT

    maneuver.update(1.0)
    assert maneuver.phase == COUNTER_STEER

    maneuver.update(1.8)
    assert maneuver.phase == REACQUIRE

    maneuver.update(2.0, lane_reacquired=True)
    assert maneuver.phase == DONE


def test_shift_left_steering_is_positive():
    maneuver = LaneChangeManeuver(config())
    maneuver.start(LEFT, 0.0)

    speed, steering, emergency = maneuver.command()

    assert speed > 0.0
    assert steering > 0.0
    assert emergency is False


def test_counter_steer_is_negative():
    maneuver = LaneChangeManeuver(config())
    maneuver.start(LEFT, 0.0)
    maneuver.update(1.0)

    speed, steering, emergency = maneuver.command()

    assert speed > 0.0
    assert steering < 0.0
    assert emergency is False


def test_reacquire_has_no_mission_command():
    maneuver = LaneChangeManeuver(config())
    maneuver.start(LEFT, 0.0)
    maneuver.update(1.0)
    maneuver.update(1.8)

    assert maneuver.phase == REACQUIRE
    assert maneuver.command() is None


def test_timeout_fails_safe():
    maneuver = LaneChangeManeuver(config())
    maneuver.start(LEFT, 0.0)

    maneuver.update(6.0)

    assert maneuver.phase == FAILED

    speed, steering, emergency = maneuver.command()

    assert speed == 0.0
    assert steering == 0.0
    assert emergency is True
