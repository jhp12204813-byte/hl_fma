from fma_mission.lane_change_maneuver import (
    COUNTER_STEER,
    DONE,
    FAILED,
    KEEP,
    LEFT,
    REACQUIRE,
    SHIFT_LEFT,
    CrossingConfig,
    DashedBoundaryCrossing,
    LaneChangeConfig,
    LaneChangeManeuver,
)


def maneuver_config():
    return LaneChangeConfig(
        forward_speed_mps=0.12,
        shift_steering_rad=0.12,
        counter_steering_rad=0.10,
        min_shift_sec=0.4,
        max_shift_sec=4.0,
        counter_duration_sec=0.8,
        max_maneuver_sec=8.0,
    )


def test_keep_finishes_immediately():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    assert m.start(KEEP, 0.0) == DONE
    assert m.command() is None


def test_left_does_not_counter_before_crossing():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    assert m.start(LEFT, 0.0) == SHIFT_LEFT

    m.update(
        1.0,
        dashed_crossed=False,
    )

    assert m.phase == SHIFT_LEFT


def test_crossing_starts_counter_steer():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    m.start(LEFT, 0.0)

    m.update(
        0.5,
        dashed_crossed=True,
    )

    assert m.phase == COUNTER_STEER


def test_shift_timeout_fails_instead_of_blind_change():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    m.start(LEFT, 0.0)

    m.update(
        4.0,
        dashed_crossed=False,
    )

    assert m.phase == FAILED


def test_counter_then_reacquire_then_done():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    m.start(LEFT, 0.0)

    m.update(
        0.5,
        dashed_crossed=True,
    )

    assert m.phase == COUNTER_STEER

    m.update(
        1.3,
        dashed_crossed=True,
    )

    assert m.phase == REACQUIRE
    assert m.command() is None

    m.update(
        1.5,
        dashed_crossed=True,
        lane_reacquired=True,
    )

    assert m.phase == DONE


def test_shift_left_command_positive():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    m.start(LEFT, 0.0)

    speed, steering, emergency = (
        m.command()
    )

    assert speed > 0
    assert steering > 0
    assert emergency is False


def test_counter_command_negative():
    m = LaneChangeManeuver(
        maneuver_config()
    )

    m.start(LEFT, 0.0)

    m.update(
        0.5,
        dashed_crossed=True,
    )

    speed, steering, emergency = (
        m.command()
    )

    assert speed > 0
    assert steering < 0
    assert emergency is False


def test_crossing_requires_left_then_right():
    crossing = DashedBoundaryCrossing(
        CrossingConfig(
            arm_distance_m=0.9,
            confirm_distance_m=0.2,
            min_confidence=0.55,
            confirm_samples=2,
        )
    )

    assert not crossing.update(
        valid=True,
        side='LEFT',
        x_m=-0.70,
        confidence=0.9,
    )

    assert not crossing.update(
        valid=True,
        side='LEFT',
        x_m=-0.20,
        confidence=0.9,
    )

    assert not crossing.update(
        valid=True,
        side='RIGHT',
        x_m=0.30,
        confidence=0.9,
    )

    assert crossing.update(
        valid=True,
        side='RIGHT',
        x_m=0.35,
        confidence=0.9,
    )


def test_far_right_boundary_does_not_fake_crossing():
    crossing = DashedBoundaryCrossing(
        CrossingConfig()
    )

    crossing.update(
        valid=True,
        side='LEFT',
        x_m=-0.5,
        confidence=0.9,
    )

    for _ in range(5):
        assert not crossing.update(
            valid=True,
            side='RIGHT',
            x_m=1.7,
            confidence=0.9,
        )


def test_invalid_or_low_confidence_does_not_cross():
    crossing = DashedBoundaryCrossing(
        CrossingConfig()
    )

    crossing.update(
        valid=True,
        side='LEFT',
        x_m=-0.4,
        confidence=0.9,
    )

    for _ in range(5):
        assert not crossing.update(
            valid=True,
            side='RIGHT',
            x_m=0.3,
            confidence=0.2,
        )


def test_width_prefers_learned_pair():
    from fma_mission.lane_change_maneuver import resolve_lane_width

    width, source = resolve_lane_width(3.18, 3.30)

    assert width == 3.18
    assert source == 'LEARNED_PAIR'


def test_width_uses_current_pair_if_no_learned_width():
    from fma_mission.lane_change_maneuver import resolve_lane_width

    width, source = resolve_lane_width(None, 3.32)

    assert width == 3.32
    assert source == 'CURRENT_PAIR'


def test_width_falls_back_to_325_if_measurement_is_missing():
    from fma_mission.lane_change_maneuver import resolve_lane_width

    width, source = resolve_lane_width(None, None)

    assert width == 3.25
    assert source == 'FALLBACK'


def test_width_falls_back_if_pair_width_is_implausible():
    from fma_mission.lane_change_maneuver import resolve_lane_width

    width, source = resolve_lane_width(4.1, 2.7)

    assert width == 3.25
    assert source == 'FALLBACK'


def test_separator_can_be_inferred_from_only_right_boundary():
    from fma_mission.lane_change_maneuver import separator_before_crossing

    x, confidence, source = separator_before_crossing(
        left_valid=False,
        left_x=float('nan'),
        left_confidence=0.0,
        right_valid=True,
        right_x=2.9,
        right_confidence=0.9,
        lane_width=3.25,
    )

    assert abs(x - (-0.35)) < 1e-9
    assert confidence == 0.9
    assert source == 'LEFT_INFERRED_FROM_RIGHT'


def test_separator_can_be_inferred_from_only_left_after_crossing():
    from fma_mission.lane_change_maneuver import separator_after_crossing

    x, confidence, source = separator_after_crossing(
        left_valid=True,
        left_x=-3.0,
        left_confidence=0.9,
        right_valid=False,
        right_x=float('nan'),
        right_confidence=0.0,
        lane_width=3.25,
    )

    assert abs(x - 0.25) < 1e-9
    assert confidence == 0.9
    assert source == 'RIGHT_INFERRED_FROM_LEFT'
