"""Boundary-confirmed LEFT lane-change logic."""

from dataclasses import dataclass
import math


IDLE = 'IDLE'
SHIFT_LEFT = 'SHIFT_LEFT'
COUNTER_STEER = 'COUNTER_STEER'
REACQUIRE = 'REACQUIRE'
DONE = 'DONE'
FAILED = 'FAILED'

KEEP = 'KEEP'
LEFT = 'LEFT'


@dataclass(frozen=True)
class LaneChangeConfig:
    forward_speed_mps: float = 0.12
    shift_steering_rad: float = 0.12
    counter_steering_rad: float = 0.10

    min_shift_sec: float = 0.40
    max_shift_sec: float = 4.00
    counter_duration_sec: float = 0.80
    max_maneuver_sec: float = 8.00


@dataclass(frozen=True)
class CrossingConfig:
    arm_distance_m: float = 0.90
    confirm_distance_m: float = 0.20
    min_confidence: float = 0.55
    confirm_samples: int = 2


def resolve_lane_width(
    expected_width_m,
    selected_width_m,
    *,
    minimum_m=3.0,
    maximum_m=3.5,
    fallback_m=3.25,
):
    """Learned PAIR -> current PAIR -> nominal midpoint fallback."""

    def usable(value):
        return (
            value is not None
            and math.isfinite(float(value))
            and minimum_m <= float(value) <= maximum_m
        )

    if usable(expected_width_m):
        return float(expected_width_m), 'LEARNED_PAIR'

    if usable(selected_width_m):
        return float(selected_width_m), 'CURRENT_PAIR'

    return float(fallback_m), 'FALLBACK'


def separator_before_crossing(
    *,
    left_valid,
    left_x,
    left_confidence,
    right_valid,
    right_x,
    right_confidence,
    lane_width,
):
    """Separator is current lane's LEFT boundary."""

    if left_valid and math.isfinite(float(left_x)):
        return float(left_x), float(left_confidence), 'LEFT_OBSERVED'

    if right_valid and math.isfinite(float(right_x)):
        return (
            float(right_x) - lane_width,
            float(right_confidence),
            'LEFT_INFERRED_FROM_RIGHT',
        )

    return None, 0.0, 'NONE'


def separator_after_crossing(
    *,
    left_valid,
    left_x,
    left_confidence,
    right_valid,
    right_x,
    right_confidence,
    lane_width,
):
    """After LEFT change, separator is new lane's RIGHT boundary."""

    if right_valid and math.isfinite(float(right_x)):
        return float(right_x), float(right_confidence), 'RIGHT_OBSERVED'

    if left_valid and math.isfinite(float(left_x)):
        return (
            float(left_x) + lane_width,
            float(left_confidence),
            'RIGHT_INFERRED_FROM_LEFT',
        )

    return None, 0.0, 'NONE'


class LaneBoundaryCrossing:

    def __init__(self, config=None):
        self.config = config or CrossingConfig()
        self.reset()

    def reset(self):
        self.armed = False
        self.crossed = False
        self.right_streak = 0

    def update(self, *, valid, side, x_m, confidence):
        if self.crossed:
            return True

        if not valid:
            return False

        if not (
            math.isfinite(x_m)
            and math.isfinite(confidence)
            and confidence >= self.config.min_confidence
        ):
            return False

        if not self.armed:
            if (
                side == 'LEFT'
                and -self.config.arm_distance_m <= x_m < 0.0
            ):
                self.armed = True

            return False

        if (
            side == 'RIGHT'
            and self.config.confirm_distance_m
            <= x_m
            <= self.config.arm_distance_m
        ):
            self.right_streak += 1
        elif side == 'LEFT' and x_m < 0.0:
            self.right_streak = 0

        if self.right_streak >= self.config.confirm_samples:
            self.crossed = True

        return self.crossed


# 기존 이름을 사용한 코드가 있어도 깨지지 않게 유지.
DashedBoundaryCrossing = LaneBoundaryCrossing


class LaneChangeManeuver:

    def __init__(self, config=None):
        self.config = config or LaneChangeConfig()
        self.reset()

    def reset(self):
        self.request = None
        self.phase = IDLE
        self.started_at = None
        self.phase_started_at = None

    def _enter(self, phase, now):
        self.phase = phase
        self.phase_started_at = now

    def start(self, request, now):
        self.reset()
        self.request = request
        self.started_at = now

        if request == KEEP:
            self._enter(DONE, now)
        elif request == LEFT:
            self._enter(SHIFT_LEFT, now)
        else:
            self._enter(FAILED, now)

        return self.phase

    def update(
        self,
        now,
        *,
        dashed_crossed=False,
        lane_reacquired=False,
    ):
        if self.phase in (IDLE, DONE, FAILED):
            return self.phase

        if now - self.started_at >= self.config.max_maneuver_sec:
            self._enter(FAILED, now)
            return self.phase

        elapsed = now - self.phase_started_at

        if self.phase == SHIFT_LEFT:
            if (
                dashed_crossed
                and elapsed >= self.config.min_shift_sec
            ):
                self._enter(COUNTER_STEER, now)

            elif elapsed >= self.config.max_shift_sec:
                self._enter(FAILED, now)

        elif self.phase == COUNTER_STEER:
            if elapsed >= self.config.counter_duration_sec:
                self._enter(REACQUIRE, now)

        elif self.phase == REACQUIRE and lane_reacquired:
            self._enter(DONE, now)

        return self.phase

    def command(self):
        if self.phase == SHIFT_LEFT:
            return (
                self.config.forward_speed_mps,
                abs(self.config.shift_steering_rad),
                False,
            )

        if self.phase == COUNTER_STEER:
            return (
                self.config.forward_speed_mps,
                -abs(self.config.counter_steering_rad),
                False,
            )

        if self.phase == FAILED:
            return (0.0, 0.0, True)

        return None
