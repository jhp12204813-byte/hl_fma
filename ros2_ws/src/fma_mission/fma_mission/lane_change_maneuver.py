"""Timed left lane-change maneuver state machine."""

from dataclasses import dataclass


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
    shift_steering_rad: float = 0.10
    counter_steering_rad: float = 0.08
    shift_duration_sec: float = 1.0
    counter_duration_sec: float = 0.8
    max_maneuver_sec: float = 6.0


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

    def update(self, now, lane_reacquired=False):
        if self.phase in (IDLE, DONE, FAILED):
            return self.phase

        if now - self.started_at >= self.config.max_maneuver_sec:
            self._enter(FAILED, now)
            return self.phase

        elapsed = now - self.phase_started_at

        if (
            self.phase == SHIFT_LEFT
            and elapsed >= self.config.shift_duration_sec
        ):
            self._enter(COUNTER_STEER, now)

        elif (
            self.phase == COUNTER_STEER
            and elapsed >= self.config.counter_duration_sec
        ):
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

        # REACQUIRE deliberately publishes nothing so /cmd/lane
        # can take over after the arbiter mission timeout.
        return None
