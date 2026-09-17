"""Temporal voting for the three-slot signal-car board."""

from collections import Counter, deque
from dataclasses import dataclass

from fma_interfaces.msg import LaneSignal


WAIT = 'WAIT'
KEEP = 'KEEP'
LEFT = 'LEFT'


@dataclass(frozen=True)
class SignalCarConfig:
    window_sec: float = 2.0
    min_valid_samples: int = 4
    majority_ratio: float = 0.75
    min_confidence: float = 0.60


class SignalCarVoter:

    def __init__(self, config=None):
        self.config = config or SignalCarConfig()
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def _prune(self, now):
        cutoff = now - self.config.window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def classify(
        self,
        left_state,
        center_state,
        right_state,
        left_confidence,
        center_confidence,
        right_confidence,
    ):
        states = (
            left_state,
            center_state,
            right_state,
        )

        confidences = (
            left_confidence,
            center_confidence,
            right_confidence,
        )

        # Blink-off / missing observation.
        if LaneSignal.UNKNOWN in states:
            return WAIT

        if min(confidences) < self.config.min_confidence:
            return WAIT

        # X / green / X
        if states == (
            LaneSignal.CLOSED,
            LaneSignal.OPEN,
            LaneSignal.CLOSED,
        ):
            return KEEP

        # green / X / X
        if states == (
            LaneSignal.OPEN,
            LaneSignal.CLOSED,
            LaneSignal.CLOSED,
        ):
            return LEFT

        return WAIT

    def update(
        self,
        now,
        left_state,
        center_state,
        right_state,
        left_confidence,
        center_confidence,
        right_confidence,
    ):
        self._prune(now)

        decision = self.classify(
            left_state,
            center_state,
            right_state,
            left_confidence,
            center_confidence,
            right_confidence,
        )

        # UNKNOWN/blink-off frames are ignored instead of voting against.
        if decision != WAIT:
            self.samples.append((now, decision))

        self._prune(now)

        if len(self.samples) < self.config.min_valid_samples:
            return WAIT

        counts = Counter(decision for _, decision in self.samples)
        winner, count = counts.most_common(1)[0]

        if count / len(self.samples) < self.config.majority_ratio:
            return WAIT

        return winner
