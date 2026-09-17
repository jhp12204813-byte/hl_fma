"""Mission-conditioned behavior for the same detected obstacle."""

from dataclasses import dataclass
from enum import Enum
import math

from fma_interfaces.msg import MissionState


class DummyAction(Enum):
    NONE = 'NONE'
    AVOID = 'AVOID'
    STOP = 'STOP'


@dataclass(frozen=True)
class DummyDecision:
    action: DummyAction
    distance_m: float | None = None
    x_m: float | None = None
    y_m: float | None = None
    confidence: float | None = None


def nearest_relevant_obstacle(obstacles, min_confidence=0.0):
    candidates = []

    for obstacle in obstacles:
        if not obstacle.in_path:
            continue
        if not math.isfinite(obstacle.distance_m) or obstacle.distance_m <= 0.0:
            continue
        if not math.isfinite(obstacle.confidence):
            continue
        if obstacle.confidence < min_confidence:
            continue

        candidates.append(obstacle)

    if not candidates:
        return None

    return min(candidates, key=lambda item: item.distance_m)


def decide_dummy_behavior(
    mission,
    active,
    obstacles,
    *,
    avoid_trigger_distance_m,
    stop_trigger_distance_m,
    min_confidence=0.0,
):
    """Return mission behavior without generating a steering trajectory."""

    if not active:
        return DummyDecision(DummyAction.NONE)

    obstacle = nearest_relevant_obstacle(
        obstacles,
        min_confidence=min_confidence,
    )

    if obstacle is None:
        return DummyDecision(DummyAction.NONE)

    common = dict(
        distance_m=float(obstacle.distance_m),
        x_m=float(obstacle.x_m),
        y_m=float(obstacle.y_m),
        confidence=float(obstacle.confidence),
    )

    if mission == MissionState.S_CURVE:
        if obstacle.distance_m <= avoid_trigger_distance_m:
            return DummyDecision(DummyAction.AVOID, **common)

    elif mission == MissionState.CHILD_DUMMY:
        if obstacle.distance_m <= stop_trigger_distance_m:
            return DummyDecision(DummyAction.STOP, **common)

    return DummyDecision(DummyAction.NONE)
