"""Independent bounding-box smoothing and three-panel association."""

from collections import deque
from dataclasses import dataclass
from itertools import permutations
from typing import Optional

import numpy as np

try:
    from . import config
    from .utils import Detection
except ImportError:
    import config
    from utils import Detection


class BBoxTracker:
    """Reject one-frame jumps with a median, then smooth coordinates by EMA."""

    def __init__(self, alpha=config.BBOX_SMOOTH_ALPHA,
                 history_size=config.BBOX_HISTORY_SIZE,
                 max_missing=config.MAX_MISSING_FRAMES):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("BBOX_SMOOTH_ALPHA must be in (0, 1]")
        self.alpha = float(alpha)
        self.history = deque(maxlen=max(1, int(history_size)))
        self.max_missing = max(0, int(max_missing))
        self.smoothed = None
        self.missing_frames = 0
        self.last_confidence = 0.0

    def update(self, detection: Optional[Detection]):
        if detection is None:
            self.missing_frames += 1
            if self.missing_frames > self.max_missing:
                self.reset()
            return self.current()
        raw = np.asarray(detection.bbox, dtype=np.float32)
        self.history.append(raw)
        robust_box = np.median(np.stack(self.history), axis=0)
        self.smoothed = (robust_box if self.smoothed is None else
                         self.alpha * robust_box + (1.0 - self.alpha) * self.smoothed)
        self.missing_frames = 0
        self.last_confidence = detection.confidence
        return self.current()

    def current(self):
        return None if self.smoothed is None else tuple(float(v) for v in self.smoothed)

    def reset(self):
        self.history.clear()
        self.smoothed = None
        self.missing_frames = 0
        self.last_confidence = 0.0


@dataclass(frozen=True)
class TrackedPanel:
    """A stable left-to-right slot assignment for one sign-panel box."""

    slot: int
    bbox: Optional[tuple]
    confidence: float
    observed: bool


class SignPanelTracker:
    """Associate up to three independently detected panels with fixed slots.

    Three simultaneous detections establish left/center/right identity. With
    one or two detections, minimum center-distance association preserves the
    existing identities. Before the first complete 3-panel observation, a
    partial set remains unassigned rather than guessing an unsafe lane mapping.
    """

    def __init__(self, max_distance=config.PANEL_ASSOCIATION_MAX_DISTANCE):
        self.trackers = [BBoxTracker(), BBoxTracker(), BBoxTracker()]
        self.initialized = False
        self.max_distance = float(max_distance)

    @staticmethod
    def _center_x(bbox):
        return (bbox[0] + bbox[2]) / 2.0

    def update(self, detections):
        detections = sorted(detections, key=lambda item: item.center_x)
        observed = [False, False, False]
        assignments = {}
        if len(detections) >= 3:
            chosen = sorted(sorted(detections, key=lambda item: item.confidence,
                                   reverse=True)[:3], key=lambda item: item.center_x)
            assignments = dict(enumerate(chosen))
            self.initialized = True
        elif self.initialized and detections:
            active = [index for index, tracker in enumerate(self.trackers)
                      if tracker.current() is not None]
            if len(active) >= len(detections):
                best = None
                for slot_order in permutations(active, len(detections)):
                    cost = sum(abs(detection.center_x -
                                   self._center_x(self.trackers[slot].current()))
                               for detection, slot in zip(detections, slot_order))
                    if best is None or cost < best[0]:
                        best = (cost, slot_order)
                for detection, slot in zip(detections, best[1]):
                    old = self.trackers[slot].current()
                    old_width = max(old[2] - old[0], 1.0)
                    if abs(detection.center_x - self._center_x(old)) <= self.max_distance * old_width:
                        assignments[slot] = detection

        for slot, tracker in enumerate(self.trackers):
            detection = assignments.get(slot)
            tracker.update(detection)
            observed[slot] = detection is not None
        if not any(tracker.current() is not None for tracker in self.trackers):
            self.initialized = False
        return [TrackedPanel(index + 1, tracker.current(), tracker.last_confidence,
                             observed[index])
                for index, tracker in enumerate(self.trackers)]
