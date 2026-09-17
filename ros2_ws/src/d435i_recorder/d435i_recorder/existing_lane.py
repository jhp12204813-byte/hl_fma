"""Reuse the workspace lane algorithm directly, without its ROS node."""
from dataclasses import asdict
import hashlib
from pathlib import Path


class ExistingLane:
    def __init__(self, config=None):
        from fma_perception import lane_detection
        self.module = lane_detection
        self.config = lane_detection.LaneConfig(**(config or {}))
        source = Path(lane_detection.__file__).resolve()
        self.provenance = dict(backend='fma_perception.lane_detection.detect_lane',
                               lane_color='yellow_only',
                               source=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                               config=asdict(self.config))

    def detect(self, frame):
        estimate = self.module.detect_lane(frame, self.config, yellow_only=True)
        calibrated = self.config.lane_width_m > 0 and self.config.meters_per_pixel_y > 0
        result = dict(visual_detected=bool(estimate.visual_detected),
                      detected=bool(estimate.detected), confidence=estimate.confidence,
                      metric_calibrated=calibrated,
                      lateral_error_m=estimate.lateral_error_m if estimate.detected else None,
                      heading_error_rad=estimate.heading_error_rad if estimate.detected else None)
        return result, estimate.debug
