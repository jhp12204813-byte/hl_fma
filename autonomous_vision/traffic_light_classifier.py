"""Dynamic-bbox tracking and relative-ROI HSV signal classification."""

from dataclasses import dataclass, field
from typing import Dict, Optional

import cv2
import numpy as np

try:
    from . import config
    from .bbox_tracker import BBoxTracker  # Compatibility re-export.
    from .stability_filter import StableStateFilter
    from .utils import BBox, Detection, clamp_bbox
except ImportError:
    import config
    from bbox_tracker import BBoxTracker
    from stability_filter import StableStateFilter
    from utils import BBox, Detection, clamp_bbox


class SignalStateStabilizer(StableStateFilter):
    """Backward-compatible traffic-signal name for the generic filter."""

    def __init__(self, confirm_frames=config.TRAFFIC_CONFIRM_FRAMES,
                 lost_frames=config.TRAFFIC_LOST_FRAMES):
        super().__init__(confirm_frames, lost_frames)


@dataclass
class LampMeasurement:
    on: bool = False
    pixel_ratio: float = 0.0
    contour_area: float = 0.0
    contour_area_ratio: float = 0.0


@dataclass
class ClassificationResult:
    detected: bool
    bbox: Optional[tuple]
    raw_state: str = "UNKNOWN"
    stable_state: str = "UNKNOWN"
    confidence: float = 0.0
    lamps: Dict[str, LampMeasurement] = field(default_factory=dict)
    debug_images: Dict[str, np.ndarray] = field(default_factory=dict)


class TrafficLightClassifier:
    """Classify lamps inside a YOLO-provided dynamic traffic-light bbox."""

    def __init__(self):
        self.stabilizer = SignalStateStabilizer()
        self._validate_config()

    @staticmethod
    def _validate_config():
        for name, roi in config.LAMP_ROIS.items():
            x1, y1, x2, y2 = roi
            if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
                raise ValueError(f"{name}_LAMP_ROI must be normalized xyxy: {roi}")
        if config.MORPH_KERNEL_SIZE < 1:
            raise ValueError("MORPH_KERNEL_SIZE must be positive")

    @staticmethod
    def _color_mask(hsv, lamp_name):
        if lamp_name == "RED":
            first = cv2.inRange(hsv, config.RED_LOW_1, config.RED_HIGH_1)
            second = cv2.inRange(hsv, config.RED_LOW_2, config.RED_HIGH_2)
            return cv2.bitwise_or(first, second)
        if lamp_name == "YELLOW":
            return cv2.inRange(hsv, config.YELLOW_LOW, config.YELLOW_HIGH)
        return cv2.inRange(hsv, config.GREEN_LOW, config.GREEN_HIGH)

    @staticmethod
    def _raw_state(lamps):
        flags = {name: value.on for name, value in lamps.items()}
        conditions = {
            "RED": flags.get("RED", False),
            "YELLOW": flags.get("YELLOW", False),
            "GREEN_AND_LEFT": flags.get("GREEN_LEFT", False) and flags.get("GREEN", False),
            "GREEN_LEFT": flags.get("GREEN_LEFT", False),
            "GREEN": flags.get("GREEN", False),
        }
        return next((state for state in config.STATE_PRIORITY if conditions.get(state)), "UNKNOWN")

    def classify(self, frame, bbox: Optional[BBox], confidence=0.0, debug=False):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        if bbox is None:
            stable = self.stabilizer.update("UNKNOWN")
            return ClassificationResult(False, None, stable_state=stable)

        bounds = clamp_bbox(bbox, frame.shape[1], frame.shape[0])
        if bounds is None:
            stable = self.stabilizer.update("UNKNOWN")
            return ClassificationResult(False, None, stable_state=stable)
        x1, y1, x2, y2 = bounds
        crop = frame[y1:y2, x1:x2]
        crop_h, crop_w = crop.shape[:2]
        if (crop_w < config.MIN_TRAFFIC_LIGHT_WIDTH or
                crop_h < config.MIN_TRAFFIC_LIGHT_HEIGHT):
            stable = self.stabilizer.update("UNKNOWN")
            return ClassificationResult(True, bounds, stable_state=stable,
                                        confidence=float(confidence))
        kernel = np.ones((config.MORPH_KERNEL_SIZE, config.MORPH_KERNEL_SIZE), np.uint8)
        lamps = {}
        debug_images = {"traffic_crop": crop.copy()} if debug else {}

        for name, relative in config.LAMP_ROIS.items():
            rx1, ry1, rx2, ry2 = relative
            lx1, ly1 = int(rx1 * crop_w), int(ry1 * crop_h)
            lx2, ly2 = max(lx1 + 1, int(rx2 * crop_w)), max(ly1 + 1, int(ry2 * crop_h))
            lamp_crop = crop[ly1:ly2, lx1:lx2]
            hsv = cv2.cvtColor(lamp_crop, cv2.COLOR_BGR2HSV)
            mask = self._color_mask(hsv, name)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel,
                                    iterations=config.MORPH_OPEN_ITERATIONS)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel,
                                    iterations=config.MORPH_CLOSE_ITERATIONS)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest = max((cv2.contourArea(item) for item in contours), default=0.0)
            area = float(mask.size)
            pixel_ratio = float(cv2.countNonZero(mask)) / max(area, 1.0)
            contour_ratio = float(largest) / max(area, 1.0)
            is_on = (pixel_ratio >= config.MIN_COLOR_RATIO and
                     largest >= config.MIN_CONTOUR_AREA and
                     contour_ratio >= config.MIN_CONTOUR_AREA_RATIO)
            lamps[name] = LampMeasurement(is_on, pixel_ratio, float(largest), contour_ratio)

            if debug:
                cv2.rectangle(debug_images["traffic_crop"], (lx1, ly1), (lx2, ly2),
                              (0, 255, 0) if is_on else (120, 120, 120), 1)
                cv2.putText(debug_images["traffic_crop"], name, (lx1 + 2, max(12, ly1 + 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, .32, (255, 255, 255), 1)
                canvas = np.zeros((crop_h, crop_w), dtype=np.uint8)
                canvas[ly1:ly2, lx1:lx2] = mask
                debug_images[f"{name.lower()}_mask"] = canvas

        raw = self._raw_state(lamps)
        stable = self.stabilizer.update(raw)
        return ClassificationResult(True, bounds, raw, stable, float(confidence), lamps, debug_images)
