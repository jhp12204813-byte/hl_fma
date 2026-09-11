"""Dynamic message-board crop and conservative LED active/idle detection."""

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

try:
    from . import config
    from .utils import BBox, clamp_bbox
except ImportError:
    import config
    from utils import BBox, clamp_bbox


@dataclass
class MessageBoardResult:
    detected: bool
    bbox: Optional[tuple]
    confidence: float = 0.0
    active: bool = False
    recognized_text: str = ""
    recognized_class: str = ""
    recognition_confidence: float = 0.0
    mean_brightness: float = 0.0
    active_pixel_ratio: float = 0.0
    saturated_pixel_ratio: float = 0.0
    debug_images: dict = field(default_factory=dict)


class MessageBoardReader:
    """Initial reader: detect ON/OFF only; OCR/classification is intentionally absent."""

    def read(self, frame, bbox: Optional[BBox], confidence=0.0, debug=False):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be a non-empty BGR image")
        if bbox is None:
            return MessageBoardResult(False, None)
        bounds = clamp_bbox(bbox, frame.shape[1], frame.shape[0])
        if bounds is None:
            return MessageBoardResult(False, None)
        x1, y1, x2, y2 = bounds
        crop = frame[y1:y2, x1:x2]
        if (crop.shape[1] < config.MIN_MESSAGE_BOARD_WIDTH or
                crop.shape[0] < config.MIN_MESSAGE_BOARD_HEIGHT):
            return MessageBoardResult(True, bounds, float(confidence))

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        saturation, value = hsv[:, :, 1], hsv[:, :, 2]
        bright = (value >= config.MESSAGE_BRIGHT_PIXEL_VALUE).astype(np.uint8) * 255
        saturated = ((value >= config.MESSAGE_BRIGHT_PIXEL_VALUE) &
                     (saturation >= config.MESSAGE_SATURATION_MIN)).astype(np.uint8) * 255
        kernel = np.ones((config.MESSAGE_MORPH_KERNEL_SIZE,
                          config.MESSAGE_MORPH_KERNEL_SIZE), np.uint8)
        active_mask = cv2.morphologyEx(bright, cv2.MORPH_OPEN, kernel)
        active_mask = cv2.morphologyEx(active_mask, cv2.MORPH_CLOSE, kernel)
        saturated = cv2.morphologyEx(saturated, cv2.MORPH_OPEN, kernel)
        active_ratio = cv2.countNonZero(active_mask) / float(active_mask.size)
        saturated_ratio = cv2.countNonZero(saturated) / float(saturated.size)
        mean_brightness = float(value.mean())
        active = (active_ratio >= config.MESSAGE_MIN_ACTIVE_PIXEL_RATIO and
                  (saturated_ratio >= config.MESSAGE_MIN_SATURATED_PIXEL_RATIO or
                   mean_brightness >= config.MESSAGE_MIN_MEAN_BRIGHTNESS))
        debug_images = {"message_board_crop": crop, "message_board_active_mask": active_mask}
        if not debug:
            debug_images = {}
        return MessageBoardResult(True, bounds, float(confidence), active,
                                  mean_brightness=mean_brightness,
                                  active_pixel_ratio=float(active_ratio),
                                  saturated_pixel_ratio=float(saturated_ratio),
                                  debug_images=debug_images)
