"""Shared data structures and small image helpers."""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2

BBox = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Detection:
    """One YOLO result in absolute ``xyxy`` image coordinates."""

    class_id: int
    class_name: str
    confidence: float
    bbox: BBox

    @property
    def x1(self):
        return self.bbox[0]

    @property
    def y1(self):
        return self.bbox[1]

    @property
    def x2(self):
        return self.bbox[2]

    @property
    def y2(self):
        return self.bbox[3]

    @property
    def center(self):
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    @property
    def center_x(self):
        return self.center[0]

    @property
    def center_y(self):
        return self.center[1]

    @property
    def size(self):
        x1, y1, x2, y2 = self.bbox
        return (max(0.0, x2 - x1), max(0.0, y2 - y1))

    @property
    def bbox_width(self):
        return self.size[0]

    @property
    def bbox_height(self):
        return self.size[1]


def clamp_bbox(bbox: BBox, width: int, height: int) -> Optional[Tuple[int, int, int, int]]:
    """Clamp an xyxy box to an image, returning ``None`` for an empty box."""
    x1, y1, x2, y2 = bbox
    x1 = max(0, min(width - 1, int(round(x1))))
    y1 = max(0, min(height - 1, int(round(y1))))
    x2 = max(0, min(width, int(round(x2))))
    y2 = max(0, min(height, int(round(y2))))
    return None if x2 <= x1 or y2 <= y1 else (x1, y1, x2, y2)


def parse_source(value: str):
    """Convert a numeric camera argument to int; keep file/URL sources as text."""
    return int(value) if value.strip().lstrip("-").isdigit() else value


def open_writer(path: str, fps: float, size):
    """Create an MP4 writer, making its parent directory when needed."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(destination), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open output video writer: {destination}")
    return writer
