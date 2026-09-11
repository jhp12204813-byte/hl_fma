"""One-pass Ultralytics detection for traffic light and signal-board parts."""

from time import perf_counter
from typing import List

try:
    from .utils import Detection
except ImportError:  # Allows direct execution from this directory.
    from utils import Detection


class YOLODetector:
    """Find traffic_light, sign_panel, and message_board in one inference.

    Importing Ultralytics is deferred until construction so classifier unit
    tests can run in the lightweight OpenCV virtual environment.
    """

    def __init__(self, weights, confidence=0.35, iou=0.45, image_size=640,
                 device=None):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed. Install autonomous_vision/"
                "requirements-yolo.txt in the intended virtual environment."
            ) from exc
        self.model = YOLO(str(weights))
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.image_size = int(image_size)
        self.device = device
        self.last_inference_ms = 0.0

    def detect(self, frame) -> List[Detection]:
        started = perf_counter()
        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.image_size,
            device=self.device,
            verbose=False,
        )
        self.last_inference_ms = (perf_counter() - started) * 1000.0
        detections = []
        if not results:
            return detections
        result = results[0]
        names = result.names
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            name = names[class_id] if isinstance(names, dict) else names[class_id]
            if str(name) not in {"traffic_light", "sign_panel", "message_board"}:
                continue
            xyxy = tuple(float(v) for v in box.xyxy[0].tolist())
            detections.append(Detection(
                class_id=class_id,
                class_name=str(name),
                confidence=float(box.conf[0].item()),
                bbox=xyxy,
            ))
        return detections

    def runtime_description(self):
        """Return a short CPU/CUDA description without requiring direct torch use."""
        try:
            import torch
            if torch.cuda.is_available():
                return f"CUDA: {torch.cuda.get_device_name(0)}"
            return "CPU (CUDA unavailable)"
        except Exception:
            return "device information unavailable"
