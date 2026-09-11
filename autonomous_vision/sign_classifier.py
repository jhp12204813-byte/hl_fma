"""Batched sign-panel classification and per-slot temporal stabilization."""

from dataclasses import dataclass
from time import perf_counter

import numpy as np

try:
    from . import config
    from .stability_filter import StableStateFilter
except ImportError:
    import config
    from stability_filter import StableStateFilter


VALID_SIGN_STATES = {"GREEN_ARROW", "RED_X", "LANE_CHANGE"}


@dataclass(frozen=True)
class SignPrediction:
    class_name: str
    confidence: float


@dataclass(frozen=True)
class SlotState:
    slot: int
    lane: str
    raw_state: str
    stable_state: str
    class_confidence: float
    detection_confidence: float
    bbox: object
    observed: bool


class SignPanelClassifier:
    """Use one Ultralytics classification model for every panel.

    ``classify_batch`` passes all crops in one call so three panels do not
    cause three independent model invocations on Jetson.
    """

    def __init__(self, weights, image_size=config.SIGN_CLASSIFY_SIZE,
                 confidence_threshold=config.SIGN_CLASS_CONF_THRESHOLD, device=None):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("ultralytics is required for sign classification") from exc
        self.model = YOLO(str(weights))
        self.image_size = int(image_size)
        self.threshold = float(confidence_threshold)
        self.device = device
        self.last_inference_ms = 0.0

    def classify(self, image):
        return self.classify_batch([image])[0]

    def warmup(self, batch_size=3):
        """Allocate model/CUDA kernels before timing real camera frames."""
        sample = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.classify_batch([sample] * max(1, int(batch_size)))
        return self.last_inference_ms

    def classify_batch(self, images):
        if not images:
            self.last_inference_ms = 0.0
            return []
        started = perf_counter()
        results = self.model.predict(source=images, imgsz=self.image_size,
                                     device=self.device, verbose=False)
        self.last_inference_ms = (perf_counter() - started) * 1000.0
        predictions = []
        for result in results:
            if result.probs is None:
                predictions.append(SignPrediction("UNKNOWN", 0.0))
                continue
            class_id = int(result.probs.top1)
            confidence = float(result.probs.top1conf.item())
            names = result.names
            name = str(names[class_id] if isinstance(names, dict) else names[class_id]).upper()
            if confidence < self.threshold or name not in VALID_SIGN_STATES:
                name = "UNKNOWN"
            predictions.append(SignPrediction(name, confidence))
        return predictions


class SlotStateManager:
    """Maintain raw/stable classification independently for SLOT 1/2/3."""

    def __init__(self):
        self.filters = [StableStateFilter(config.SIGN_CONFIRM_FRAMES,
                                          config.SIGN_LOST_FRAMES)
                        for _ in range(3)]
        self.stable_confidences = [0.0, 0.0, 0.0]

    def update(self, tracked_panels, predictions_by_slot):
        states = []
        for tracked in tracked_panels:
            prediction = predictions_by_slot.get(tracked.slot,
                                                 SignPrediction("UNKNOWN", 0.0))
            stable = self.filters[tracked.slot - 1].update(prediction.class_name)
            if stable == prediction.class_name and stable != "UNKNOWN":
                self.stable_confidences[tracked.slot - 1] = prediction.confidence
            elif stable == "UNKNOWN":
                self.stable_confidences[tracked.slot - 1] = 0.0
            states.append(SlotState(
                slot=tracked.slot,
                lane=config.SLOT_LANE_MAPPING[tracked.slot],
                raw_state=prediction.class_name,
                stable_state=stable,
                # Report the confidence of the current observation even when
                # it was rejected as UNKNOWN.  Stable state retention is kept
                # separately and must not hide why the raw state was rejected.
                class_confidence=prediction.confidence,
                detection_confidence=tracked.confidence,
                bbox=tracked.bbox,
                observed=tracked.observed,
            ))
        return states
