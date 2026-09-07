"""Unit tests that do not require Ultralytics or a trained checkpoint."""

import cv2
import numpy as np
from types import SimpleNamespace

from autonomous_vision import config
from autonomous_vision.bbox_tracker import BBoxTracker, SignPanelTracker, TrackedPanel
from autonomous_vision.message_board import MessageBoardReader
from autonomous_vision.sign_board_interpreter import SignBoardInterpreter
from autonomous_vision.sign_classifier import (
    SignPanelClassifier, SignPrediction, SlotStateManager,
)
from autonomous_vision.traffic_light_classifier import (
    SignalStateStabilizer, TrafficLightClassifier,
)
from autonomous_vision.main import choose_detection
from autonomous_vision.utils import Detection


def colored_frame(active):
    frame = np.zeros((120, 420, 3), np.uint8)
    bbox = (10, 10, 410, 110)
    colors = {"RED": (0, 0, 255), "YELLOW": (0, 255, 255),
              "GREEN_LEFT": (0, 255, 0), "GREEN": (0, 255, 0)}
    for name in active:
        x1, y1, x2, y2 = config.LAMP_ROIS[name]
        center = (10 + int((x1 + x2) * 200), 10 + int((y1 + y2) * 50))
        cv2.circle(frame, center, 18, colors[name], -1)
    return frame, bbox


def test_all_signal_states_and_combined():
    for active, expected in [(["RED"], "RED"), (["YELLOW"], "YELLOW"),
                             (["GREEN_LEFT"], "GREEN_LEFT"), (["GREEN"], "GREEN"),
                             (["GREEN_LEFT", "GREEN"], "GREEN_AND_LEFT")]:
        classifier = TrafficLightClassifier()
        frame, bbox = colored_frame(active)
        result = classifier.classify(frame, bbox)
        assert result.raw_state == expected


def test_state_confirmation_and_loss():
    stable = SignalStateStabilizer(confirm_frames=3, lost_frames=2)
    assert stable.update("GREEN") == "UNKNOWN"
    assert stable.update("GREEN") == "UNKNOWN"
    assert stable.update("GREEN") == "GREEN"
    assert stable.update("UNKNOWN") == "GREEN"
    assert stable.update("UNKNOWN") == "UNKNOWN"


def test_bbox_smoothing_and_expiry():
    tracker = BBoxTracker(alpha=0.5, history_size=3, max_missing=2)
    detection = Detection(0, "traffic_light", .9, (10, 20, 110, 120))
    assert tracker.update(detection) == (10.0, 20.0, 110.0, 120.0)
    assert tracker.update(None) is not None
    assert tracker.update(None) is not None
    assert tracker.update(None) is None


def test_three_detection_classes_are_selected_independently():
    detections = [
        Detection(0, "traffic_light", .8, (10, 10, 50, 40)),
        Detection(1, "sign_panel", .9, (100, 40, 160, 100)),
        Detection(2, "message_board", .7, (100, 120, 260, 190)),
    ]
    assert choose_detection(detections, "traffic_light").class_id == 0
    assert choose_detection(detections, "message_board").class_id == 2


def panel(center_x, confidence=.9):
    return Detection(1, "sign_panel", confidence,
                     (center_x - 20, 20, center_x + 20, 70))


def test_panel_slots_sort_then_preserve_identity_when_one_is_missing():
    tracker = SignPanelTracker()
    first = tracker.update([panel(300), panel(100), panel(200)])
    assert [item.bbox[0] for item in first] == [80.0, 180.0, 280.0]
    second = tracker.update([panel(105), panel(305)])
    assert [item.observed for item in second] == [True, False, True]


def test_partial_panels_before_initialization_are_not_guessed():
    result = SignPanelTracker().update([panel(100), panel(200)])
    assert all(item.bbox is None for item in result)


def slot(slot_number, state):
    return SimpleNamespace(slot=slot_number, stable_state=state)


def test_lane_change_interpretation_and_ambiguous_fail_closed():
    interpreter = SignBoardInterpreter()
    valid = interpreter.interpret("CENTER", [slot(1, "RED_X"),
                                             slot(2, "LANE_CHANGE"),
                                             slot(3, "GREEN_ARROW")])
    assert valid.lane_change_required and valid.target_lane == "RIGHT"
    assert valid.target_slot == 3 and valid.board_valid
    ambiguous = interpreter.interpret("CENTER", [slot(1, "GREEN_ARROW"),
                                                 slot(2, "LANE_CHANGE"),
                                                 slot(3, "GREEN_ARROW")])
    assert not ambiguous.board_valid and ambiguous.target_lane == "UNKNOWN"


def test_message_board_on_off():
    reader = MessageBoardReader()
    dark = np.zeros((100, 220, 3), np.uint8)
    assert not reader.read(dark, (10, 10, 210, 90), .8).active
    active = dark.copy()
    cv2.rectangle(active, (50, 35), (170, 55), (0, 255, 0), -1)
    result = reader.read(active, (10, 10, 210, 90), .8, debug=True)
    assert result.active and "message_board_active_mask" in result.debug_images


def test_sign_classifier_batches_and_rejects_low_confidence():
    class FakeModel:
        calls = 0

        def predict(self, source, **kwargs):
            self.calls += 1
            return [
                SimpleNamespace(probs=SimpleNamespace(top1=0, top1conf=np.array(.95)),
                                names={0: "GREEN_ARROW"}),
                SimpleNamespace(probs=SimpleNamespace(top1=0, top1conf=np.array(.40)),
                                names={0: "GREEN_ARROW"}),
            ]

    classifier = SignPanelClassifier.__new__(SignPanelClassifier)
    classifier.model, classifier.image_size = FakeModel(), 224
    classifier.threshold, classifier.device = .70, None
    results = classifier.classify_batch([np.zeros((20, 20, 3), np.uint8)] * 2)
    assert classifier.model.calls == 1
    assert [item.class_name for item in results] == ["GREEN_ARROW", "UNKNOWN"]


def test_slot_classification_requires_consecutive_frames():
    manager = SlotStateManager()
    tracks = [TrackedPanel(index, (10, 10, 50, 50), .9, True)
              for index in (1, 2, 3)]
    predictions = {index: SignPrediction("RED_X", .9) for index in (1, 2, 3)}
    assert manager.update(tracks, predictions)[0].stable_state == "UNKNOWN"
    assert manager.update(tracks, predictions)[0].stable_state == "UNKNOWN"
    assert manager.update(tracks, predictions)[0].stable_state == "RED_X"
