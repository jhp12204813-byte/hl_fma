#!/usr/bin/env python3
"""Standalone perception: one detector pass, classification, and interpretation."""

import argparse
from pathlib import Path
from time import perf_counter

import cv2

try:
    from . import config
    from .bbox_tracker import BBoxTracker, SignPanelTracker
    from .message_board import MessageBoardReader
    from .sign_board_interpreter import SignBoardInterpreter
    from .sign_classifier import SignPanelClassifier, SlotStateManager
    from .traffic_light_classifier import TrafficLightClassifier
    from .utils import clamp_bbox, open_writer, parse_source
    from .yolo_detector import YOLODetector
except ImportError:
    import config
    from bbox_tracker import BBoxTracker, SignPanelTracker
    from message_board import MessageBoardReader
    from sign_board_interpreter import SignBoardInterpreter
    from sign_classifier import SignPanelClassifier, SlotStateManager
    from traffic_light_classifier import TrafficLightClassifier
    from utils import clamp_bbox, open_writer, parse_source
    from yolo_detector import YOLODetector


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="0", help="Camera index, video path, or stream URL")
    parser.add_argument("--detector-weights", default="autonomous_vision/models/detector_best.pt")
    parser.add_argument("--classifier-weights",
                        help="Optional sign classifier; omit for panel-crop calibration")
    parser.add_argument("--current-lane", choices=("LEFT", "CENTER", "RIGHT", "UNKNOWN"),
                        default="UNKNOWN")
    parser.add_argument("--confidence", type=float, default=config.DETECTION_CONF_THRESHOLD)
    parser.add_argument("--iou", type=float, default=config.DETECTION_IOU_THRESHOLD)
    parser.add_argument("--imgsz", type=int, default=config.DETECTION_IMAGE_SIZE)
    parser.add_argument("--classify-imgsz", type=int, default=config.SIGN_CLASSIFY_SIZE)
    parser.add_argument(
        "--classifier-confidence", type=float,
        default=config.SIGN_CLASS_CONF_THRESHOLD,
        help="Minimum sign-panel class confidence; lower values reduce UNKNOWN but increase false states",
    )
    parser.add_argument("--device", default=None, help="Ultralytics device, e.g. 0 or cpu")
    parser.add_argument("--frame-skip", type=int, default=config.YOLO_FRAME_SKIP)
    parser.add_argument("--debug", action="store_true", help="Show HSV masks and panel crops")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--output", help="Optional annotated MP4 path")
    return parser


def choose_detection(detections, class_name, previous_bbox=None):
    """Choose one object, favoring confidence and previous-track proximity."""
    candidates = [item for item in detections if item.class_name == class_name]
    if not candidates:
        return None
    if previous_bbox is None:
        return max(candidates, key=lambda item: item.confidence)
    px = (previous_bbox[0] + previous_bbox[2]) / 2.0
    py = (previous_bbox[1] + previous_bbox[3]) / 2.0
    diagonal = max(((previous_bbox[2] - previous_bbox[0]) ** 2 +
                    (previous_bbox[3] - previous_bbox[1]) ** 2) ** .5, 1.0)
    return max(candidates, key=lambda item: item.confidence - .05 *
               (((item.center_x - px) ** 2 + (item.center_y - py) ** 2) ** .5 / diagonal))


def extract_observed_panels(frame, tracked_panels):
    """Return {slot: crop} only for boxes actually observed this detector cycle."""
    crops = {}
    for panel in tracked_panels:
        if not panel.observed or panel.bbox is None:
            continue
        bounds = clamp_bbox(panel.bbox, frame.shape[1], frame.shape[0])
        if bounds is None:
            continue
        x1, y1, x2, y2 = bounds
        if (x2 - x1 < config.MIN_SIGN_PANEL_WIDTH or
                y2 - y1 < config.MIN_SIGN_PANEL_HEIGHT):
            continue
        crops[panel.slot] = frame[y1:y2, x1:x2]
    return crops


def draw_traffic(frame, result, debug=False):
    color = config.STATE_COLORS[result.stable_state]
    if result.bbox is None:
        return
    x1, y1, x2, y2 = result.bbox
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, f"traffic_light {result.confidence:.2f} {result.stable_state}",
                (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 2)
    if debug:
        width, height = x2 - x1, y2 - y1
        for name, (rx1, ry1, rx2, ry2) in config.LAMP_ROIS.items():
            a = (x1 + int(rx1 * width), y1 + int(ry1 * height))
            b = (x1 + int(rx2 * width), y1 + int(ry2 * height))
            cv2.rectangle(frame, a, b, (220, 220, 220), 1)
            cv2.putText(frame, name, (a[0], max(12, a[1] - 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, .3, (255, 255, 255), 1)


def draw_panels(frame, slot_states):
    colors = ((255, 0, 255), (255, 255, 0), (0, 165, 255))
    for state in slot_states:
        if state.bbox is None:
            continue
        bounds = clamp_bbox(state.bbox, frame.shape[1], frame.shape[0])
        if bounds is None:
            continue
        x1, y1, x2, y2 = bounds
        color = colors[state.slot - 1]
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2 if state.observed else 1)
        # Show both instantaneous and temporally confirmed states.  This makes
        # it clear whether UNKNOWN came from low classifier confidence or from
        # the three-frame stabilization delay.
        label = (f"SLOT {state.slot}/{state.lane} raw={state.raw_state} "
                 f"stable={state.stable_state} {state.class_confidence:.2f}")
        cv2.putText(frame, label, (x1, max(18, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, color, 2)


def draw_message_board(frame, result):
    if result.bbox is None:
        return
    x1, y1, x2, y2 = result.bbox
    color = (0, 220, 255) if result.active else (130, 130, 130)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    cv2.putText(frame, f"message_board active={result.active} {result.confidence:.2f}",
                (x1, max(20, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 2)


def draw_status(frame, traffic, slots, decision, message, fps, detector_ms, classifier_ms,
                current_lane):
    lines = [f"Traffic Light: {traffic.stable_state}"]
    lines.extend(f"SLOT {state.slot}/{state.lane}: {state.stable_state} "
                 f"{state.class_confidence:.2f}" for state in slots)
    lines.extend([
        f"Current Lane: {current_lane}",
        f"Lane Change Required: {decision.lane_change_required}",
        f"Target Lane: {decision.target_lane}",
        f"Current Lane Allowed: {decision.current_lane_allowed}",
        f"Board Valid: {decision.board_valid}",
        f"Reason: {decision.reason}",
        f"Message Board: detected={message.detected} active={message.active}",
        f"FPS: {fps:.1f}",
        f"YOLO: {detector_ms:.1f} ms",
        f"Sign Classification: {classifier_ms:.1f} ms",
    ])
    for index, text in enumerate(lines):
        y = 25 + index * 22
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .47, (0, 0, 0), 3)
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1)


def run(args):
    detector_path = Path(args.detector_weights)
    if not detector_path.is_file():
        raise FileNotFoundError(f"YOLO detector weights not found: {detector_path}")
    detector = YOLODetector(detector_path, args.confidence, args.iou, args.imgsz, args.device)
    sign_classifier = None
    if args.classifier_weights:
        classifier_path = Path(args.classifier_weights)
        if not classifier_path.is_file():
            raise FileNotFoundError(f"Sign classifier weights not found: {classifier_path}")
        if not 0.0 <= args.classifier_confidence <= 1.0:
            raise ValueError("--classifier-confidence must be between 0 and 1")
        sign_classifier = SignPanelClassifier(classifier_path, args.classify_imgsz,
                                              args.classifier_confidence, args.device)
        print(f"Sign classifier warm-up: {sign_classifier.warmup(3):.1f} ms")
    print(f"Inference device: {detector.runtime_description()}")

    capture = cv2.VideoCapture(parse_source(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open source: {args.source}")
    traffic_tracker = BBoxTracker()
    panel_tracker = SignPanelTracker()
    message_tracker = BBoxTracker()
    traffic_classifier = TrafficLightClassifier()
    slot_manager = SlotStateManager()
    interpreter = SignBoardInterpreter()
    message_reader = MessageBoardReader()
    tracked_panels = panel_tracker.update([])
    frame_index, last_detector_ms, classifier_ms, fps = 0, 0.0, 0.0, 0.0
    previous_time, writer = perf_counter(), None
    frame_skip = max(1, args.frame_skip)
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index % frame_skip == 0:
                # The only detector invocation in this frame.
                detections = detector.detect(frame)
                traffic_tracker.update(choose_detection(
                    detections, config.TRAFFIC_LIGHT_CLASS_NAME, traffic_tracker.current()))
                panel_detections = [item for item in detections
                                    if item.class_name == config.SIGN_PANEL_CLASS_NAME]
                tracked_panels = panel_tracker.update(panel_detections)
                message_tracker.update(choose_detection(
                    detections, config.MESSAGE_BOARD_CLASS_NAME, message_tracker.current()))
                last_detector_ms = detector.last_inference_ms

            traffic = traffic_classifier.classify(
                frame, traffic_tracker.current(), traffic_tracker.last_confidence, args.debug)
            panel_crops = extract_observed_panels(frame, tracked_panels)
            predictions = {}
            if sign_classifier is not None and panel_crops:
                ordered_slots = sorted(panel_crops)
                batch = sign_classifier.classify_batch([panel_crops[slot] for slot in ordered_slots])
                predictions = dict(zip(ordered_slots, batch))
                classifier_ms = sign_classifier.last_inference_ms
            else:
                classifier_ms = 0.0
            slot_states = slot_manager.update(tracked_panels, predictions)
            decision = interpreter.interpret(args.current_lane, slot_states)
            message = message_reader.read(frame, message_tracker.current(),
                                          message_tracker.last_confidence, args.debug)

            now = perf_counter()
            instant_fps = 1.0 / max(now - previous_time, 1e-9)
            fps = instant_fps if fps == 0.0 else .1 * instant_fps + .9 * fps
            previous_time = now
            draw_traffic(frame, traffic, args.debug)
            draw_panels(frame, slot_states)
            draw_message_board(frame, message)
            draw_status(frame, traffic, slot_states, decision, message, fps,
                        last_detector_ms, classifier_ms, args.current_lane)

            if args.output and writer is None:
                source_fps = capture.get(cv2.CAP_PROP_FPS)
                writer = open_writer(args.output, source_fps if source_fps > 0 else 30.0,
                                     (frame.shape[1], frame.shape[0]))
            if writer is not None:
                writer.write(frame)
            if not args.no_display:
                cv2.imshow("Autonomous Vision", frame)
                if args.debug:
                    for name, image in traffic.debug_images.items():
                        cv2.imshow(name, image)
                    for slot, image in panel_crops.items():
                        cv2.imshow(f"sign_panel_slot_{slot}", image)
                    for name, image in message.debug_images.items():
                        cv2.imshow(name, image)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            frame_index += 1
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


def main():
    args = build_parser().parse_args()
    try:
        run(args)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc


if __name__ == "__main__":
    main()
