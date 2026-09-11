#!/usr/bin/env python3
"""Save individually detected sign_panel crops for manual class sorting."""

import argparse
from pathlib import Path

import cv2

try:
    from . import config
    from .bbox_tracker import SignPanelTracker
    from .main import extract_observed_panels
    from .utils import parse_source
    from .yolo_detector import YOLODetector
except ImportError:
    import config
    from bbox_tracker import SignPanelTracker
    from main import extract_observed_panels
    from utils import parse_source
    from yolo_detector import YOLODetector


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--detector-weights", default="autonomous_vision/models/detector_best.pt")
    parser.add_argument("--output-dir", default="autonomous_vision/dataset_sign_cls/unclassified")
    parser.add_argument("--save-every", type=int, default=10)
    parser.add_argument("--confidence", type=float, default=config.DETECTION_CONF_THRESHOLD)
    parser.add_argument("--imgsz", type=int, default=config.DETECTION_IMAGE_SIZE)
    parser.add_argument("--device", default=None)
    parser.add_argument("--no-display", action="store_true")
    return parser


def main():
    args = build_parser().parse_args()
    weights = Path(args.detector_weights)
    if not weights.is_file():
        raise SystemExit(f"ERROR: detector weights not found: {weights}")
    try:
        detector = YOLODetector(weights, args.confidence,
                                config.DETECTION_IOU_THRESHOLD, args.imgsz, args.device)
    except RuntimeError as exc:
        raise SystemExit(f"ERROR: {exc}") from exc
    capture = cv2.VideoCapture(parse_source(args.source))
    if not capture.isOpened():
        raise SystemExit(f"ERROR: cannot open source: {args.source}")
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    tracker = SignPanelTracker()
    frame_number = saved = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            detections = detector.detect(frame)
            panels = [item for item in detections if item.class_name == config.SIGN_PANEL_CLASS_NAME]
            tracked = tracker.update(panels)
            crops = extract_observed_panels(frame, tracked)
            preview = frame.copy()
            for tracked_panel in tracked:
                if tracked_panel.bbox is None:
                    continue
                x1, y1, x2, y2 = map(int, tracked_panel.bbox)
                cv2.rectangle(preview, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(preview, f"SLOT {tracked_panel.slot}", (x1, max(18, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 0, 255), 2)
            if frame_number % max(1, args.save_every) == 0:
                for slot, image in crops.items():
                    filename = output_dir / f"frame_{frame_number:08d}_slot_{slot}.jpg"
                    if not cv2.imwrite(str(filename), image):
                        raise RuntimeError(f"Could not save {filename}")
                    saved += 1
            if not args.no_display:
                cv2.imshow("Sign Panel Preview", preview)
                for slot, image in crops.items():
                    cv2.imshow(f"slot_{slot}", image)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            frame_number += 1
    finally:
        capture.release()
        cv2.destroyAllWindows()
    print(f"Saved {saved} unclassified crops to {output_dir}")


if __name__ == "__main__":
    main()
