"""Evaluate obstacle weights for avoidance, where missed obstacles matter most.

Both detector classes are collapsed to one OBSTACLE_AVOIDANCE category. This
means child_dummy/vehicle_obstacle confusion is not counted as a miss. The
script is evaluation-only and never edits labels, datasets, models, or ROS2.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomous_vision.detection_metrics import match_detections
from autonomous_vision.obstacle_classes import CLASS_NAMES


def load_gt(label, width, height):
    rows = np.loadtxt(label, ndmin=2) if label.read_text().strip() else np.empty((0, 5))
    centers = rows[:, 1:3] * [width, height]
    sizes = rows[:, 3:5] * [width, height]
    return np.concatenate([centers-sizes/2, centers+sizes/2], axis=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, nargs="+", required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    thresholds = [round(x, 2) for x in np.arange(0.01, 0.51, 0.01)]
    base = args.data / "images" / args.split
    paths = sorted(p for p in base.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    report = {"policy": "class-agnostic OBSTACLE_AVOIDANCE; rank by FN then FP",
              "iou": 0.5, "agnostic_nms": True, "split": args.split, "models": []}
    for weight in args.models:
        model = YOLO(weight)
        if model.names != dict(enumerate(CLASS_NAMES)):
            raise ValueError(f"Unexpected classes in {weight}: {model.names}")
        totals = {t: [0, 0, 0] for t in thresholds}
        score_confusion = np.zeros((2, 2), dtype=int)
        predictions = (result for start in range(0, len(paths), args.batch)
                       for result in model.predict(
                           source=[str(p) for p in paths[start:start+args.batch]],
                           conf=0.01, iou=0.7, agnostic_nms=True, imgsz=args.imgsz,
                           rect=False, device=args.device, stream=True, verbose=False))
        for result in predictions:
            h, w = result.orig_shape
            rel = Path(result.path).relative_to(base).with_suffix(".txt")
            label = args.data / "labels" / args.split / rel
            gt_boxes = load_gt(label, w, h)
            pred_boxes = result.boxes.xyxy.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            pred_cls = result.boxes.cls.cpu().numpy().astype(int)
            # Avoidance metric: every GT/prediction belongs to the same class.
            gt_common = np.zeros(len(gt_boxes), dtype=int)
            pred_common = np.zeros(len(pred_boxes), dtype=int)
            for threshold, values in totals.items():
                matched = match_detections(gt_common, gt_boxes, pred_common, pred_boxes,
                                           scores, confidence=threshold, iou_threshold=0.5)
                values[0] += matched["tp"]
                values[1] += matched["fp"]
                values[2] += matched["fn"]
            # Keep class confusion informational only, using 0.05 and greedy IoU matches.
            if len(gt_boxes) and len(pred_boxes):
                rows = np.loadtxt(label, ndmin=2)
                gt_cls = rows[:, 0].astype(int)
                order = np.argsort(-scores)
                used = set()
                for j in order:
                    if scores[j] < 0.05:
                        continue
                    x1 = np.maximum(gt_boxes[:, 0], pred_boxes[j, 0]); y1 = np.maximum(gt_boxes[:, 1], pred_boxes[j, 1])
                    x2 = np.minimum(gt_boxes[:, 2], pred_boxes[j, 2]); y2 = np.minimum(gt_boxes[:, 3], pred_boxes[j, 3])
                    inter = np.maximum(0, x2-x1)*np.maximum(0, y2-y1)
                    union = ((gt_boxes[:, 2]-gt_boxes[:, 0])*(gt_boxes[:, 3]-gt_boxes[:, 1]) +
                             (pred_boxes[j, 2]-pred_boxes[j, 0])*(pred_boxes[j, 3]-pred_boxes[j, 1])-inter)
                    iou = inter/np.maximum(union, 1e-9)
                    for i in np.argsort(-iou):
                        if iou[i] >= 0.5 and int(i) not in used:
                            used.add(int(i)); score_confusion[gt_cls[i], pred_cls[j]] += 1; break
        metrics = []
        for threshold, (tp, fp, fn) in totals.items():
            recall = tp/(tp+fn) if tp+fn else 0.0
            precision = tp/(tp+fp) if tp+fp else 0.0
            f2 = 5*precision*recall/(4*precision+recall) if precision+recall else 0.0
            metrics.append({"confidence": threshold, "tp": tp, "fp": fp, "fn": fn,
                            "recall": recall, "precision": precision, "f2": f2})
        # First minimize FN, then FP. Also expose best F2 as a less aggressive option.
        recall_first = min(metrics, key=lambda x: (x["fn"], x["fp"], -x["confidence"]))
        best_f2 = max(metrics, key=lambda x: (x["f2"], x["recall"], -x["fp"]))
        item = {"weight": str(weight), "recall_first": recall_first, "best_f2": best_f2,
                "class_confusion_at_0.05": score_confusion.tolist(), "thresholds": metrics}
        report["models"].append(item)
        print(weight)
        print(" recall-first", recall_first)
        print(" best-F2", best_f2)
        print(" class confusion rows=GT, cols=prediction", score_confusion.tolist())
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
