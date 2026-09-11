"""Dependency-light, fixed-threshold object-detection matching helpers."""
import numpy as np


def match_detections(gt_classes, gt_boxes, pred_classes, pred_boxes, scores,
                     confidence=0.4, iou_threshold=0.5):
    """Return class-aware one-to-one TP/FP/FN counts for pixel xyxy boxes."""
    gt_classes = np.asarray(gt_classes)
    gt_boxes = np.asarray(gt_boxes)
    pred_classes = np.asarray(pred_classes)
    pred_boxes = np.asarray(pred_boxes)
    scores = np.asarray(scores)
    valid = np.flatnonzero(gt_classes >= 0)
    candidates = np.flatnonzero((pred_classes >= 0) & (scores >= confidence))
    candidates = candidates[np.argsort(-scores[candidates], kind="stable")]
    matched, matches = set(), []
    for pred_index in candidates:
        choices = [gt_index for gt_index in valid
                   if gt_index not in matched and gt_classes[gt_index] == pred_classes[pred_index]]
        if not choices:
            continue
        boxes = gt_boxes[choices]
        intersection = np.maximum(
            np.minimum(boxes[:, 2:], pred_boxes[pred_index, 2:]) -
            np.maximum(boxes[:, :2], pred_boxes[pred_index, :2]), 0).prod(axis=-1)
        areas = np.maximum(boxes[:, 2:] - boxes[:, :2], 0).prod(axis=-1)
        pred_area = np.maximum(pred_boxes[pred_index, 2:] - pred_boxes[pred_index, :2], 0).prod()
        ious = intersection / np.maximum(areas + pred_area - intersection, 1e-9)
        best = int(np.argmax(ious))
        if ious[best] >= iou_threshold:
            gt_index = choices[best]
            matched.add(gt_index)
            matches.append({"gt_index": int(gt_index), "pred_index": int(pred_index),
                            "iou": float(ious[best])})
    tp = len(matches)
    fp = len(candidates) - tp
    fn = len(valid) - tp
    return {"tp": tp, "fp": fp, "fn": fn,
            "precision": tp/(tp+fp) if tp+fp else 0.0,
            "recall": tp/(tp+fn) if tp+fn else 0.0, "matches": matches}
