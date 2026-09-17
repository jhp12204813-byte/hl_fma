"""Unvalidated white transverse-line baseline; distances are optical Z only."""
import cv2
import numpy as np


def candidates(bgr):
    height, width = bgr.shape[:2]
    top = int(height * .4)
    hsv = cv2.cvtColor(bgr[top:], cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 170), (179, 65, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 9), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w >= width * .3 and w / h >= 4:
            boxes.append([x, y + top, w, h])
    return sorted(boxes)


def measure(rgb_ns, bbox, depths, scale):
    result = dict(rgb_timestamp_ns=rgb_ns, matched_depth_timestamp_ns=None,
                  timestamp_difference_ns=None, bbox=bbox, optical_z_m=None,
                  valid_pixel_count=0, valid_ratio=0., detection_status='candidate',
                  depth_status='no_match')
    if not depths:
        return result
    stamp, depth = min(depths, key=lambda item: abs(item[0] - rgb_ns))
    delta = abs(stamp - rgb_ns)
    if delta > 20_000_000:
        return result
    result.update(matched_depth_timestamp_ns=stamp, timestamp_difference_ns=delta)
    x, y, w, h = bbox
    region = depth[y:y+h, x:x+w].astype(np.float64) * scale
    valid = region[(region >= .1) & (region <= 10)]
    ratio = valid.size / region.size if region.size else 0.
    result.update(valid_pixel_count=int(valid.size), valid_ratio=ratio,
                  depth_status='insufficient_valid_depth')
    if valid.size >= 20 and ratio >= .3:
        result.update(optical_z_m=float(np.median(valid)), depth_status='valid')
    return result
