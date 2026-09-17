"""Experimental D435i image-space marking candidates; no vehicle interface.

This is a new vision baseline, not MORAI's GPS stop-line detector.
No candidate is a validated stop command or a calibrated lane estimate.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionConfig:
    roi_top: float = .48
    yellow_h_min: int = 12
    yellow_h_max: int = 42
    yellow_s_min: int = 65
    yellow_v_min: int = 70
    white_s_max: int = 65
    white_v_min: int = 150
    stop_min_width: float = .35

    def __post_init__(self):
        if not 0 <= self.roi_top < .9 or not 0 < self.stop_min_width <= 1:
            raise ValueError('Invalid ROI/stop width ratio')
        if not 0 <= self.yellow_h_min <= self.yellow_h_max <= 179:
            raise ValueError('Invalid hue range')
        for value in (self.yellow_s_min, self.yellow_v_min,
                      self.white_s_max, self.white_v_min):
            if not 0 <= value <= 255:
                raise ValueError('Invalid saturation/value threshold')


def detect(bgr, config=VisionConfig()):
    if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('Expected uint8 BGR')
    h, w = bgr.shape[:2]
    top = int(h * config.roi_top)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (config.yellow_h_min, config.yellow_s_min,
                             config.yellow_v_min), (config.yellow_h_max, 255, 255))
    white = cv2.inRange(hsv, (0, 0, config.white_v_min), (179, config.white_s_max, 255))
    result = dict(lane_candidates=[], stop_candidates=[], crosswalk_pattern=False,
                  status='unvalidated_image_candidates', roi_top_px=top)
    zebra = []
    for color, mask in (('yellow', yellow), ('white', white)):
        mask[:top] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < w * h * .00025:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            (_, _), (rw, rh), _ = cv2.minAreaRect(contour)
            length, thickness = max(rw, rh), min(rw, rh)
            if thickness <= 0:
                continue
            vx, vy, px, py = [float(v) for v in cv2.fitLine(
                contour, cv2.DIST_L2, 0, .01, .01).ravel()]
            # Wide longitudinal white blocks resemble zebra stripes, not thin lanes.
            if color == 'white' and bw > .045*w and bh > .05*h and area/(bw*bh) > .35:
                zebra.append([x, y, bw, bh])
            if (color == 'yellow' and length > .09*h and length/thickness >= 5
                    and thickness < .045*w and abs(vy) > .22
                    and bh > .055*h):
                ys = [y, y+bh-1]
                points = [[int(np.clip(px+(yy-py)*vx/vy, 0, w-1)), yy] for yy in ys]
                result['lane_candidates'].append(dict(color=color, points=points,
                                                       bbox=[x, y, bw, bh]))
            if (color == 'white' and bw >= config.stop_min_width*w
                    and bw/bh >= 5 and area/(bw*bh) >= .45 and abs(vy) < .20):
                result['stop_candidates'].append(dict(bbox=[x, y, bw, bh],
                                                       status='transverse_white_candidate'))
    # A same-row pair is only an ambiguity flag; never silently discard its evidence.
    result['crosswalk_pattern'] = any(
        abs(a[1]+a[3]/2-b[1]-b[3]/2) < .08*h
        and abs(a[0]-b[0]) > .08*w
        for i, a in enumerate(zebra) for b in zebra[i+1:])
    if result['crosswalk_pattern']:
        for candidate in result['stop_candidates']:
            candidate['status'] = 'ambiguous_crosswalk'
    return result


def annotate(bgr, result, draw_lanes=True, text_y=22):
    out = bgr.copy()
    cv2.line(out, (0, result['roi_top_px']), (out.shape[1]-1, result['roi_top_px']), (100, 100, 100))
    for lane in result['lane_candidates'] if draw_lanes else []:
        cv2.line(out, tuple(lane['points'][0]), tuple(lane['points'][1]), (0, 255, 0), 2)
    for stop in result['stop_candidates']:
        x, y, w, h = stop['bbox']
        cv2.rectangle(out, (x, y), (x+w, y+h), (0, 128, 255), 2)
        cv2.putText(out, stop['status'], (x, max(20, y-5)), 0, .45, (0, 128, 255), 1)
    text = 'CANDIDATES ONLY' + (' | CROSSWALK?' if result['crosswalk_pattern'] else '')
    cv2.putText(out, text, (10, text_y), 0, .5, (0, 255, 255), 1)
    return out
