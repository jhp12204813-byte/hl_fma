"""RGB candidate function copied from d435i_recorder.stop_line (Apache-2.0).
White transverse markings only; no distance or semantic accuracy claim.
"""
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

