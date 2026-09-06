#!/usr/bin/env python3
"""Calibrated dual-lane following and stop-line detection for the NUCLEO car."""

import argparse
import json
from pathlib import Path
import re
import time

import cv2
import numpy as np
import serial


STEERING_CENTER = 2182
STEERING_MIN = 50
STEERING_MAX = 4040
CALIBRATION_FILE = Path(__file__).with_name("left_lane_80cm.json")


def weighted_fit(candidates, reference_y, width, side):
    if not candidates:
        return None

    projected_x = np.array([
        (float(reference_y) - candidate[1]) / candidate[0]
        for candidate in candidates
    ])
    # Select the lane boundary nearest the vehicle corridor when several
    # parallel yellow structures (curb/wall/guardrail) are visible.
    percentile = 65.0 if side == "left" else 35.0
    anchor_x = float(np.percentile(projected_x, percentile))
    inlier_limit = max(25.0, float(width) * 0.08)
    inliers = [
        candidate for candidate, x_at_reference in zip(candidates, projected_x)
        if abs(float(x_at_reference) - anchor_x) <= inlier_limit
    ]
    if not inliers:
        return None

    weight_sum = sum(candidate[2] for candidate in inliers)
    slope = sum(candidate[0] * candidate[2] for candidate in inliers) / weight_sum
    lane_x = sum(
        ((float(reference_y) - candidate[1]) / candidate[0]) * candidate[2]
        for candidate in inliers
    ) / weight_sum
    intercept = float(reference_y) - slope * lane_x
    return slope, intercept, min(1.0, weight_sum / 500.0)


def detect_lanes(
    frame,
    reference_y_ratio,
    roi_top_y_ratio,
    roi_top_right_x_ratio,
    roi_bottom_right_x_ratio,
    yellow_h_min,
    yellow_h_max,
    yellow_s_min,
    yellow_v_min,
    yellow_lab_l_min,
    yellow_lab_a_max,
    yellow_lab_b_min,
    hough_threshold,
    lane_slope_min,
    lane_slope_max,
):
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(
        hsv, np.array([yellow_h_min, yellow_s_min, yellow_v_min], dtype=np.uint8),
        np.array([yellow_h_max, 255, 255], dtype=np.uint8)
    )
    # Direct sunlight can wash the yellow paint down to very low HSV
    # saturation.  Lab keeps the yellow/blue component usable in that case;
    # the a-channel limit rejects neutral white road markings and pavement.
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    washed_yellow = cv2.inRange(
        lab,
        np.array([yellow_lab_l_min, 0, yellow_lab_b_min], dtype=np.uint8),
        np.array([255, yellow_lab_a_max, 255], dtype=np.uint8),
    )
    # The selected left boundary is yellow. Restricting this detector to yellow
    # avoids following bright pavement seams, curbs, or stop lines as a lane.
    lane_mask = cv2.bitwise_or(yellow, washed_yellow)
    lane_mask = cv2.morphologyEx(
        lane_mask, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8)
    )
    lane_mask = cv2.GaussianBlur(lane_mask, (5, 5), 0)
    edges = cv2.Canny(lane_mask, 55, 150)

    left_polygon = np.array([[
        (0, height - 1),
        (0, int(height * roi_top_y_ratio)),
        (int(width * roi_top_right_x_ratio), int(height * roi_top_y_ratio)),
        (int(width * roi_bottom_right_x_ratio), height - 1),
    ]], dtype=np.int32)
    right_polygon = np.array([[
        (int(width * (1.0 - roi_bottom_right_x_ratio)), height - 1),
        (int(width * (1.0 - roi_top_right_x_ratio)), int(height * roi_top_y_ratio)),
        (width - 1, int(height * roi_top_y_ratio)),
        (width - 1, height - 1),
    ]], dtype=np.int32)
    reference_y = int(height * reference_y_ratio)

    def detect_one(polygon, side):
        roi = np.zeros_like(edges)
        cv2.fillPoly(roi, polygon, 255)
        cropped = cv2.bitwise_and(edges, roi)
        segments = cv2.HoughLinesP(
            cropped, 1, np.pi / 180, threshold=hough_threshold,
            minLineLength=max(28, width // 18),
            maxLineGap=max(35, width // 10),
        )
        candidates = []
        if segments is not None:
            for x1, y1, x2, y2 in segments[:, 0]:
                if x1 == x2:
                    continue
                slope = (float(y2) - float(y1)) / (float(x2) - float(x1))
                if not (lane_slope_min <= abs(slope) <= lane_slope_max):
                    continue
                if side == "left" and slope >= 0.0:
                    continue
                if side == "right" and slope <= 0.0:
                    continue
                mean_x = (float(x1) + float(x2)) * 0.5
                if side == "left" and mean_x > width * roi_bottom_right_x_ratio:
                    continue
                if side == "right" and mean_x < width * (1.0 - roi_bottom_right_x_ratio):
                    continue
                intercept = float(y1) - slope * float(x1)
                length = float(np.hypot(x2 - x1, y2 - y1))
                candidates.append((slope, intercept, length))

        fit = weighted_fit(candidates, reference_y, width, side)
        if fit is None:
            return cropped, None, None, 0.0

        slope, intercept, confidence = fit
        lane_x = (float(reference_y) - intercept) / slope
        if side == "left" and not (-float(width) <= lane_x <= width * 0.72):
            return cropped, None, None, 0.0
        if side == "right" and not (width * 0.28 <= lane_x <= float(width) * 2.0):
            return cropped, None, None, 0.0
        y_top = int(height * 0.36)
        y_bottom = height - 1
        line = (
            int((float(y_bottom) - intercept) / slope), y_bottom,
            int((float(y_top) - intercept) / slope), y_top,
        )
        return cropped, line, lane_x, confidence

    left_result = detect_one(left_polygon, "left")
    right_result = detect_one(right_polygon, "right")
    return left_polygon, right_polygon, left_result, right_result


def detect_stop_line(frame, trigger_y_ratio):
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(
        hsv, np.array([0, 0, 175], dtype=np.uint8),
        np.array([180, 70, 255], dtype=np.uint8)
    )
    roi = np.zeros_like(white)
    cv2.rectangle(
        roi, (int(width * 0.05), int(height * 0.45)),
        (int(width * 0.95), int(height * 0.92)), 255, -1
    )
    masked = cv2.bitwise_and(white, roi)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (31, 3))
    masked = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width < width * 0.38:
            continue
        if box_height < 3 or box_height > height * 0.16:
            continue
        if box_width / float(box_height) < 4.0:
            continue
        center_y = y + box_height * 0.5
        if center_y < height * trigger_y_ratio:
            continue
        if best is None or center_y > best[4]:
            best = (x, y, box_width, box_height, center_y)
    return masked, best


def load_calibration(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        left = data.get("left_x_normalized")
        right = data.get("right_x_normalized")
        return (
            None if left is None else float(left),
            None if right is None else float(right),
        )
    except (FileNotFoundError, ValueError, TypeError, json.JSONDecodeError):
        return None, None


def save_calibration(path, left_normalized, right_normalized):
    data = {"distance_from_left_lane_cm": 80}
    if left_normalized is not None:
        data["left_x_normalized"] = float(left_normalized)
    if right_normalized is not None:
        data["right_x_normalized"] = float(right_normalized)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def steering_packet(target):
    safe_target = max(STEERING_MIN, min(STEERING_MAX, int(target)))
    return f"T{safe_target:04d}".encode("ascii")


def calculate_steering_target(
    left_x,
    right_x,
    left_reference_x,
    right_reference_x,
    lane_center_reference_x,
    steering_gain,
    max_delta,
):
    if left_x is not None and right_x is not None:
        detected_x = (left_x + right_x) * 0.5
        if left_reference_x is not None and right_reference_x is not None:
            reference_x = (left_reference_x + right_reference_x) * 0.5
        else:
            reference_x = lane_center_reference_x
        tracking_mode = "BOTH"
    elif left_x is not None and left_reference_x is not None:
        detected_x = left_x
        reference_x = left_reference_x
        tracking_mode = "LEFT ONLY"
    elif right_x is not None and right_reference_x is not None:
        detected_x = right_x
        reference_x = right_reference_x
        tracking_mode = "RIGHT ONLY"
    else:
        return None, None, "LANES LOST"

    lateral_error_px = reference_x - detected_x
    lower = max(STEERING_MIN, STEERING_CENTER - max_delta)
    upper = min(STEERING_MAX, STEERING_CENTER + max_delta)
    raw_target = STEERING_CENTER + steering_gain * lateral_error_px
    target = int(round(max(lower, min(upper, raw_target))))
    return lateral_error_px, target, tracking_mode


def try_send_stop(board):
    if board is None or not board.is_open:
        return
    try:
        # The firmware uses polling RX and may miss a byte while transmitting
        # telemetry/ACK text. Repeat STOP so safety does not depend only on the
        # independent 700 ms firmware command timeout.
        for _ in range(3):
            board.write(b"X")
            board.flush()
            time.sleep(0.02)
    except (serial.SerialException, OSError):
        pass


def draw_display(
    frame, left_roi, right_roi, left_line, right_line, left_x, right_x,
    left_reference_x, right_reference_x, lane_center_reference_x,
    reference_y_ratio, left_confidence, right_confidence, tracking_mode,
    stop_line, auto_enabled, stop_latched, dry_run, lateral_error_px,
    calculated_target, command_target, fps,
):
    display = frame.copy()
    height, width = display.shape[:2]
    reference_y = int(height * reference_y_ratio)
    cv2.polylines(display, left_roi, True, (0, 255, 255), 2)
    cv2.polylines(display, right_roi, True, (255, 255, 0), 2)

    if left_line is not None:
        cv2.line(display, left_line[:2], left_line[2:], (0, 255, 0), 6)
        cv2.circle(display, (int(left_x), reference_y), 8, (0, 255, 0), -1)
        cv2.putText(
            display, "LEFT", (max(4, int(left_x) + 8), reference_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
        )
    if right_line is not None:
        cv2.line(display, right_line[:2], right_line[2:], (0, 165, 255), 6)
        cv2.circle(display, (int(right_x), reference_y), 8, (0, 165, 255), -1)
        cv2.putText(
            display, "RIGHT", (min(width - 70, int(right_x) + 8), reference_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1
        )

    for reference_x, label in (
        (left_reference_x, "L ref"),
        (right_reference_x, "R ref"),
    ):
        if reference_x is not None:
            cv2.circle(display, (int(reference_x), reference_y), 7, (255, 0, 255), 2)
            cv2.putText(
                display, label, (max(4, min(width - 50, int(reference_x) + 6)), reference_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1,
            )

    if left_x is not None and right_x is not None:
        detected_center = int(round((left_x + right_x) * 0.5))
        cv2.circle(display, (detected_center, reference_y), 8, (255, 255, 0), -1)
        cv2.putText(
            display, "detected center", (max(4, detected_center - 65), reference_y - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1,
        )
    cv2.circle(display, (int(lane_center_reference_x), reference_y), 9, (255, 0, 255), 2)
    cv2.putText(
        display, "target center", (int(lane_center_reference_x) + 8, reference_y - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1,
    )
    cv2.line(display, (0, reference_y), (width - 1, reference_y), (255, 0, 0), 1)

    if stop_line is not None:
        x, y, box_width, box_height, _ = stop_line
        cv2.rectangle(display, (x, y), (x + box_width, y + box_height), (0, 0, 255), 3)

    if dry_run:
        mode, mode_color = "DRY RUN - NO SERIAL COMMANDS", (0, 255, 255)
    elif stop_latched:
        mode, mode_color = "STOP LINE - LATCHED", (0, 0, 255)
    elif auto_enabled:
        mode, mode_color = "AUTO LANE-CENTER FOLLOW", (0, 255, 255)
    else:
        mode, mode_color = "SAFE STOP / CALIBRATION", (255, 255, 255)

    error_text = "LANES LOST"
    if lateral_error_px is not None:
        error_text = f"tracking={tracking_mode}  lateral error={lateral_error_px:+.1f}px"
    calibration_text = (
        "lane references: OK"
        if left_reference_x is not None and right_reference_x is not None
        else "Center vehicle, show both lanes, press K"
    )
    cv2.putText(display, mode, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, mode_color, 2)
    cv2.putText(display, calibration_text, (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 0, 255), 2)
    error_color = (0, 0, 255) if lateral_error_px is None else (0, 255, 0)
    cv2.putText(display, error_text, (18, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.62, error_color, 2)
    calculated_text = "calculated steer=N/A"
    if calculated_target is not None:
        calculated_text = f"calculated steer={calculated_target} command={command_target}"
    cv2.putText(
        display,
        f"{calculated_text}  conf L={left_confidence:.2f} R={right_confidence:.2f}",
        (18, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (0, 255, 0), 2
    )
    cv2.putText(
        display, f"FPS={fps:.1f}", (width - 125, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2
    )
    cv2.putText(
        display, "K calibrate | G go | X/Space stop | R reset | Q quit",
        (18, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2
    )
    return display


def camera_source(value):
    return int(value) if value.isdigit() else value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--camera", default="/dev/video8", help="camera index or device path")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--reference-y", type=float, default=0.45)
    parser.add_argument("--steering-gain", type=float, default=6.0)
    parser.add_argument("--max-steering-delta", type=int, default=650)
    parser.add_argument("--stop-line-y", type=float, default=0.60)
    parser.add_argument(
        "--reference-x", "--left-reference-x", dest="left_reference_x",
        type=float,
        help="desired left-lane x as 0.0-1.0 image ratio; overrides calibration file",
    )
    parser.add_argument(
        "--right-reference-x", type=float,
        help="desired right-lane x as 0.0-1.0 image ratio; overrides calibration file",
    )
    parser.add_argument(
        "--lane-center-x", type=float, default=0.50,
        help="desired lane center as 0.0-1.0 image ratio when no two-line calibration exists",
    )
    parser.add_argument("--lane-lost-timeout", type=float, default=0.25)
    parser.add_argument(
        "--run-duration", type=float,
        help="automatically stop this many seconds after G",
    )
    parser.add_argument(
        "--max-speed-kmh", type=float,
        help="stop if STM32 telemetry reports a greater absolute speed",
    )
    parser.add_argument("--roi-top-y", type=float, default=0.32)
    parser.add_argument("--roi-top-right-x", type=float, default=0.62)
    parser.add_argument("--roi-bottom-right-x", type=float, default=0.72)
    parser.add_argument("--yellow-h-min", type=int, default=10)
    parser.add_argument("--yellow-h-max", type=int, default=45)
    parser.add_argument("--yellow-s-min", type=int, default=55)
    parser.add_argument("--yellow-v-min", type=int, default=65)
    parser.add_argument("--yellow-lab-l-min", type=int, default=145)
    parser.add_argument("--yellow-lab-a-max", type=int, default=128)
    parser.add_argument("--yellow-lab-b-min", type=int, default=135)
    parser.add_argument("--hough-threshold", type=int, default=25)
    parser.add_argument("--min-confidence", type=float, default=0.12)
    parser.add_argument("--lane-slope-min", type=float, default=0.18)
    parser.add_argument("--lane-slope-max", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true", help="camera only; do not open serial")
    args = parser.parse_args()

    if not (0.45 <= args.reference_y <= 0.85):
        parser.error("--reference-y must be between 0.45 and 0.85")
    if not (0.50 <= args.stop_line_y <= 0.85):
        parser.error("--stop-line-y must be between 0.50 and 0.85")
    if args.left_reference_x is not None and not (0.0 <= args.left_reference_x <= 1.0):
        parser.error("--left-reference-x must be between 0.0 and 1.0")
    if args.right_reference_x is not None and not (0.0 <= args.right_reference_x <= 1.0):
        parser.error("--right-reference-x must be between 0.0 and 1.0")
    if not (0.0 <= args.lane_center_x <= 1.0):
        parser.error("--lane-center-x must be between 0.0 and 1.0")
    if args.max_steering_delta < 0:
        parser.error("--max-steering-delta must be non-negative")
    if args.lane_lost_timeout <= 0.0:
        parser.error("--lane-lost-timeout must be positive")
    if args.run_duration is not None and args.run_duration <= 0.0:
        parser.error("--run-duration must be positive")
    if args.max_speed_kmh is not None and args.max_speed_kmh <= 0.0:
        parser.error("--max-speed-kmh must be positive")
    if not (0.0 < args.roi_top_y < 1.0):
        parser.error("--roi-top-y must be between 0.0 and 1.0")
    if not (0.0 < args.roi_top_right_x <= args.roi_bottom_right_x <= 1.0):
        parser.error("ROI right x ratios must satisfy 0 < top <= bottom <= 1")
    if not (0 <= args.yellow_h_min <= args.yellow_h_max <= 179):
        parser.error("yellow hue limits must satisfy 0 <= min <= max <= 179")
    if not (0 <= args.yellow_s_min <= 255 and 0 <= args.yellow_v_min <= 255):
        parser.error("yellow saturation/value minimums must be between 0 and 255")
    if not all(
        0 <= value <= 255
        for value in (
            args.yellow_lab_l_min,
            args.yellow_lab_a_max,
            args.yellow_lab_b_min,
        )
    ):
        parser.error("yellow Lab thresholds must be between 0 and 255")
    if args.hough_threshold <= 0:
        parser.error("--hough-threshold must be positive")
    if not (0.0 <= args.min_confidence <= 1.0):
        parser.error("--min-confidence must be between 0.0 and 1.0")
    if not (0.0 < args.lane_slope_min < args.lane_slope_max):
        parser.error("lane slope limits must satisfy 0 < min < max")

    board = None
    camera = None
    auto_enabled = False
    auto_started_at = None
    stop_latched = False
    stop_frames = 0
    lane_missing_since = None
    last_control_time = 0.0
    filtered_steering = float(STEERING_CENTER)
    lane_width_px = None
    calibrated_left, calibrated_right = load_calibration(CALIBRATION_FILE)
    left_reference_normalized = (
        args.left_reference_x
        if args.left_reference_x is not None
        else calibrated_left
    )
    right_reference_normalized = (
        args.right_reference_x
        if args.right_reference_x is not None
        else calibrated_right
    )
    if left_reference_normalized is not None and right_reference_normalized is None:
        right_reference_normalized = 2.0 * args.lane_center_x - left_reference_normalized
    elif right_reference_normalized is not None and left_reference_normalized is None:
        left_reference_normalized = 2.0 * args.lane_center_x - right_reference_normalized
    previous_frame_time = None
    display_fps = 0.0
    telemetry_buffer = ""
    max_speed_mm_s = (
        None if args.max_speed_kmh is None
        else args.max_speed_kmh * 1000.0 / 3.6
    )

    def send(command):
        if board is not None:
            board.write(command)

    def send_stop():
        try_send_stop(board)

    try:
        if not args.dry_run:
            board = serial.Serial(args.port, 115200, timeout=0)
            board.reset_input_buffer()
            send_stop()

        camera = cv2.VideoCapture(camera_source(args.camera))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not camera.isOpened():
            raise RuntimeError(f"Cannot open camera {args.camera}")

        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera frame read failed")

            left_roi, right_roi, left_result, right_result = detect_lanes(
                frame,
                args.reference_y,
                args.roi_top_y,
                args.roi_top_right_x,
                args.roi_bottom_right_x,
                args.yellow_h_min,
                args.yellow_h_max,
                args.yellow_s_min,
                args.yellow_v_min,
                args.yellow_lab_l_min,
                args.yellow_lab_a_max,
                args.yellow_lab_b_min,
                args.hough_threshold,
                args.lane_slope_min,
                args.lane_slope_max,
            )
            _, left_line, left_x, left_confidence = left_result
            _, right_line, right_x, right_confidence = right_result
            _, stop_line = detect_stop_line(frame, args.stop_line_y)
            frame_width = float(frame.shape[1])
            left_reference_x = (
                None if left_reference_normalized is None
                else left_reference_normalized * frame_width
            )
            right_reference_x = (
                None if right_reference_normalized is None
                else right_reference_normalized * frame_width
            )
            if left_reference_x is not None and right_reference_x is not None:
                lane_center_reference_x = (left_reference_x + right_reference_x) * 0.5
            else:
                lane_center_reference_x = args.lane_center_x * frame_width

            now = time.monotonic()
            if previous_frame_time is not None and now > previous_frame_time:
                instant_fps = 1.0 / (now - previous_frame_time)
                display_fps = instant_fps if display_fps == 0.0 else 0.9 * display_fps + 0.1 * instant_fps
            previous_frame_time = now
            control_left_x = left_x if left_confidence >= args.min_confidence else None
            control_right_x = right_x if right_confidence >= args.min_confidence else None
            if (
                control_left_x is not None
                and control_right_x is not None
                and control_left_x >= control_right_x
            ):
                if left_confidence <= right_confidence:
                    control_left_x = None
                else:
                    control_right_x = None
            if control_left_x is not None and control_right_x is not None:
                observed_width = control_right_x - control_left_x
                if frame_width * 0.15 <= observed_width <= frame_width * 1.20:
                    lane_width_px = (
                        observed_width
                        if lane_width_px is None
                        else 0.85 * lane_width_px + 0.15 * observed_width
                    )
            if lane_width_px is not None:
                # These references make the steering error continuous when
                # BOTH changes to LEFT ONLY or RIGHT ONLY.
                left_reference_x = lane_center_reference_x - lane_width_px * 0.5
                right_reference_x = lane_center_reference_x + lane_width_px * 0.5
            lateral_error_px, calculated_target, tracking_mode = calculate_steering_target(
                control_left_x,
                control_right_x,
                left_reference_x,
                right_reference_x,
                lane_center_reference_x,
                args.steering_gain,
                args.max_steering_delta,
            )
            stop_frames = stop_frames + 1 if stop_line is not None else 0
            if auto_enabled:
                if (
                    args.run_duration is not None
                    and auto_started_at is not None
                    and now - auto_started_at >= args.run_duration
                ):
                    send_stop()
                    auto_enabled = False
                    auto_started_at = None
                    print(f"STOP: run duration reached ({args.run_duration:.1f}s)")
                elif stop_frames >= 3:
                    send_stop()
                    auto_enabled = False
                    auto_started_at = None
                    stop_latched = True
                    print("STOP: stop line detected")
                elif calculated_target is None:
                    if lane_missing_since is None:
                        lane_missing_since = now
                    elif now - lane_missing_since >= args.lane_lost_timeout:
                        send_stop()
                        auto_enabled = False
                        auto_started_at = None
                        print("STOP: both lanes lost")
                else:
                    lane_missing_since = None
                    if now - last_control_time >= 0.10:
                        filtered_steering = (
                            0.72 * filtered_steering + 0.28 * calculated_target
                        )
                        filtered_steering = max(
                            STEERING_MIN, min(STEERING_MAX, filtered_steering)
                        )
                        send(b"W" + steering_packet(round(filtered_steering)))
                        last_control_time = now

            display = draw_display(
                frame, left_roi, right_roi, left_line, right_line, left_x, right_x,
                left_reference_x, right_reference_x, lane_center_reference_x,
                args.reference_y, left_confidence, right_confidence, tracking_mode,
                stop_line, auto_enabled, stop_latched, args.dry_run,
                lateral_error_px, calculated_target, round(filtered_steering),
                display_fps,
            )
            cv2.imshow("Dual Lane Center Follow", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("k"):
                if control_left_x is None and control_right_x is None:
                    print("Calibration failed: no stable lane is visible")
                else:
                    if control_left_x is not None:
                        left_reference_normalized = control_left_x / frame_width
                    if control_right_x is not None:
                        right_reference_normalized = control_right_x / frame_width
                    if left_reference_normalized is not None and right_reference_normalized is None:
                        right_reference_normalized = (
                            2.0 * args.lane_center_x - left_reference_normalized
                        )
                    elif right_reference_normalized is not None and left_reference_normalized is None:
                        left_reference_normalized = (
                            2.0 * args.lane_center_x - right_reference_normalized
                        )
                    save_calibration(
                        CALIBRATION_FILE,
                        left_reference_normalized,
                        right_reference_normalized,
                    )
                    print(
                        "Lane calibration saved: "
                        f"left={left_reference_normalized:.4f}, "
                        f"right={right_reference_normalized:.4f}"
                    )
            elif key == ord("g"):
                if args.dry_run:
                    print("GO blocked in --dry-run mode")
                elif stop_latched:
                    print("GO blocked: press R after checking the stop line")
                elif calculated_target is None:
                    print("GO blocked: no usable lane/reference pair")
                else:
                    auto_enabled = True
                    auto_started_at = time.monotonic()
                    lane_missing_since = None
                    last_control_time = 0.0
                    print(
                        f"AUTO GO ({tracking_mode}): firmware W command "
                        "(current firmware: fixed 30% PWM); press X/Space to stop"
                    )
            elif key in (ord("x"), ord(" ")):
                send_stop()
                auto_enabled = False
                auto_started_at = None
                print("MANUAL STOP")
            elif key == ord("r"):
                send_stop()
                auto_enabled = False
                auto_started_at = None
                stop_latched = False
                stop_frames = 0
                print("Stop-line latch reset; vehicle remains stopped")

            if board is not None and board.in_waiting:
                text = board.read(board.in_waiting).decode("utf-8", errors="replace")
                print(text, end="")
                telemetry_buffer += text
                while "\n" in telemetry_buffer:
                    line, telemetry_buffer = telemetry_buffer.split("\n", 1)
                    speed_match = re.search(r"SPEED=(-?\d+)mm/s", line)
                    if speed_match is None or max_speed_mm_s is None:
                        continue
                    speed_mm_s = abs(int(speed_match.group(1)))
                    if auto_enabled and speed_mm_s > max_speed_mm_s:
                        send_stop()
                        auto_enabled = False
                        auto_started_at = None
                        print(
                            "STOP: speed limit exceeded "
                            f"({speed_mm_s * 0.0036:.2f} km/h > "
                            f"{args.max_speed_kmh:.2f} km/h)"
                        )

    except (serial.SerialException, RuntimeError, OSError) as error:
        print(f"Error: {error}")
        return 1
    except KeyboardInterrupt:
        print("Interrupted; STOP will be sent before exit")
        return 130
    finally:
        try_send_stop(board)
        if board is not None:
            time.sleep(0.1)
            try:
                board.close()
            except (serial.SerialException, OSError):
                pass
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
