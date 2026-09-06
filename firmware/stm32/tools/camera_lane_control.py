#!/usr/bin/env python3
"""USB-camera lane detection plus keyboard control for NUCLEO-F401RE."""

import argparse
import time

import cv2
import numpy as np
import serial


MANUAL_COMMANDS = {
    ord("w"): b"W",
    ord("s"): b"S",
    ord("a"): b"A",
    ord("d"): b"D",
    ord("c"): b"C",
    ord("x"): b"X",
    ord("0"): b"X",
    ord(" "): b"X",
    ord("p"): b"P",
}


def weighted_line(lines: list[tuple[float, float, float]]) -> tuple[float, float] | None:
    if not lines:
        return None
    total_weight = sum(item[2] for item in lines)
    slope = sum(item[0] * item[2] for item in lines) / total_weight
    intercept = sum(item[1] * item[2] for item in lines) / total_weight
    return slope, intercept


def line_points(
    line: tuple[float, float] | None, y_bottom: int, y_top: int
) -> tuple[int, int, int, int] | None:
    if line is None:
        return None
    slope, intercept = line
    if abs(slope) < 0.01:
        return None
    x_bottom = int((y_bottom - intercept) / slope)
    x_top = int((y_top - intercept) / slope)
    return x_bottom, y_bottom, x_top, y_top


def detect_lanes(frame: np.ndarray):
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    white = cv2.inRange(hsv, np.array([0, 0, 155]), np.array([180, 85, 255]))
    yellow = cv2.inRange(hsv, np.array([12, 65, 80]), np.array([42, 255, 255]))
    color_mask = cv2.bitwise_or(white, yellow)
    color_mask = cv2.GaussianBlur(color_mask, (5, 5), 0)
    edges = cv2.Canny(color_mask, 60, 160)

    roi = np.zeros_like(edges)
    polygon = np.array(
        [[
            (int(width * 0.05), height - 1),
            (int(width * 0.40), int(height * 0.55)),
            (int(width * 0.60), int(height * 0.55)),
            (int(width * 0.95), height - 1),
        ]],
        dtype=np.int32,
    )
    cv2.fillPoly(roi, polygon, 255)
    cropped = cv2.bitwise_and(edges, roi)

    segments = cv2.HoughLinesP(
        cropped,
        1,
        np.pi / 180,
        threshold=35,
        minLineLength=max(25, width // 16),
        maxLineGap=max(30, width // 12),
    )

    left_candidates: list[tuple[float, float, float]] = []
    right_candidates: list[tuple[float, float, float]] = []
    if segments is not None:
        for segment in segments[:, 0]:
            x1, y1, x2, y2 = (int(value) for value in segment)
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.35 or abs(slope) > 4.0:
                continue
            intercept = y1 - slope * x1
            length = float(np.hypot(x2 - x1, y2 - y1))
            mean_x = (x1 + x2) * 0.5
            if slope < 0 and mean_x < width * 0.62:
                left_candidates.append((slope, intercept, length))
            elif slope > 0 and mean_x > width * 0.38:
                right_candidates.append((slope, intercept, length))

    left = weighted_line(left_candidates)
    right = weighted_line(right_candidates)
    y_bottom = height - 1
    y_top = int(height * 0.58)
    left_points = line_points(left, y_bottom, y_top)
    right_points = line_points(right, y_bottom, y_top)

    lookahead_y = int(height * 0.72)
    lane_center = None
    assumed_half_width = width * 0.22
    if left is not None and right is not None:
        left_x = (lookahead_y - left[1]) / left[0]
        right_x = (lookahead_y - right[1]) / right[0]
        if width * 0.12 < right_x - left_x < width * 0.9:
            lane_center = (left_x + right_x) * 0.5
    elif left is not None:
        lane_center = (lookahead_y - left[1]) / left[0] + assumed_half_width
    elif right is not None:
        lane_center = (lookahead_y - right[1]) / right[0] - assumed_half_width

    offset = None
    if lane_center is not None and -width * 0.25 < lane_center < width * 1.25:
        offset = (lane_center - width * 0.5) / (width * 0.5)

    return cropped, left_points, right_points, lane_center, lookahead_y, offset


def draw_result(frame, left, right, lane_center, lookahead_y, offset, assist):
    overlay = frame.copy()
    for line in (left, right):
        if line is not None:
            cv2.line(overlay, line[:2], line[2:], (0, 255, 0), 7)

    height, width = frame.shape[:2]
    cv2.line(
        overlay,
        (width // 2, height - 1),
        (width // 2, int(height * 0.58)),
        (255, 0, 0),
        2,
    )
    if lane_center is not None:
        cv2.circle(overlay, (int(lane_center), lookahead_y), 9, (0, 0, 255), -1)

    status = "LANE LOST" if offset is None else f"offset={offset:+.2f}"
    mode = "ASSIST ON" if assist else "MANUAL"
    color = (0, 255, 0) if offset is not None else (0, 0, 255)
    cv2.putText(overlay, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(overlay, mode, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(
        overlay,
        "WASD drive | L assist | X stop | Q quit",
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    return cv2.addWeighted(frame, 0.35, overlay, 0.65, 0)


def camera_source(value: str):
    return int(value) if value.isdigit() else value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--camera", default="2", help="camera index or video path")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    board = None
    camera = None
    assist = False
    last_assist_command = 0.0

    try:
        board = serial.Serial(args.port, 115200, timeout=0)
        board.reset_input_buffer()
        board.write(b"X")

        camera = cv2.VideoCapture(camera_source(args.camera))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not camera.isOpened():
            raise RuntimeError(f"Cannot open camera {args.camera}")

        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera frame read failed")

            _, left, right, lane_center, lookahead_y, offset = detect_lanes(frame)
            display = draw_result(
                frame, left, right, lane_center, lookahead_y, offset, assist
            )
            cv2.imshow("NUCLEO Lane Control", display)

            now = time.monotonic()
            if assist and now - last_assist_command >= 0.12:
                if offset is None:
                    board.write(b"H")
                elif offset < -0.08:
                    board.write(b"A")
                elif offset > 0.08:
                    board.write(b"D")
                else:
                    board.write(b"C")
                last_assist_command = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("l"):
                assist = not assist
                board.write(b"H")
            elif key in MANUAL_COMMANDS:
                command = MANUAL_COMMANDS[key]
                if command in (b"A", b"D", b"C"):
                    assist = False
                board.write(command)

            if board.in_waiting:
                print(board.read(board.in_waiting).decode("utf-8", errors="replace"), end="")

    except (serial.SerialException, RuntimeError) as error:
        print(f"Error: {error}")
        return 1
    finally:
        if board is not None and board.is_open:
            board.write(b"X")
            time.sleep(0.1)
            board.close()
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
