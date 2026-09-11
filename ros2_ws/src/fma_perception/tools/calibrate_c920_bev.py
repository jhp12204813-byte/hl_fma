#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


SESSIONS = [
    (
        "20260911_211455",
        [
            (0.0, 2.0),
            (0.0, 3.0),
            (0.0, 4.0),
        ],
    ),
    (
        "20260911_211722",
        [
            (-1.0, 2.0),
            (-1.0, 3.0),
            (-1.0, 4.0),
        ],
    ),
    (
        "20260911_211856",
        [
            (+1.0, 2.0),
            (+1.0, 3.0),
            (+1.0, 4.0),
        ],
    ),
]


def find_video(session_dir):
    preferred = session_dir / "color.mp4"
    if preferred.exists():
        return preferred

    videos = sorted(session_dir.rglob("*.mp4"))
    if not videos:
        raise FileNotFoundError(f"No mp4 found in {session_dir}")

    return videos[0]


def put_lines(img, lines):
    y = 28
    for line in lines:
        cv2.putText(
            img,
            line,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 28


def read_frame(cap, index):
    cap.set(cv2.CAP_PROP_POS_FRAMES, index)
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f"Failed to read frame {index}")
    return frame


def choose_frame(video_path, session_name):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")

    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if count <= 0:
        cap.release()
        raise RuntimeError("Invalid frame count")

    index = count // 2
    window = f"C920 frame select - {session_name}"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    while True:
        frame = read_frame(cap, index)
        view = frame.copy()

        put_lines(
            view,
            [
                f"Session: {session_name}",
                f"Frame: {index}/{count-1}",
                "LEFT/RIGHT or A/D: -1/+1 frame",
                "P/N: -30/+30 frames",
                "ENTER: select frame    Q: quit",
                "Yellow tape-measure BODY is NOT a calibration point.",
            ],
        )

        cv2.imshow(window, view)
        key = cv2.waitKeyEx(0)

        if key in (13, 10):
            selected = frame.copy()
            break
        elif key in (ord("q"), ord("Q"), 27):
            cap.release()
            cv2.destroyWindow(window)
            raise KeyboardInterrupt
        elif key in (65361, 2424832, ord("a"), ord("A")):
            index = max(0, index - 1)
        elif key in (65363, 2555904, ord("d"), ord("D")):
            index = min(count - 1, index + 1)
        elif key in (ord("p"), ord("P")):
            index = max(0, index - 30)
        elif key in (ord("n"), ord("N")):
            index = min(count - 1, index + 30)

    cap.release()
    cv2.destroyWindow(window)
    return index, selected


def click_points(frame, session_name, ground_points):
    clicked = []
    window = f"C920 calibration - {session_name}"

    def mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(clicked) < len(ground_points):
                clicked.append((float(x), float(y)))

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, mouse)

    while True:
        view = frame.copy()

        for i, (u, v) in enumerate(clicked):
            gx, gy = ground_points[i]

            cv2.circle(
                view,
                (int(round(u)), int(round(v))),
                7,
                (0, 0, 255),
                -1,
            )

            cv2.putText(
                view,
                f"{i+1}: ({gx:+.1f},{gy:.1f})m",
                (int(u) + 10, int(v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if len(clicked) < len(ground_points):
            gx, gy = ground_points[len(clicked)]
            current = f"CLICK POST-IT BOTTOM CENTER: ({gx:+.1f}, {gy:.1f}) m"
        else:
            current = "ALL POINTS DONE - press ENTER"

        put_lines(
            view,
            [
                current,
                "Use ONLY the POST-IT bottom-center point.",
                "IGNORE the yellow tape-measure body at the far end.",
                "U: undo last point",
                "ENTER: accept after all points",
                "Q: quit",
            ],
        )

        cv2.imshow(window, view)
        key = cv2.waitKeyEx(20)

        if key in (ord("u"), ord("U")):
            if clicked:
                clicked.pop()

        elif key in (ord("q"), ord("Q"), 27):
            cv2.destroyWindow(window)
            raise KeyboardInterrupt

        elif key in (13, 10):
            if len(clicked) == len(ground_points):
                break

    cv2.destroyWindow(window)
    return clicked


def draw_debug(frame, image_points, ground_points):
    out = frame.copy()

    for i, ((u, v), (gx, gy)) in enumerate(
        zip(image_points, ground_points)
    ):
        p = (int(round(u)), int(round(v)))

        cv2.circle(out, p, 8, (0, 0, 255), -1)

        cv2.putText(
            out,
            f"{i+1} ({gx:+.1f},{gy:.1f})m",
            (p[0] + 10, p[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return out


def ground_to_bev_matrix(
    half_width_m=3.0,
    near_m=0.7,
    far_m=6.0,
    resolution_m=0.01,
):
    width = int(round(2.0 * half_width_m / resolution_m))
    height = int(round((far_m - near_m) / resolution_m))

    matrix = np.array(
        [
            [
                1.0 / resolution_m,
                0.0,
                width / 2.0,
            ],
            [
                0.0,
                -1.0 / resolution_m,
                far_m / resolution_m,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ],
        dtype=np.float64,
    )

    return matrix, width, height


def make_bev_preview(
    frame,
    h_image_to_ground,
    output_path,
    half_width_m=3.0,
    near_m=0.7,
    far_m=6.0,
    resolution_m=0.01,
):
    ground_to_bev, width, height = ground_to_bev_matrix(
        half_width_m,
        near_m,
        far_m,
        resolution_m,
    )

    h_image_to_bev = ground_to_bev @ h_image_to_ground

    bev = cv2.warpPerspective(
        frame,
        h_image_to_bev,
        (width, height),
        flags=cv2.INTER_LINEAR,
    )

    # x=0 center line
    center_x = width // 2
    cv2.line(
        bev,
        (center_x, 0),
        (center_x, height - 1),
        (255, 255, 255),
        1,
    )

    # 1 m spacing guides
    for y_m in range(1, 7):
        if not (near_m <= y_m <= far_m):
            continue

        py = int(round((far_m - y_m) / resolution_m))

        cv2.line(
            bev,
            (0, py),
            (width - 1, py),
            (100, 100, 100),
            1,
        )

        cv2.putText(
            bev,
            f"{y_m}m",
            (8, max(18, py - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # Calibration data ends at y=4 m.
    # Everything farther than 4 m is extrapolation.
    y4_px = int(round((far_m - 4.0) / resolution_m))

    if y4_px > 0:
        overlay = bev.copy()
        cv2.rectangle(
            overlay,
            (0, 0),
            (width - 1, y4_px),
            (0, 0, 120),
            -1,
        )

        bev = cv2.addWeighted(
            overlay,
            0.25,
            bev,
            0.75,
            0,
        )

        cv2.putText(
            bev,
            "EXTRAPOLATED: calibration points stop at 4m",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imwrite(str(output_path), bev)
    return h_image_to_bev


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--root",
        type=Path,
        default=Path("~/c920_recordings"),
    )

    ap.add_argument(
        "--output",
        type=Path,
        default=Path("~/c920_bev_calibration"),
    )

    ap.add_argument(
        "--config",
        type=Path,
        default=Path("config/c920_bev_calibration.yaml"),
    )

    ap.add_argument(
        "--ransac-threshold-m",
        type=float,
        default=0.05,
    )

    args = ap.parse_args()

    root = args.root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    config_path = args.config.expanduser()

    output.mkdir(parents=True, exist_ok=True)
    config_path.parent.mkdir(parents=True, exist_ok=True)

    all_image = []
    all_ground = []
    records = []

    image_width = None
    image_height = None

    selected_preview_frame = None

    for session_name, ground_points in SESSIONS:
        session_dir = root / session_name
        video_path = find_video(session_dir)

        print()
        print("SESSION:", session_name)
        print("VIDEO:", video_path)

        frame_index, frame = choose_frame(
            video_path,
            session_name,
        )

        h, w = frame.shape[:2]

        if image_width is None:
            image_width = w
            image_height = h
        elif (w, h) != (image_width, image_height):
            raise RuntimeError(
                "Video resolutions differ between sessions: "
                f"{w}x{h} vs {image_width}x{image_height}"
            )

        if selected_preview_frame is None:
            selected_preview_frame = frame.copy()

        points = click_points(
            frame,
            session_name,
            ground_points,
        )

        debug = draw_debug(
            frame,
            points,
            ground_points,
        )

        cv2.imwrite(
            str(
                output
                / f"debug_{session_name}_frame_{frame_index}.png"
            ),
            debug,
        )

        for uv, xy in zip(points, ground_points):
            all_image.append(uv)
            all_ground.append(xy)

            records.append(
                {
                    "session": session_name,
                    "frame_index": int(frame_index),
                    "image_uv_px": [
                        float(uv[0]),
                        float(uv[1]),
                    ],
                    "ground_xy_m": [
                        float(xy[0]),
                        float(xy[1]),
                    ],
                }
            )

    image_pts = np.asarray(
        all_image,
        dtype=np.float64,
    )

    ground_pts = np.asarray(
        all_ground,
        dtype=np.float64,
    )

    H, inliers = cv2.findHomography(
        image_pts,
        ground_pts,
        method=cv2.RANSAC,
        ransacReprojThreshold=args.ransac_threshold_m,
    )

    if H is None:
        raise RuntimeError("findHomography failed")

    H_inv = np.linalg.inv(H)

    projected = cv2.perspectiveTransform(
        image_pts.reshape(-1, 1, 2),
        H,
    ).reshape(-1, 2)

    errors = np.linalg.norm(
        projected - ground_pts,
        axis=1,
    )

    inlier_flags = (
        inliers.reshape(-1).astype(bool)
        if inliers is not None
        else np.ones(len(errors), dtype=bool)
    )

    for i, record in enumerate(records):
        record["reprojection_xy_m"] = [
            float(projected[i, 0]),
            float(projected[i, 1]),
        ]

        record["error_m"] = float(errors[i])
        record["ransac_inlier"] = bool(inlier_flags[i])

    print()
    print("=== REPROJECTION ERRORS ===")

    for i, record in enumerate(records):
        xy = record["ground_xy_m"]

        print(
            f"{i+1:02d}",
            record["session"],
            f"ground=({xy[0]:+.2f},{xy[1]:.2f})",
            f"error={record['error_m']*100:.2f} cm",
            "INLIER" if record["ransac_inlier"] else "OUTLIER",
        )

    mean_error = float(np.mean(errors))
    median_error = float(np.median(errors))
    max_error = float(np.max(errors))

    print()
    print("=== ERROR SUMMARY ===")
    print(f"mean   : {mean_error*100:.2f} cm")
    print(f"median : {median_error*100:.2f} cm")
    print(f"max    : {max_error*100:.2f} cm")
    print(
        "inliers:",
        int(inlier_flags.sum()),
        "/",
        len(inlier_flags),
    )

    bev_meta = {
        "half_width_m": 3.0,
        "near_m": 0.7,
        "far_m": 6.0,
        "resolution_m": 0.01,
        "calibrated_forward_max_m": 4.0,
    }

    ground_to_bev, bev_width, bev_height = ground_to_bev_matrix(
        bev_meta["half_width_m"],
        bev_meta["near_m"],
        bev_meta["far_m"],
        bev_meta["resolution_m"],
    )

    h_image_to_bev = make_bev_preview(
        selected_preview_frame,
        H,
        output / "bev_preview.png",
        bev_meta["half_width_m"],
        bev_meta["near_m"],
        bev_meta["far_m"],
        bev_meta["resolution_m"],
    )

    config = {
        "camera_name": "C920",
        "image_width": int(image_width),
        "image_height": int(image_height),
        "coordinate_convention": {
            "x": "vehicle lateral, positive right",
            "y": "vehicle forward, positive forward",
            "unit": "meter",
            "origin": "vehicle center reference",
        },
        "homography_image_to_ground": H.tolist(),
        "homography_ground_to_image": H_inv.tolist(),
        "homography_ground_to_bev": ground_to_bev.tolist(),
        "homography_image_to_bev": h_image_to_bev.tolist(),
        "bev": {
            **bev_meta,
            "width_px": int(bev_width),
            "height_px": int(bev_height),
        },
        "reprojection_error_m": {
            "mean": mean_error,
            "median": median_error,
            "max": max_error,
        },
        "calibration_points": records,
    }

    with config_path.open(
        "w",
        encoding="utf-8",
    ) as f:
        yaml.safe_dump(
            config,
            f,
            sort_keys=False,
            allow_unicode=True,
        )

    output_config = output / "c920_bev_calibration.yaml"

    with output_config.open(
        "w",
        encoding="utf-8",
    ) as f:
        yaml.safe_dump(
            config,
            f,
            sort_keys=False,
            allow_unicode=True,
        )

    print()
    print("=== SAVED ===")
    print("config :", config_path)
    print("copy   :", output_config)
    print("preview:", output / "bev_preview.png")
    print()
    print("Open bev_preview.png and check that road geometry")
    print("looks reasonable and the vehicle center is centered.")


if __name__ == "__main__":
    main()
