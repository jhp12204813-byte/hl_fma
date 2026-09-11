#!/usr/bin/env python3

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fma_perception.c920_bev import C920BEV
from fma_perception.paper_lane_tracker import PaperLaneTracker


def find_video(session_dir):
    p = session_dir / "color.mp4"
    if p.exists():
        return p

    vids = sorted(session_dir.rglob("*.mp4"))
    if not vids:
        raise FileNotFoundError(session_dir)

    return vids[0]


def make_mask(frame, bev):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Same starting thresholds as D435i lane pipeline.
    yellow = cv2.inRange(
        hsv,
        (5, 25, 60),
        (40, 255, 255),
    )

    white = cv2.inRange(
        hsv,
        (0, 0, 170),
        (179, 65, 255),
    )

    # C920 lane A/B: yellow-only.
    # Avoid crosswalk/curb/bright pavement entering the lane tracker.
    src_mask = yellow

    # Binary/categorical mask -> nearest-neighbor warp.
    bev_mask = cv2.warpPerspective(
        src_mask,
        bev.H,
        (bev.width, bev.height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )

    return src_mask, bev_mask


def draw_fit(img, fit, color):
    if fit is None:
        return

    y0 = int(fit["y_min"])
    y1 = int(fit["y_max"])

    if y1 <= y0:
        return

    ys = np.linspace(y0, y1, 120)
    xs = np.polyval(fit["coefficients"], ys)

    pts = np.c_[xs, ys]

    ok = (
        (pts[:, 0] >= 0)
        & (pts[:, 0] < img.shape[1])
        & (pts[:, 1] >= 0)
        & (pts[:, 1] < img.shape[0])
    )

    pts = pts[ok].astype(np.int32)

    if len(pts) >= 2:
        cv2.polylines(
            img,
            [pts.reshape(-1, 1, 2)],
            False,
            color,
            2,
            cv2.LINE_AA,
        )


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--root",
        type=Path,
        default=Path("~/c920_recordings"),
    )

    ap.add_argument(
        "--config",
        type=Path,
        default=Path("config/c920_bev_calibration.yaml"),
    )

    ap.add_argument(
        "--output",
        type=Path,
        default=Path("~/c920_lane_replay"),
    )

    ap.add_argument(
        "--session",
        action="append",
        default=None,
        help="May be repeated. Default: all session folders.",
    )

    args = ap.parse_args()

    root = args.root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    bev = C920BEV(args.config)

    if args.session:
        sessions = [root / x for x in args.session]
    else:
        sessions = sorted(
            p for p in root.iterdir()
            if p.is_dir()
        )

    total = Counter()
    all_rows = []

    for session_dir in sessions:
        video = find_video(session_dir)

        print()
        print("SESSION:", session_dir.name)
        print("VIDEO:", video)

        cap = cv2.VideoCapture(str(video))

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 30.0

        tracker = PaperLaneTracker()
        counts = Counter()

        out_video = output / f"{session_dir.name}_lane.mp4"

        # Original 640x480 + BEV 600x530 currently.
        canvas_w = bev.image_width + bev.width
        canvas_h = max(bev.image_height, bev.height)

        writer = cv2.VideoWriter(
            str(out_video),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (canvas_w, canvas_h),
        )

        index = 0

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if (
                frame.shape[1] != bev.image_width
                or frame.shape[0] != bev.image_height
            ):
                raise RuntimeError(
                    f"Unexpected frame size: "
                    f"{frame.shape[1]}x{frame.shape[0]}"
                )

            _, mask = make_mask(frame, bev)

            result = tracker.process(mask)

            state = result.get(
                "pair_state",
                result["state"],
            )

            counts[state] += 1
            total[state] += 1

            row = {
                "session": session_dir.name,
                "index": index,
                "state": state,
                "horizontal_suppressed_pixels": int(
                    result.get(
                        "horizontal_suppressed_pixels",
                        0,
                    )
                ),
            }

            if result.get("pair_quality") is not None:
                row["pair_quality"] = result["pair_quality"]

            if result.get("center") is not None:
                row["center_coefficients"] = [
                    float(x)
                    for x in result["center"]["coefficients"]
                ]
                row["center_y_min"] = int(
                    result["center"]["y_min"]
                )
                row["center_y_max"] = int(
                    result["center"]["y_max"]
                )

            all_rows.append(row)

            # D435i replay-style BEV debug.
            bev_debug = np.full(
                (bev.height, bev.width, 3),
                25,
                dtype=np.uint8,
            )

            # Detected yellow/white lane pixels.
            bev_debug[mask > 0] = (235, 235, 235)

            # left = cyan, right = yellow, center = green
            draw_fit(
                bev_debug,
                result.get("left"),
                (255, 255, 0),
            )

            draw_fit(
                bev_debug,
                result.get("right"),
                (0, 255, 255),
            )

            draw_fit(
                bev_debug,
                result.get("center"),
                (0, 255, 0),
            )

            # Vehicle center x=0.
            cv2.line(
                bev_debug,
                (bev.width // 2, 0),
                (bev.width // 2, bev.height - 1),
                (0, 180, 180),
                1,
            )

            cv2.putText(
                bev_debug,
                state,
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            overlay = frame.copy()

            cv2.putText(
                overlay,
                f"frame={index}  {state}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            canvas = np.zeros(
                (canvas_h, canvas_w, 3),
                dtype=np.uint8,
            )

            canvas[
                :bev.image_height,
                :bev.image_width,
            ] = overlay

            canvas[
                :bev.height,
                bev.image_width:
                bev.image_width + bev.width,
            ] = bev_debug

            writer.write(canvas)
            index += 1

        cap.release()
        writer.release()

        print(
            "frames", index,
            dict(counts),
        )

    results_path = output / "results.json"

    results_path.write_text(
        json.dumps(
            all_rows,
            indent=2,
            ensure_ascii=False,
        )
    )

    print()
    print("=== TOTAL ===")
    print(dict(total))
    print("saved:", output)


if __name__ == "__main__":
    main()
