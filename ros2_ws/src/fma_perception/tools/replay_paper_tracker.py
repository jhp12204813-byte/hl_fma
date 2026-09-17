"""Evaluate paper-style tracker on saved D435i RGB+depth sessions."""
import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from fma_perception.ground_plane import estimate
from fma_perception.ground_tracker import GroundConfig, raster
from fma_perception.paper_lane_tracker import PaperLaneTracker


def project_image_mask_to_bev(image_mask, camera, plane, cfg):
    """Project an image-space exclusion mask onto the same metric BEV as raster()."""
    h, w = image_mask.shape[:2]

    n = np.asarray(plane["normal"], dtype=float)
    d = plane["offset_m"]

    forward = np.array([0.0, 0.0, 1.0]) - n * n[2]
    forward /= np.linalg.norm(forward)

    right = np.cross(n, forward)
    right /= np.linalg.norm(right)

    origin = -d * n

    width = int(round(2 * cfg.half_width_m / cfg.resolution_m))
    height = int(round((cfg.far_m - cfg.near_m) / cfg.resolution_m))

    by, bx = np.mgrid[:height, :width]
    x = (bx - width / 2) * cfg.resolution_m
    y = cfg.far_m - by * cfg.resolution_m

    xyz = origin + x[:, :, None] * right + y[:, :, None] * forward

    uv, _ = cv2.projectPoints(
        xyz.reshape(-1, 3),
        np.zeros(3),
        np.zeros(3),
        np.asarray(camera["k"]).reshape(3, 3),
        np.asarray(camera["d"]),
    )
    uv = uv.reshape(height, width, 2).astype(np.float32)

    visible = (
        (xyz[:, :, 2] > 0.1)
        & (uv[:, :, 0] >= 0)
        & (uv[:, :, 0] <= w - 1)
        & (uv[:, :, 1] >= int(h * cfg.roi_top))
        & (uv[:, :, 1] <= h - 1)
    )

    bev = cv2.remap(
        image_mask,
        uv[:, :, 0],
        uv[:, :, 1],
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
    )
    bev[~visible] = 0
    return bev


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root", type=Path, default=Path.home() / "d435i_recordings")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--frames-per-block", type=int, default=10)
    p.add_argument("--far-m", type=float, default=4.0)
    p.add_argument("--session", type=str, default=None)
    p.add_argument("--lane-colors", choices=["yellow","white","yellow_white"], default="yellow")
    p.add_argument("--road-mark-model", type=Path, default=None)
    p.add_argument("--road-mark-conf", type=float, default=0.15)
    p.add_argument("--road-mark-iou", type=float, default=0.30)
    a = p.parse_args()

    a.output.mkdir(parents=True, exist_ok=False)
    cfg = GroundConfig(far_m=a.far_m)

    road_mark_model = None
    if a.road_mark_model is not None:
        from ultralytics import YOLO
        road_mark_model = YOLO(str(a.road_mark_model))

    all_rows = []
    total_states = Counter()
    session_summaries = []

    for video in sorted(a.root.glob("*/color.mp4")):
        if a.session and video.parent.name != a.session:
            continue
        root = video.parent
        session = json.loads((root / "session.json").read_text())

        cam = session["camera_info"]["color"]
        depth_info = session["camera_info"]["depth"]

        if (
            cam["header"]["frame_id"] != depth_info["header"]["frame_id"]
            or not np.allclose(cam["k"], depth_info["k"])
        ):
            raise ValueError("Misaligned CameraInfo")

        frames = [json.loads(x) for x in (root / "frames.jsonl").open()]
        depths = [json.loads(x) for x in (root / "depth_frames.jsonl").open()]
        depth_stamps = np.array([x["timestamp_ns"] for x in depths], dtype=np.int64)

        cap = cv2.VideoCapture(str(video))
        writer = cv2.VideoWriter(
            str(a.output / (root.name + "_debug.mp4")),
            cv2.VideoWriter_fourcc(*"mp4v"),
            30,
            (1240, 480),
        )
        if not writer.isOpened():
            raise RuntimeError("debug video writer open failed")
        session_states = Counter()
        evaluated = 0

        try:
            for start in [
                len(frames) // 4,
                len(frames) // 2,
                3 * len(frames) // 4,
            ]:
                tracker = PaperLaneTracker()
                cap.set(cv2.CAP_PROP_POS_FRAMES, start)

                for index in range(
                    start,
                    min(start + a.frames_per_block, len(frames))
                ):
                    ok, frame = cap.read()
                    if not ok:
                        break

                    stamp = frames[index]["timestamp_ns"]
                    j = int(np.argmin(np.abs(depth_stamps - stamp)))
                    delta = abs(int(depth_stamps[j]) - stamp)

                    if delta > 20_000_000:
                        continue

                    depth = cv2.imread(str(root / depths[j]["filename"]), -1)
                    if depth is None:
                        continue

                    try:
                        plane, _, _ = estimate(
                            depth,
                            cam,
                            depths[j]["depth_scale_m"],
                            max_depth_m=cfg.far_m,
                        )

                        if not plane["accepted"]:
                            state = "PLANE_REJECTED"
                            result = {}
                        else:
                            mask, bev_debug, _, _ = raster(
                                frame,
                                depth,
                                cam,
                                depths[j]["depth_scale_m"],
                                plane,
                                cfg,
                                return_visibility=True,
                                lane_colors=a.lane_colors,
                            )

                            lane_mask = mask
                            road_marking_boxes = 0
                            removed_pixels = 0

                            if road_mark_model is not None:
                                prediction = road_mark_model.predict(
                                    frame,
                                    conf=a.road_mark_conf,
                                    iou=a.road_mark_iou,
                                    device=0,
                                    verbose=False,
                                )[0]

                                image_exclusion = np.zeros(
                                    frame.shape[:2], dtype=np.uint8
                                )

                                if prediction.boxes is not None:
                                    boxes = prediction.boxes.xyxy.cpu().numpy()
                                    road_marking_boxes = len(boxes)

                                    for x1, y1, x2, y2 in boxes:
                                        x1 = max(0, min(frame.shape[1] - 1, int(x1)))
                                        x2 = max(0, min(frame.shape[1] - 1, int(x2)))
                                        y1 = max(0, min(frame.shape[0] - 1, int(y1)))
                                        y2 = max(0, min(frame.shape[0] - 1, int(y2)))

                                        if x2 > x1 and y2 > y1:
                                            cv2.rectangle(
                                                image_exclusion,
                                                (x1, y1),
                                                (x2, y2),
                                                255,
                                                -1,
                                            )

                                if road_marking_boxes:
                                    exclusion_bev = project_image_mask_to_bev(
                                        image_exclusion, cam, plane, cfg
                                    )

                                    lane_mask = mask.copy()
                                    removed = (
                                        (lane_mask > 0) & (exclusion_bev > 0)
                                    )
                                    removed_pixels = int(np.count_nonzero(removed))
                                    lane_mask[exclusion_bev > 0] = 0

                                    # Red pixels in debug = lane pixels removed only
                                    # from the tracker copy, not from the raw mask.
                                    bev_debug[removed] = (0, 0, 255)

                            result = tracker.process(lane_mask)
                            result["road_marking_boxes"] = road_marking_boxes
                            result["road_marking_removed_pixels"] = removed_pixels
                            state = result["state"]
                            if state == "BOTH_VISUAL":
                                state = result.get("pair_state", "PAIR_WEAK")

                            # Paper tracker visualization on metric BEV.
                            for key, color in (
                                ("left", (0,255,255)),
                                ("right", (255,255,0)),
                            ):
                                lane = result.get(key)
                                if lane is not None:
                                    ys = np.arange(lane["y_min"], lane["y_max"] + 1)
                                    xs = np.polyval(lane["coefficients"], ys)
                                    pts = np.c_[
                                        np.clip(xs, 0, bev_debug.shape[1]-1),
                                        ys
                                    ].astype(np.int32)
                                    cv2.polylines(bev_debug, [pts], False, color, 2)

                            center = result.get("center")
                            if center is not None:
                                ys = np.arange(center["y_min"], center["y_max"] + 1)
                                xs = np.polyval(center["coefficients"], ys)
                                pts = np.c_[
                                    np.clip(xs, 0, bev_debug.shape[1]-1),
                                    ys
                                ].astype(np.int32)
                                cv2.polylines(
                                    bev_debug, [pts], False, (0,255,0), 2
                                )

                    except ValueError:
                        state = "ERROR"
                        result = {}
                        tracker.reset()

                    session_states[state] += 1
                    total_states[state] += 1
                    evaluated += 1

                    row = {
                        "session": root.name,
                        "index": index,
                        "state": state,
                    }

                    if result.get("pair_quality") is not None:
                        row["pair_quality"] = result["pair_quality"]

                    if result.get("center") is not None:
                        row["center_coefficients"] = [
                            float(v) for v in result["center"]["coefficients"]
                        ]
                        row["center_y_min"] = int(result["center"]["y_min"])
                        row["center_y_max"] = int(result["center"]["y_max"])

                    if result.get("left") is not None:
                        row["left_points"] = result["left"]["points"]
                        row["left_residual_px"] = result["left"]["residual_px"]
                        row["left_y_min"] = result["left"]["y_min"]
                        row["left_y_max"] = result["left"]["y_max"]

                    if result.get("right") is not None:
                        row["right_points"] = result["right"]["points"]
                        row["right_residual_px"] = result["right"]["residual_px"]
                        row["right_y_min"] = result["right"]["y_min"]
                        row["right_y_max"] = result["right"]["y_max"]

                    row["road_marking_boxes"] = int(
                        result.get("road_marking_boxes", 0)
                    )
                    row["road_marking_removed_pixels"] = int(
                        result.get("road_marking_removed_pixels", 0)
                    )
                    row["joint_refit_attempted"] = bool(
                        result.get("joint_refit_attempted", False)
                    )
                    row["joint_refit_used"] = bool(
                        result.get("joint_refit_used", False)
                    )
                    row["joint_refit_candidate_reason"] = result.get(
                        "joint_refit_candidate_reason"
                    )
                    row["joint_candidate_left_range"] = result.get(
                        "joint_candidate_left_range"
                    )
                    row["joint_candidate_right_range"] = result.get(
                        "joint_candidate_right_range"
                    )
                    row["joint_candidate_left_points"] = result.get(
                        "joint_candidate_left_points"
                    )
                    row["joint_candidate_right_points"] = result.get(
                        "joint_candidate_right_points"
                    )

                    all_rows.append(row)

                    overlay = frame.copy()
                    cv2.putText(
                        overlay,
                        f"{root.name} frame={index} state={state}",
                        (10,25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0,255,255),
                        1,
                    )

                    if 'bev_debug' not in locals() or bev_debug is None:
                        bev_debug = np.zeros((330,600,3), np.uint8)

                    canvas = np.zeros((480,1240,3), np.uint8)
                    canvas[:480,:640] = overlay
                    bh,bw = bev_debug.shape[:2]
                    canvas[:min(480,bh),640:640+min(600,bw)] = \
                        bev_debug[:min(480,bh),:min(600,bw)]

                    writer.write(canvas)
                    bev_debug = None

        finally:
            cap.release()
            writer.release()

        session_summaries.append({
            "session": root.name,
            "frames": evaluated,
            "NONE": session_states["NONE"],
            "LEFT": session_states["LEFT_ONLY"],
            "RIGHT": session_states["RIGHT_ONLY"],
            "PAIR_VALID": session_states["PAIR_VALID"],
            "PAIR_WEAK": session_states["PAIR_WEAK"],
        })

    print("\n=== TOTAL ===")
    print(dict(total_states))

    print("\n=== SESSION RESULTS ===")
    for r in session_summaries:
        print(
            r["session"],
            "frames", r["frames"],
            "NONE", r["NONE"],
            "LEFT", r["LEFT"],
            "RIGHT", r["RIGHT"],
            "PAIR_VALID", r["PAIR_VALID"],
            "PAIR_WEAK", r["PAIR_WEAK"],
        )

    (a.output / "results.json").write_text(
        json.dumps(all_rows, indent=2)
    )

    (a.output / "metrics.json").write_text(
        json.dumps(dict(total_states), indent=2)
    )


if __name__ == "__main__":
    main()
