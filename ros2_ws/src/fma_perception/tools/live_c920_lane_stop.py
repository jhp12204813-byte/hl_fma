#!/usr/bin/env python3
"""Single C920 live visualization using the existing replay pipeline only.

No ROS, serial, vehicle commands, or camera access on import.
Recording stores the displayed composite at the requested FPS (no timestamps).
"""
import argparse
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from replay_c920_lane import (
    C920BEV, PaperLaneTracker, StopLineDetector, StopLineTracker,
    draw_fit, make_mask,
)

REPO = Path(__file__).resolve().parents[4]
WINDOW = 'C920 lane + stop diagnostics'


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30.0)
    # Same calibration file as replay, resolved relative to repository root.
    ap.add_argument('--config', type=Path,
                    default=REPO / 'config/c920_bev_calibration.yaml')
    ap.add_argument('--record', type=Path, help='Record the displayed composite MP4')
    return ap


def annotate(frame, bev, mask, lane, stop, tracked, fps):
    """Overlay only: never alter masks or tracker outputs."""
    debug = bev.warp_to_bev(frame)
    debug[stop['debug_mask'] > 0] = (220, 140, 220)
    debug[mask > 0] = (0, 220, 220)
    contours, _ = cv2.findContours(stop['corridor_mask'].copy(),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(debug, contours, -1, (150, 100, 0), 1)
    for name, color in [('left', (255, 255, 0)), ('right', (0, 255, 255)),
                        ('center', (0, 255, 0)), ('temporary_center', (255, 0, 255))]:
        draw_fit(debug, lane.get(name), color)
    for candidate in stop['candidates']:
        x, y = candidate['x'], candidate['y']
        w, h = candidate['width'], candidate['height']
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 165, 255), 2)
    # Draw the existing detector's reported distance; no new candidate selection.
    if stop['stop_line_detected']:
        _, v = bev.ground_to_bev_pixel(0, stop['stop_line_distance_m'])
        v = int(round(v))
        if 0 <= v < bev.height:
            xs = np.flatnonzero(stop['corridor_mask'][v])
            if len(xs):
                endpoints = [(int(xs[0]), v), (int(xs[-1]), v)]
                cv2.line(debug, *endpoints, (0, 0, 255), 3)
                for point in endpoints:
                    cv2.circle(debug, point, 4, (0, 0, 255), -1)
    quality = lane.get('pair_quality') or {}
    valid = bool(quality.get('valid', False))
    state = lane.get('pair_state', lane['state'])
    distance = stop['stop_line_distance_m']
    distance_text = '--' if distance is None else f'{distance:.2f}'
    # Match the already selected detector result; do not select a new stop line.
    selected = next((candidate for candidate in stop['candidates']
                     if candidate['distance_m'] == distance), None) if stop['stop_line_detected'] else None
    thickness = None if selected is None else selected['height'] * bev.resolution_m
    near_edge = None if thickness is None or distance is None else distance - thickness / 2.0
    thickness_text = '--' if thickness is None else f'{thickness:.2f}'
    near_edge_text = '--' if near_edge is None else f'{near_edge:.2f}'
    held_distance = tracked['tracked_stop_distance_m'] if tracked['stop_tracked'] else None
    held_text = '--' if held_distance is None else f'{held_distance:.2f}m'
    lines = [
        f"LANE: {'VALID' if valid else 'INVALID'}  {state}",
        f"CENTER: {lane.get('center_source', 'NONE')}",
        f"PAIR: {quality.get('reason', 'unavailable')}",
        f"STOP: {'YES' if stop['stop_line_detected'] else 'NO'}",
        f'STOP DIST: {distance_text} m',
        f"STOP CONF: {stop['stop_line_confidence']:.2f}",
        f"TRACK: {'YES' if tracked['stop_tracked'] else 'NO'} {held_text} "
        f"miss={tracked['stop_missed_count']}",
        f"CROSSWALK: {stop['crosswalk_detected']}  candidates={stop['candidate_count']}",
        f'FPS: {fps:.1f}',
        'q/ESC: quit  r: reset',
        f'STOP CENTER: {distance_text} m',
        f'STOP NEAR EDGE: {near_edge_text} m',
        f'STOP THICKNESS: {thickness_text} m',
    ]
    # Separate footer keeps diagnostics from hiding camera or BEV observations.
    width = bev.image_width + bev.width
    top_height = max(bev.image_height, bev.height)
    canvas = np.zeros((top_height + 240, width, 3), dtype=np.uint8)
    canvas[:bev.image_height, :bev.image_width] = frame
    canvas[:bev.height, bev.image_width:] = debug
    for i, line in enumerate(lines):
        column, row = divmod(i, 7)
        cv2.putText(canvas, line, (12 + column * (width // 2), top_height + 30 + row * 29),
                    cv2.FONT_HERSHEY_SIMPLEX, .52, (235, 235, 235), 1, cv2.LINE_AA)
    return canvas, ' | '.join(lines[i] for i in (0, 3, 4, 5, 8, 10, 11, 12))


def rejection_debug(bev, tracker, detector, lane, stop, white_bev):
    """Read-only diagnostic replay of row gates; never feed back into detection.

    The existing stop detector selects ROW BANDS, not connected components.
    Component geometry below is descriptive only, not an angle/aspect gate.
    """
    quality = lane.get('pair_quality') or {}
    measured = quality.get('median_width_px')
    width = '--' if measured is None else f'{measured:.1f}px/{measured * bev.resolution_m:.2f}m'
    minimum = tracker.cfg.min_pair_width_px
    lines = [f"LANE DEBUG width={width} allowed_min={minimum:.1f}px/"
             f"{minimum * bev.resolution_m:.2f}m allowed_max=NONE "
             f"reason={quality.get('reason', 'pair_unavailable')}"]
    cfg = detector.config
    raw, corridor = stop['debug_mask'], stop['corridor_mask']
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (
        max(1, int(round(cfg.continuity_close_m / bev.resolution_m))), 1))
    continuity = cv2.morphologyEx(raw, cv2.MORPH_CLOSE, kernel)
    coverage = np.zeros(raw.shape[0])
    longest = np.zeros(raw.shape[0])
    active = np.zeros(raw.shape[0], dtype=bool)
    for v in range(raw.shape[0]):
        distance = bev.bev_pixel_to_ground(0, v)[1]
        xs = np.flatnonzero(corridor[v])
        if not (cfg.min_distance_m <= distance <= cfg.max_distance_m) or not len(xs):
            continue
        x0, x1 = int(xs[0]), int(xs[-1]) + 1
        active[v] = True
        coverage[v] = np.count_nonzero(raw[v, x0:x1]) / float(x1-x0)
        longest[v] = detector._longest_run(continuity[v, x0:x1]) / float(x1-x0)
    good = active & (coverage >= cfg.min_row_coverage_ratio) & (longest >= cfg.min_continuous_ratio)
    rows = np.flatnonzero(good)
    groups = np.split(rows, np.flatnonzero(np.diff(rows) > 1) + 1) if len(rows) else []
    bands = []
    for group in groups:
        thickness = len(group) * bev.resolution_m
        reason = ('too_thin' if thickness < cfg.min_thickness_m else
                  'too_thick' if thickness > cfg.max_thickness_m else
                  'crosswalk' if stop['crosswalk_detected'] else 'eligible_band')
        bands.append(f'y={group[0]}..{group[-1]} thickness={thickness:.3f}m:{reason}')
    n, labels, stats, _ = cv2.connectedComponentsWithStats(white_bev)
    roi_n = cv2.connectedComponents(raw)[0] - 1
    lines.append(
        f'STOP DEBUG raw_components={n-1} corridor_components={roi_n} '
        f'white_px={np.count_nonzero(white_bev)} corridor_white_px={np.count_nonzero(raw)} '
        f'distance_allowed={cfg.min_distance_m:.2f}..{cfg.max_distance_m:.2f}m '
        f'coverage_max={coverage.max():.3f}/min={cfg.min_row_coverage_ratio:.3f} '
        f'continuity_max={longest.max():.3f}/min={cfg.min_continuous_ratio:.3f} '
        f'joint_good_rows={len(rows)} thickness_allowed='
        f'{cfg.min_thickness_m:.3f}..{cfg.max_thickness_m:.3f}m')
    lines.append('STOP ROW GATES ' + (
        '; '.join(bands) if bands else
        'no_white_pixels' if not np.any(white_bev) else
        'outside_distance_or_corridor' if not np.any(raw) else
        'no_row_passes_coverage_AND_continuity'))
    # Limit console volume: all components counted; largest eight described.
    ids = sorted(range(1, n), key=lambda label: int(stats[label, cv2.CC_STAT_AREA]), reverse=True)
    for label in ids[:8]:
        x, y, w, h, area = map(int, stats[label])
        ys, xs = np.nonzero(labels[y:y+h, x:x+w] == label)
        angle = '--'
        if len(xs) > 1:
            eigenvalues, vectors = np.linalg.eigh(np.cov(np.c_[xs, ys].T))
            if eigenvalues[-1] > 0 and not np.isclose(eigenvalues[-1], eigenvalues[0]):
                axis = vectors[:, -1]
                angle = f'{np.degrees(np.arctan2(abs(axis[1]), abs(axis[0]))):.1f}deg'
        selected = raw[y+ys, x+xs] > 0
        component_rows = y + ys[selected]
        if not len(component_rows):
            reason = 'outside_distance_or_corridor'
        elif not np.any(good[component_rows]):
            reason = 'rows_fail_coverage_or_continuity'
        else:
            reason = 'see_ROW_GATES_for_band_thickness_and_crosswalk'
        lines.append(
            f'  WHITE component={label} area={area}px bbox_width={w}px/{w*bev.resolution_m:.3f}m '
            f'bbox_thickness={h}px/{h*bev.resolution_m:.3f}m angle={angle} '
            f'aspect={w/float(h):.2f} row_gate_context={reason}')
    lines.append(f'STOP components_shown={min(8, n-1)}/{n-1}; '
                 'angle/aspect/component_bbox are measurements ONLY, not detector rejection criteria')
    return '\n'.join(lines)


def main(argv=None):
    args = parser().parse_args(argv)
    if args.width <= 0 or args.height <= 0 or not math.isfinite(args.fps) or args.fps <= 0:
        raise ValueError('width, height and fps must be positive and finite')
    bev = C920BEV(args.config)
    if (args.width, args.height) != (bev.image_width, bev.image_height):
        raise ValueError('Requested resolution differs from calibration; use a matching --config')
    tracker = PaperLaneTracker()
    detector = StopLineDetector(resolution_m=bev.resolution_m,
                                far_m=bev.far_m, width_px=bev.width)
    stop_tracker = StopLineTracker()
    cap = writer = None
    try:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open {args.device} using V4L2')
        for prop, value in [(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                            (cv2.CAP_PROP_FRAME_WIDTH, args.width),
                            (cv2.CAP_PROP_FRAME_HEIGHT, args.height),
                            (cv2.CAP_PROP_FPS, args.fps)]:
            if not cap.set(prop, value):
                print(f'Camera property {prop} was not accepted; check negotiated format.', file=sys.stderr)
        print(f"Camera: {args.device} V4L2 "
              f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
              f"FPS={cap.get(cv2.CAP_PROP_FPS):.1f}; config={args.config}")
        if args.record:
            record = args.record.expanduser()
            if record.exists():
                raise FileExistsError(f'Recording already exists: {record}')
            writer = cv2.VideoWriter(str(record), cv2.VideoWriter_fourcc(*'mp4v'), args.fps,
                                     (bev.image_width + bev.width,
                                      max(bev.image_height, bev.height) + 240))
            if not writer.isOpened():
                raise RuntimeError(f'Cannot create recording: {record}')
        last_frame = time.monotonic()
        last_log = -math.inf
        measured_fps = 0.0
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera frame read failed; stopping live display')
            if frame.shape[:2] != (bev.image_height, bev.image_width):
                raise RuntimeError(f'Camera frame {frame.shape[:2]} does not match calibration')
            _, mask, white_bev = make_mask(frame, bev)
            lane = tracker.process(mask)
            stop = detector.detect(white_bev, lane)
            tracked = stop_tracker.update(stop)
            now = time.monotonic()
            instant = 1.0 / max(now - last_frame, 1e-9)
            measured_fps = instant if not measured_fps else .9 * measured_fps + .1 * instant
            last_frame = now
            canvas, status = annotate(frame, bev, mask, lane, stop, tracked, measured_fps)
            cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(1) & 0xFF
            if writer is not None:
                writer.write(canvas)
            if now - last_log >= 1.0:
                print(status, flush=True)
                print(rejection_debug(bev, tracker, detector, lane, stop, white_bev), flush=True)
                last_log = now
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('r'), ord('R')):
                tracker.reset()
                stop_tracker.reset()
                print('Lane and stop trackers reset (existing reset APIs).', flush=True)
    finally:
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as error:
        print(f'Live C920 failed: {error}', file=sys.stderr)
        sys.exit(1)
