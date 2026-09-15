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


from fma_perception.stop_line_detector import StopLineConfig


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--competition-tracking', action='store_true', help='Metric pair/persistent single-side diagnostics')
    ap.add_argument('--nominal-lane-width-m', type=float, default=3.5)
    ap.add_argument('--stop-min-thickness-m', type=float, default=None,
                    help='Diagnostic detector minimum thickness in meters (default: 0.30 for live field/competition diagnostics)')
    ap.add_argument('--device', default='/dev/video2')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30.0)
    # Same calibration file as replay, resolved relative to repository root.
    ap.add_argument('--config', type=Path,
                    default=REPO / 'config/c920_bev_calibration.yaml')
    ap.add_argument('--record', type=Path, help='Record the displayed composite MP4')
    return ap


def right_near_overlay(debug, bev, raw_mask, lane):
    """Display-only: blue raw, cyan candidate halo, red rejected, green selected."""
    out = debug.copy()
    rows = bev.far_m - np.arange(bev.height) * bev.resolution_m
    near = (rows >= 1.) & (rows <= 3.)
    origin, _ = bev.ground_to_bev_pixel(0, 0)
    right = np.arange(bev.width) >= origin
    region = near[:, None] & right[None, :]
    out[(raw_mask > 0) & region] = (255, 100, 0)
    fit = lane.get('right')
    if fit is None:
        return out

    def point_mask(prefix):
        if prefix + '_x' not in fit or prefix + '_y' not in fit:
            return None
        xs, ys = np.asarray(fit[prefix + '_x']), np.asarray(fit[prefix + '_y'])
        valid = (np.isfinite(xs) & np.isfinite(ys) & (xs >= 0) & (xs < bev.width)
                 & (ys >= 0) & (ys < bev.height))
        points = np.zeros((bev.height, bev.width), np.uint8)
        points[ys[valid].astype(int), xs[valid].astype(int)] = 1
        return (points > 0) & region

    candidates = point_mask('_candidate')
    selected = point_mask('_inlier')
    if candidates is not None:
        halo = cv2.dilate(candidates.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        out[halo & region] = (255, 255, 0)
        if selected is not None:
            out[candidates & ~selected] = (0, 0, 255)
    if selected is not None:
        out[selected] = (0, 255, 0)
    return out


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
    if lane.get('_competition') is not None:
        from fma_perception.competition_lane_overlay import draw_competition_overlay
        debug = draw_competition_overlay(debug, bev, lane['_competition'])[:bev.height]
    else:
        debug = right_near_overlay(debug, bev, mask, lane)
    quality = lane.get('pair_quality') or {}
    competition = lane.get('_competition')
    if competition is not None:
        valid = bool(competition.get('valid', False))
        state = competition.get('source', 'INVALID')
    else:
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
    cv2.putText(canvas, 'RIGHT 1-3m: raw BLUE | candidate CYAN halo | rejected RED | selected GREEN',
                (12, top_height + 232), cv2.FONT_HERSHEY_SIMPLEX, .48,
                (235, 235, 235), 1, cv2.LINE_AA)
    if lane.get('_competition') is not None:
        result = lane['_competition']
        canvas[top_height+214:] = 0
        cv2.putText(canvas, f"{result['source']} confidence={result['confidence']:.2f} width={result['selected_lane_width_m']} EMA={result['expected_lane_width_m']}",
                    (12, top_height+232), cv2.FONT_HERSHEY_SIMPLEX, .48, (235,235,235), 1, cv2.LINE_AA)
    return canvas, ' | '.join(lines[i] for i in (0, 3, 4, 5, 8, 10, 11, 12))


def rejection_debug(bev, tracker, detector, lane, stop, white_bev):
    """Read-only diagnostic replay of row gates; never feed back into detection.

    The existing stop detector selects ROW BANDS, not connected components.
    Component geometry below is descriptive only, not an angle/aspect gate.
    """
    quality = lane.get('pair_quality') or {}
    measured = quality.get('median_width_px')
    width = '--' if measured is None else f'{measured:.1f}px/{measured * bev.resolution_m:.2f}m'

    if hasattr(tracker.cfg, 'min_pair_width_px'):
        minimum_px = tracker.cfg.min_pair_width_px
        minimum_m = minimum_px * bev.resolution_m
    else:
        minimum_m = tracker.cfg.min_lane_width_m
        minimum_px = minimum_m / bev.resolution_m

    lines = [f"LANE DEBUG width={width} allowed_min={minimum_px:.1f}px/"
             f"{minimum_m:.2f}m allowed_max=NONE "
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
    max_gap_px = max(1, int(round(cfg.merge_band_gap_m / bev.resolution_m)))
    groups = (
        np.split(rows, np.flatnonzero(np.diff(rows) > max_gap_px) + 1)
        if len(rows) else []
    )
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
    stop_config = StopLineConfig()
    if args.stop_min_thickness_m is None:
        # Competition/field diagnostic default.
        # Keep StopLineConfig's generic detector default unchanged.
        stop_config.min_thickness_m = 0.30
    else:
        if not math.isfinite(args.stop_min_thickness_m) or not 0 < args.stop_min_thickness_m <= stop_config.max_thickness_m:
            raise ValueError('stop-min-thickness-m must be positive and within detector maximum')
        stop_config.min_thickness_m = args.stop_min_thickness_m
    bev = C920BEV(args.config)
    if (args.width, args.height) != (bev.image_width, bev.image_height):
        raise ValueError('Requested resolution differs from calibration; use a matching --config')
    if args.competition_tracking:
        from fma_perception.competition_lane_tracker import CompetitionLaneTracker, MetricLaneConfig
        tracker = CompetitionLaneTracker(bev, MetricLaneConfig(nominal_lane_width_m=args.nominal_lane_width_m))
    else:
        tracker = PaperLaneTracker(diagnostics=True)
    detector = StopLineDetector(resolution_m=bev.resolution_m,
                                far_m=bev.far_m, width_px=bev.width, config=stop_config)
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
            cap.set(prop, value)
        last = time.monotonic()
        frames = 0
        fps = 0.0
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError('C920 read failed')
            _, mask, white_bev = make_mask(frame, bev)
            if args.competition_tracking:
                result = tracker.process(mask, timestamp=time.monotonic())
                lane = tracker.to_lane_result(result)
                lane['_competition'] = result
            else:
                lane = tracker.process(mask)
            stop = detector.detect(white_bev, lane)
            tracked = stop_tracker.update(stop)
            frames += 1
            now = time.monotonic()
            if now - last >= 1.0:
                fps = frames / (now - last)
                frames = 0
                last = now
            canvas, summary = annotate(frame, bev, mask, lane, stop, tracked, fps)
            cv2.imshow(WINDOW, canvas)
            if writer is None and args.record:
                args.record.parent.mkdir(parents=True, exist_ok=True)
                writer = cv2.VideoWriter(str(args.record), cv2.VideoWriter_fourcc(*'mp4v'),
                                         args.fps, (canvas.shape[1], canvas.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError(f'Cannot open video writer: {args.record}')
            if writer is not None:
                writer.write(canvas)
            print(summary)
            print(rejection_debug(bev, tracker, detector, lane, stop, white_bev))
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key == ord('r'):
                tracker.reset()
                stop_tracker.reset()
    finally:
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
