"""Offline, one-output-frame-per-input-frame review of a recorder session."""
import argparse
from bisect import bisect_left
from dataclasses import asdict
import json
from pathlib import Path
import time

import cv2
import numpy as np

from .road_vision import VisionConfig, annotate, detect


def nearest_depth(stamp, stamps, rows, tolerance_ns=20_000_000):
    i = bisect_left(stamps, stamp)
    indices = [j for j in (i-1, i) if 0 <= j < len(stamps)]
    if not indices:
        return None
    j = min(indices, key=lambda j: abs(stamps[j]-stamp))
    return rows[j] if abs(stamps[j]-stamp) <= tolerance_ns else None


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def replay(session, output, config=VisionConfig(), lane_backend='baseline', lane_config=None):
    if lane_backend not in ('baseline', 'fma'):
        raise ValueError('Unknown lane backend')
    if lane_config is not None and lane_backend != 'fma':
        raise ValueError('lane_config requires fma backend')
    lane = None
    if lane_backend == 'fma':
        from .existing_lane import ExistingLane
        lane = ExistingLane(lane_config)
    session, output = Path(session).resolve(), Path(output).resolve()
    if output == session or session in output.parents:
        raise ValueError('Output must be outside the original session')
    rows = read_rows(session/'frames.jsonl')
    if not rows or [r['index'] for r in rows] != list(range(len(rows))):
        raise ValueError('RGB index is empty or non-contiguous')
    depths = sorted(read_rows(session/'depth_frames.jsonl'), key=lambda r: r['timestamp_ns'])
    stamps = [r['timestamp_ns'] for r in depths]
    metadata = json.loads((session/'session.json').read_text())
    aligned = 'aligned_depth_to_color' in metadata.get('topics', {}).get('depth', '')
    output.mkdir(parents=True, exist_ok=False)
    cap = cv2.VideoCapture(str(session/'color.mp4'))
    fps = cap.get(cv2.CAP_PROP_FPS)
    writer = None
    count = lane_frames = stop_frames = crosswalk_frames = matched = 0
    start = time.monotonic()
    try:
        if not cap.isOpened() or not np.isfinite(fps) or fps <= 0:
            raise ValueError('Cannot open source video or invalid FPS')
        with (output/'detections.jsonl').open('w') as stream:
            for row in rows:
                ok, frame = cap.read()
                if not ok:
                    raise ValueError(f'Video decode ended before metadata index {count}')
                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(str(output/'overlay.mp4'),
                                             cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    if not writer.isOpened():
                        raise RuntimeError('Cannot open overlay writer')
                result = detect(frame, config)
                result.update(index=row['index'], timestamp_ns=row['timestamp_ns'],
                              depth_timestamp_ns=None, depth_status='no_timestamp_match')
                depth_row = nearest_depth(row['timestamp_ns'], stamps, depths)
                if depth_row is not None:
                    matched += 1
                    result.update(depth_timestamp_ns=depth_row['timestamp_ns'],
                                  depth_status='timestamp_matched_not_loaded')
                if depth_row is not None and result['stop_candidates'] and aligned:
                    path = (session/depth_row['filename']).resolve()
                    if session not in path.parents:
                        raise ValueError('Depth path escapes session')
                    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                    if depth is None or depth.dtype != np.uint16 or depth.shape != frame.shape[:2]:
                        raise ValueError(f'Invalid aligned uint16 depth: {path}')
                    result['depth_status'] = 'aligned_depth_loaded'
                    scale = depth_row['depth_scale_m']
                    if not np.isfinite(scale) or scale <= 0:
                        raise ValueError('Invalid depth scale')
                    for stop in result['stop_candidates']:
                        x, y, bw, bh = stop['bbox']
                        hsv = cv2.cvtColor(frame[y:y+bh, x:x+bw], cv2.COLOR_BGR2HSV)
                        mask = cv2.inRange(hsv, (0, 0, config.white_v_min),
                                          (179, config.white_s_max, 255)) > 0
                        z = depth[y:y+bh, x:x+bw][mask].astype(float)*scale
                        valid = z[(z >= .1) & (z <= 10)]
                        stop.update(optical_z_m=None, valid_depth_pixels=int(valid.size))
                        if valid.size >= 20 and valid.size >= .3*z.size:
                            stop['optical_z_m'] = float(np.median(valid))
                if lane is not None:
                    result['existing_lane'], lane_debug = lane.detect(frame)
                    # Baseline lane candidates are excluded from the combined result.
                    result['lane_candidates'] = []
                    debug = annotate(lane_debug, result, draw_lanes=False, text_y=150)
                else:
                    debug = annotate(frame, result)
                cv2.putText(debug, f'frame {count} t={(row["timestamp_ns"]-rows[0]["timestamp_ns"])/1e9:.2f}s',
                            (10, 171 if lane else 43), 0, .5, (0, 255, 255), 1)
                writer.write(debug)
                if count in {0, len(rows)//5, 2*len(rows)//5, 3*len(rows)//5, 4*len(rows)//5, len(rows)-1}:
                    cv2.imwrite(str(output/f'sample_{count:06d}.jpg'), debug)
                stream.write(json.dumps(result, allow_nan=False)+'\n')
                count += 1
                lane_frames += (result['existing_lane']['visual_detected'] if lane
                                else bool(result['lane_candidates']))
                stop_frames += bool(result['stop_candidates'])
                crosswalk_frames += result['crosswalk_pattern']
            if cap.read()[0]:
                raise ValueError('Video has more frames than timestamp metadata')
    finally:
        cap.release()
        if writer is not None:
            writer.release()
    check = cv2.VideoCapture(str(output/'overlay.mp4'))
    decoded = 0
    while check.read()[0]:
        decoded += 1
    check.release()
    if decoded != count:
        raise ValueError(f'Overlay decode count mismatch: {decoded} != {count}')
    summary = dict(source=str(session), frames=count, output_decoded_frames=decoded,
                   lane_backend=lane.provenance if lane else {'backend': 'baseline'},
                   lane_candidate_frames=lane_frames, stop_candidate_frames=stop_frames,
                   crosswalk_ambiguity_frames=crosswalk_frames, timestamp_matched_depth_frames=matched,
                   source_timestamp_rate_hz=(count-1)*1e9/(rows[-1]['timestamp_ns']-rows[0]['timestamp_ns'])
                   if count > 1 and rows[-1]['timestamp_ns'] > rows[0]['timestamp_ns'] else None,
                   overlay_container_fps=fps, config=asdict(config),
                   elapsed_s=time.monotonic()-start,
                   note='Unlabelled candidates, not accuracy metrics. Optical Z is not bumper/ground distance. '
                        'Overlay uses source MP4 fixed FPS; original timestamps remain in JSONL.')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--config', type=Path, help='JSON fields from VisionConfig')
    parser.add_argument('--lane-backend', choices=('baseline', 'fma'), default='baseline')
    parser.add_argument('--lane-config', type=Path, help='JSON fields from existing LaneConfig; fma only')
    args = parser.parse_args()
    config = VisionConfig(**json.loads(args.config.read_text())) if args.config else VisionConfig()
    lane_config = json.loads(args.lane_config.read_text()) if args.lane_config else None
    print(json.dumps(replay(args.session, args.output, config, args.lane_backend, lane_config), indent=2))


if __name__ == '__main__':
    main()
