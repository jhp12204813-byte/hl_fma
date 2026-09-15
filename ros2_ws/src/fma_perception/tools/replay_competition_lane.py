#!/usr/bin/env python3
"""Offline metric tracker replay. Accepts recorded files or synthetic masks, never camera devices."""
import argparse
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fma_perception.c920_bev import C920BEV
from fma_perception.competition_lane_tracker import CompetitionLaneTracker, MetricLaneConfig
from fma_perception.competition_lane_overlay import draw_competition_overlay
from replay_c920_lane import make_mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--video', type=Path)
    source.add_argument('--synthetic', action='store_true')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[4]/'config/c920_bev_calibration.yaml')
    parser.add_argument('--nominal-lane-width-m', type=float, default=MetricLaneConfig().nominal_lane_width_m)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=300)
    args = parser.parse_args()
    if args.video is not None and not args.video.is_file():
        parser.error('--video must be an existing recorded file')
    if args.max_frames <= 0:
        parser.error('--max-frames must be positive')
    bev = C920BEV(args.config)
    tracker = CompetitionLaneTracker(bev, MetricLaneConfig(nominal_lane_width_m=args.nominal_lane_width_m))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = []
    cap = cv2.VideoCapture(str(args.video)) if args.video is not None else None
    if cap is not None and not cap.isOpened():
        raise RuntimeError('Cannot open recorded video')
    try:
        for index in range(args.max_frames):
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    break
                _, mask, _ = make_mask(frame, bev)
                image = bev.warp_to_bev(frame)
            else:
                width = (2.7, 3.05, 3.2, 4.1)[(index//30) % 4]
                if index % 30 == 0:
                    tracker.reset()
                mask = np.zeros((bev.height, bev.width), np.uint8)
                xs = [-width/2, width/2] if index % 30 < 5 else [width/2]
                for x in xs:
                    u = int(round(bev.ground_to_bev_pixel(x, 0)[0]))
                    mask[20:bev.height-1, u-4:u+5] = 255
                for row in range(30, 490, 70):
                    mask[row:row+20, 555:580] = 255
                image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            start = time.perf_counter()
            result = tracker.process(mask, timestamp=index/30.)
            elapsed = time.perf_counter()-start
            record = {k: result.get(k) for k in ('source', 'confidence', 'confidence_grade', 'candidate_count',
                'selected_pair_score', 'second_pair_score', 'selected_lane_width_m', 'expected_lane_width_m',
                'width_delta_m', 'heading_diff_deg', 'pair_overlap_m', 'reject_reason')}
            records.append({'frame': index, 'processing_sec': elapsed, **record})
            if index % 30 in (0, 5, 29):
                cv2.imwrite(str(args.output_dir/f'frame_{index:04d}.png'), draw_competition_overlay(image, bev, result))
    finally:
        if cap is not None:
            cap.release()
    (args.output_dir/'diagnostics.json').write_text(json.dumps(records, indent=2))
    counts = {source: sum(r['source']==source for r in records) for source in {r['source'] for r in records}}
    print(json.dumps({'frames': len(records), 'sources': counts,
                      'max_processing_sec': max((r['processing_sec'] for r in records), default=0.)}))


if __name__ == '__main__':
    main()
