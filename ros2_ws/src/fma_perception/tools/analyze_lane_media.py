"""Sample unchanged lane detector output and save baseline/A/B evidence.

Run with PYTHONNOUSERSITE=1 and the source package on PYTHONPATH.
The Python profiler captures existing debug locals in a separate, untimed call.
Fit counts are algorithm outputs, not ground-truth accuracy measurements.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from fma_perception.lane_detection import LaneConfig, detect_lane, roi_bounds


CONFIGS = {
    'baseline': LaneConfig(),
    'A': LaneConfig(roi_top_ratio=.25, roi_bottom_ratio=.6,
                    lookahead_ratio=.4, min_abs_slope=.3),
    'B': LaneConfig(roi_top_ratio=.25, roi_bottom_ratio=.6,
                    lookahead_ratio=.4, min_abs_slope=.3, white_v_min=200),
}


def measure(bgr, config):
    start = time.perf_counter()
    result = detect_lane(bgr, config)
    elapsed = time.perf_counter() - start
    captured = {}

    def capture(frame, event, arg):
        if frame.f_code is detect_lane.__code__ and event == 'return':
            captured.update(frame.f_locals)

    previous = sys.getprofile()
    sys.setprofile(capture)
    try:
        detect_lane(bgr, config)
    finally:
        sys.setprofile(previous)
    x, top, right, bottom = roi_bounds(bgr.shape, config)
    area = (right - x + 1) * (bottom - top + 1)
    roi = np.s_[top:bottom + 1, x:right + 1]
    center = captured['lane_center_px']
    row = {
        'ms': elapsed * 1000,
        **{key: 100 * np.count_nonzero(captured[key][roi]) / area
           for key in ('white', 'yellow', 'mask')},
        'left': captured['fits'][0] is not None,
        'right': captured['fits'][1] is not None,
        'visual': result.visual_detected,
        'center': center,
        'image_center': captured['image_center_px'],
        'error': None if center is None else captured['image_center_px'] - center,
        'hsv_p10_p50_p90': np.percentile(
            captured['hsv'][roi].reshape(-1, 3), [10, 50, 90], axis=0).tolist(),
    }
    return result, row


def save_preview(output, stem, result, label):
    h, w = result.debug.shape[:2]
    scale = min(640 / w, 360 / h)
    size = (int(w * scale), int(h * scale))
    thumb = cv2.resize(result.debug, size)
    tile = np.zeros((390, 640, 3), np.uint8)
    tile[:size[1], :size[0]] = thumb
    cv2.putText(tile, label, (5, 382), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (255, 255, 255), 1)
    cv2.imwrite(str(output / f'{stem}.jpg'), tile)
    cv2.imwrite(str(output / f'{stem}_mask.png'), cv2.resize(
        result.debug_mask, size, interpolation=cv2.INTER_NEAREST))
    return tile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('media', nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for video, path in enumerate(args.media, 1):
        cap = cv2.VideoCapture(path)
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if not cap.isOpened() or count < 2 or not np.isfinite(fps) or fps <= 0:
                raise ValueError(f'Cannot sample video: {path}')
            # Penultimate frame avoids unreliable final-frame seeks in some MP4s.
            preview_indices = set(np.linspace(0, count - 2, 25, dtype=int).tolist())
            indices = sorted(preview_indices | set(range(0, count, 30)))
            previews = {name: [] for name in CONFIGS}
            for index in indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, bgr = cap.read()
                if not ok:
                    print(f'Skipped unreadable frame {index}: {path}')
                    continue
                for name, config in CONFIGS.items():
                    result, row = measure(bgr, config)
                    row.update(video=video, frame=index, time=index / fps, config=name)
                    rows.append(row)
                    if index in preview_indices:
                        label = (f'{name} v{video} f{index} L{int(row["left"])} '
                                 f'R{int(row["right"])} C={row["center"]}')
                        previews[name].append(save_preview(
                            args.output, f'{name}_v{video}_f{index}', result, label))
            for name, tiles in previews.items():
                if tiles:
                    tiles += [np.zeros_like(tiles[0])] * ((-len(tiles)) % 5)
                    sheet = np.vstack([np.hstack(tiles[i:i + 5])
                                       for i in range(0, len(tiles), 5)])
                    cv2.imwrite(str(args.output / f'{name}_v{video}_sheet.jpg'), sheet)
        finally:
            cap.release()
    (args.output / 'results.json').write_text(json.dumps(rows, indent=2))
    for video in range(1, len(args.media) + 1):
        for name in CONFIGS:
            subset = [r for r in rows if r['video'] == video and r['config'] == name]
            if not subset:
                continue
            print(json.dumps({
                'video': video, 'config': name, 'samples': len(subset),
                **{k: sum(r[k] for r in subset) for k in ('left', 'right', 'visual')},
                **{k + '_mean_percent': np.mean([r[k] for r in subset])
                   for k in ('white', 'yellow', 'mask')},
                'offline_fps': 1000 / np.mean([r['ms'] for r in subset]),
            }))


if __name__ == '__main__':
    main()
