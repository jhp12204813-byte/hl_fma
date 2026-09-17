"""Phone footage is a false-positive fixture, NOT a geometry calibration source."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from fma_perception.bev_lane import BEVConfig, BEVLaneDetector
from fma_perception.lane_detection import LaneConfig, detect_lane


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video1', type=Path, default=Path('/home/idp2/차선영상1.mp4'))
    parser.add_argument('--video2', type=Path, default=Path('/home/idp2/차선영상2.mp4'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--multilevel', action='store_true')
    parser.add_argument('--directional', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    # Only stretch the existing ROI for a pixel-space negative regression.
    # This is deliberately NOT a road-plane homography or physical calibration.
    config = BEVConfig(undistort=False, lane_colors='white_yellow', multi_height_seeds=args.multilevel,
                       directional_tracking=args.directional,
                       source_points=(0., .6, 1., .6, 1., 1., 0., 1.),
                       destination_points=(0., 0., 1., 0., 1., 1., 0., 1.))
    frames2 = sorted(set([603,643,884,840]+list(range(588,619,5))+list(range(628,659,5))
                         +list(range(869,900,5))+[480,510,540,570,690,720,750,780,810,930]))
    results = []
    for path, indices in ((args.video1,[0,60,120,180]),(args.video2,frames2)):
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise ValueError(f'Missing media {path}')
        fps = cap.get(cv2.CAP_PROP_FPS)
        try:
            for index in indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES,index)
                ok, frame = cap.read()
                if not ok:
                    raise ValueError(f'Cannot decode {path} frame {index}')
                old = detect_lane(frame)
                frame_config = replace(config, image_width=frame.shape[1], image_height=frame.shape[0])
                new = BEVLaneDetector(frame_config,LaneConfig()).process(frame,int(index/fps*1e9))
                key = path.stem+f'_f{index}'
                cv2.imwrite(str(args.output/(key+'_bev.jpg')),new.bev_debug)
                comparison = np.hstack([cv2.resize(old.debug,(640,360)),cv2.resize(new.estimate.debug,(640,360))])
                cv2.imwrite(str(args.output/(key+'_comparison.jpg')),comparison)
                results.append(dict(video=str(path),frame=index,
                    baseline_visual=old.visual_detected,bev_visual=new.estimate.visual_detected,
                    metric_detected=new.estimate.detected,diagnostics=new.diagnostics))
                print(path.name,index,old.visual_detected,new.estimate.visual_detected,
                      len(new.diagnostics['curves']),flush=True)
        finally:
            cap.release()
    import fma_perception.bev_lane as module
    summary = dict(note='Uncalibrated ROI crop only. Not a BEV accuracy or metric validation.',
                   config=asdict(config),source_sha256=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
                   opencv_version=cv2.__version__,frames=results)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    core = [r for r in results if r['video']==str(args.video2) and r['frame'] in (603,643,884)]
    assert len(core)==3 and all(not r['bev_visual'] for r in core), 'Core false-positive regression failed'
    assert all(not r['metric_detected'] for r in results)


if __name__ == '__main__':
    main()
