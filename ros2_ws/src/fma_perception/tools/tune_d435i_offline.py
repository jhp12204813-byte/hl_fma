"""Compare fixed candidates across sessions; counts are not accuracy labels."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
import cv2
import numpy as np
from fma_perception.bev_lane import BEVConfig, BEVLaneDetector
from fma_perception.lane_detection import LaneConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home()/'d435i_recordings')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--multilevel', action='store_true', help='Compare bottom vs multi-height seeds')
    parser.add_argument('--directional', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    # Image-space hypotheses only: neither measured ground geometry nor metric scale.
    base = BEVConfig(source_points=(.25,.46,.75,.46,1.,1.,0.,1.),
                     destination_points=(.2,0.,.8,0.,.8,1.,.2,1.))
    image = LaneConfig(roi_top_ratio=.42)
    candidates = {
        'A_original_hsv': (base,image),
        'B_yellow_range': (base,replace(image,yellow_h_min=8,yellow_s_min=45,yellow_v_min=60)),
        'C_wider_far_roi': (replace(base,source_points=(.12,.42,.88,.42,1.,1.,0.,1.)),
                             replace(image,yellow_h_min=8,yellow_s_min=45,yellow_v_min=60)),
        'D_short_support': (replace(base,min_support_ratio=.4,max_gap_ratio=.25,margin_ratio=.12),
                              replace(image,yellow_h_min=8,yellow_s_min=45,yellow_v_min=60)),
        'E_full_width': (replace(base,source_points=(0.,.35,1.,.35,1.,1.,0.,1.),
                                destination_points=(0.,0.,1.,0.,1.,1.,0.,1.),
                                min_support_ratio=.4,max_gap_ratio=.25,margin_ratio=.12),
                          replace(image,roi_top_ratio=.35,yellow_h_min=8,yellow_s_min=45,yellow_v_min=60)),
        'F_full_width_low_s': (replace(base,source_points=(0.,.35,1.,.35,1.,1.,0.,1.),
                                destination_points=(0.,0.,1.,0.,1.,1.,0.,1.),
                                min_support_ratio=.4,max_gap_ratio=.25,margin_ratio=.12),
                          replace(image,roi_top_ratio=.35,yellow_h_min=5,yellow_s_min=25,yellow_v_min=60)),
    }
    records=[]
    if args.multilevel:
        full, colors = candidates['F_full_width_low_s']
        candidates = {'before_bottom': (full, colors),
                      'after_multilevel': (replace(full, multi_height_seeds=True), colors)}
    if args.directional:
        full, colors = candidates['after_multilevel'] if args.multilevel else candidates['F_full_width_low_s']
        full = replace(full,multi_height_seeds=True)
        candidates = {'before_multilevel':(full,colors),
                      'after_directional':(replace(full,directional_tracking=True),colors)}
    for number,video in enumerate(sorted(args.root.glob('*/color.mp4'))):
        session=video.parent
        meta=json.loads((session/'session.json').read_text())
        timestamps=[json.loads(l)['timestamp_ns'] for l in (session/'frames.jsonl').open()]
        camera=meta['camera_info']['color']
        cap=cv2.VideoCapture(str(video));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        indices=np.linspace(0,n-1,8).astype(int)
        sheets={k:[] for k in candidates}
        try:
            for index in indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES,int(index));ok,frame=cap.read()
                if not ok: raise ValueError(f'Decode failed: {video} {index}')
                for name,(bev,im) in candidates.items():
                    detector=BEVLaneDetector(bev,im)
                    start=time.perf_counter();r=detector.process(frame,timestamps[index],camera)
                    records.append(dict(session=session.name,split='held_out' if number%3==2 else 'development',
                        frame=int(index),candidate=name,latency_ms=(time.perf_counter()-start)*1000,
                        curves=len(r.diagnostics['curves']),visual=r.estimate.visual_detected,
                        metric=r.estimate.detected,diagnostics=r.diagnostics))
                    tile=np.hstack([cv2.resize(r.estimate.debug,(320,240)),cv2.resize(r.bev_debug,(320,240))])
                    cv2.putText(tile,f'{name} f{index}',(10,232),0,.45,(0,255,255),1)
                    sheets[name].append(tile)
        finally: cap.release()
        for name,tiles in sheets.items():
            cv2.imwrite(str(args.output/f'{session.name}_{name}.jpg'),np.vstack(tiles))
        print(session.name,{name:sum(r['curves']>0 for r in records if r['session']==session.name and r['candidate']==name) for name in candidates},flush=True)
    (args.output/'results.json').write_text(json.dumps(dict(
        note='8 independent samples/session, no temporal evaluation; candidate counts are not labelled accuracy.',
        configs={k:dict(bev=asdict(b),image=asdict(i)) for k,(b,i) in candidates.items()},records=records),indent=2))
    parts=['<!doctype html><meta charset="utf-8"><h1>D435i 후보 비교</h1><p>각 세션 8개 표본. 왼쪽 원영상 / 오른쪽 BEV. 검출 횟수는 정확도가 아닙니다. 보정 미확정·metric 비활성.</p>']
    for p in sorted(args.output.glob('*.jpg')):
        parts.append(f'<h2>{p.stem}</h2><a href="{p.name}"><img width="640" src="{p.name}"></a>')
    (args.output/'index.html').write_text('\n'.join(parts))

if __name__=='__main__': main()
