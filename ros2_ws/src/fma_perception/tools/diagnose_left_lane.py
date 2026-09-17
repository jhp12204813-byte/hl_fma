"""Trace actual pipeline stages on several D435i sessions; no ROS actions."""
from pathlib import Path
from dataclasses import replace
import sys,json
import cv2,numpy as np,yaml
from fma_perception.bev_lane import BEVConfig,BEVLaneDetector,robust_curve
from fma_perception.lane_detection import LaneConfig

root=Path.home()/'d435i_recordings';out=Path.home()/'d435i_vision_reviews/left_lane_diagnosis';out.mkdir(exist_ok=True)
v=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/d435i_roi_multilevel_preview.yaml').read_text())['lane_detector']['ros__parameters']
b=BEVConfig(**{k[4:]:tuple(x) if k.endswith('_points') else x for k,x in v.items() if k.startswith('bev_')})
im=LaneConfig(**{k:x for k,x in v.items() if k in LaneConfig.__dataclass_fields__})
cv2.setNumThreads(1);records=[]
np.finfo(np.float64);np.finfo(np.float32)
for name in ['20260909_183629_622750','20260909_184157_136468','20260909_184328_535116','20260909_185208_014913']:
 p=root/name;camera=json.loads((p/'session.json').read_text())['camera_info']['color'];rows=[json.loads(l) for l in (p/'frames.jsonl').open()];cap=cv2.VideoCapture(str(p/'color.mp4'));n=int(cap.get(7))
 for idx in [n//4,n//2]:
  cap.set(1,idx);ok,f=cap.read();assert ok
  panels=[]
  for label,cfg in [('baseline',b),('support_008',replace(b,partial_support_ratio=.08)),('width_012',replace(b,max_marking_width_ratio=.12)),('windows_24',replace(b,windows=24))]:
   trace={};fits=[]
   import fma_perception.bev_lane as module
   original_fit, original_slide = module.robust_curve, module.sliding_curves
   def traced_fit(ys,xs,config,partial=False):
    result=original_fit(ys,xs,config,partial=partial)
    fits.append(dict(left=bool(np.median(xs)<320),rows=len(ys),span=float(np.ptp(ys)),success=result is not None))
    return result
   def traced_slide(warped,config):
    clean,curves,windows=original_slide(warped,config)
    trace.update(warped=warped.copy(),clean=clean.copy())
    return clean,curves,windows
   module.robust_curve,module.sliding_curves=traced_fit,traced_slide
   try:r=BEVLaneDetector(cfg,im).process(f,rows[idx]['timestamp_ns'],camera)
   finally:module.robust_curve,module.sliding_curves=original_fit,original_slide
   trace['mask']=np.uint8(np.any(r.estimate.debug_mask>0,axis=2))*255
   curves=r.diagnostics['curves'];left=[c for c in curves if np.polyval(c['coefficients'],(c['y_min']+c['y_max'])/2/479)<320]
   mask=trace['mask'];clean=trace['clean'];warped=trace['warped']
   records.append(dict(session=name,frame=idx,trial=label,left_curves=len(left),total_curves=len(curves),left_hsv_pixels=int(np.count_nonzero(mask[:,:320])),left_warp_pixels=int(np.count_nonzero(warped[:,:320])),left_clean_pixels=int(np.count_nonzero(clean[:,:320])),fits=fits,diagnostics=r.diagnostics))
   panel=np.hstack([cv2.resize(r.estimate.debug,(320,240)),cv2.resize(cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR),(320,240)),cv2.resize(r.bev_debug,(320,240))]);cv2.putText(panel,f'{label} f{idx} LEFT={len(left)}',(10,232),0,.5,(0,255,255),1);panels.append(panel)
  cv2.imwrite(str(out/f'{name}_f{idx}.jpg'),np.vstack(panels))
 cap.release()
(out/'trace.json').write_text(json.dumps(records,indent=2))
for r in records:
 print(r['session'],r['frame'],r['trial'],'left',r['left_curves'],'mask',r['left_hsv_pixels'],'warp/clean',r['left_warp_pixels'],r['left_clean_pixels'],'fits',[(f['rows'],round(f['span']),f['success']) for f in r['fits'] if f['left']],flush=True)
(out/'index.html').write_text('<meta charset="utf-8"><h1>왼쪽 차선 단계별 분석</h1><p>각 행: 원영상 / HSV mask / 추적. 기본값과 지지길이·도색폭·window 수 개별 변경 비교. 후보 수는 정확도가 아닙니다.</p>'+''.join(f'<h2>{p.stem}</h2><img width="960" src="{p.name}">' for p in sorted(out.glob('*_f*.jpg'))))
