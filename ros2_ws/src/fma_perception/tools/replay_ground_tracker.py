"""Sequential short-block regression; no control publisher or hardware access."""
import argparse,json
from dataclasses import asdict
from pathlib import Path
import cv2,numpy as np
from fma_perception.ground_tracker import GroundTracker, GroundConfig
from fma_perception.ground_metrics import summarize


def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path.home()/'d435i_recordings');p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path);p.add_argument('--frames-per-block',type=int,default=10);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);all_rows=[]
 cfg=GroundConfig(**json.loads(a.config.read_text())) if a.config else GroundConfig()
 (a.output/'config.json').write_text(json.dumps(asdict(cfg),indent=2))
 for video in sorted(a.root.glob('*/color.mp4')):
  root=video.parent;m=json.loads((root/'session.json').read_text());cam=m['camera_info']['color'];dc=m['camera_info']['depth']
  if cam['header']['frame_id']!=dc['header']['frame_id'] or not np.allclose(cam['k'],dc['k']):raise ValueError('Misaligned CameraInfo')
  rows=[json.loads(l) for l in (root/'frames.jsonl').open()];dr=[json.loads(l) for l in (root/'depth_frames.jsonl').open()];stamps=np.array([x['timestamp_ns'] for x in dr],np.int64);cap=cv2.VideoCapture(str(video));panels=[]
  writer=cv2.VideoWriter(str(a.output/(root.name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),30,(1240,480));assert writer.isOpened();count=0
  try:
   for start in [len(rows)//4,len(rows)//2,3*len(rows)//4]:
    tracker=GroundTracker(cfg);cap.set(1,start)
    for index in range(start,min(start+a.frames_per_block,len(rows))):
     ok,f=cap.read()
     if not ok:raise ValueError('Video decode error')
     stamp=rows[index]['timestamp_ns'];j=int(np.argmin(np.abs(stamps-stamp)));delta=abs(int(stamps[j])-stamp)
     if delta>20_000_000:
      all_rows.append(dict(session=root.name,index=index,status='depth_unmatched'));tracker=GroundTracker(cfg);continue
     depth=cv2.imread(str(root/dr[j]['filename']),-1)
     try:r,bev=tracker.process(f,depth,cam,dr[j]['depth_scale_m'],stamp)
     except ValueError as e:r=dict(status=str(e));bev=np.zeros((330,600,3),np.uint8);tracker=GroundTracker(cfg)
     r.update(session=root.name,index=index,block_start=start,depth_delta_ns=delta);all_rows.append(r)
     overlay=f.copy();cv2.rectangle(overlay,(0,int(480*cfg.roi_top)),(639,479),(255,150,0),2)
     cv2.putText(overlay,f'f{index} {r["status"]}',(10,22),0,.5,(0,255,255),1)
     cv2.putText(overlay,'SHORT BLOCKS: source timestamps in JSON',(10,44),0,.5,(0,255,255),1)
     canvas=np.zeros((480,1240,3),np.uint8);canvas[:,:640]=overlay;canvas[:bev.shape[0],640:]=bev
     writer.write(canvas);count+=1
     if index==start+min(5,a.frames_per_block-1):panels.append(canvas)
  finally:cap.release();writer.release()
  check=cv2.VideoCapture(str(a.output/(root.name+'.mp4')));decoded=0
  while check.read()[0]:decoded+=1
  check.release();assert decoded==count
  if panels:cv2.imwrite(str(a.output/(root.name+'.jpg')),np.vstack(panels))
  print(root.name,'frames',count,'pairs',sum(x.get('center') is not None for x in all_rows if x['session']==root.name),flush=True)
 (a.output/'results.json').write_text(json.dumps(all_rows,indent=2))
 (a.output/'metrics.json').write_text(json.dumps(summarize(all_rows),indent=2))
 (a.output/'index.html').write_text('<meta charset="utf-8"><h1>Depth inverse BEV + tracker</h1><p><a href="metrics.json">세션별 통계</a> | <a href="results.json">프레임별 rejection log</a></p><p>각 영상 3개 위치의 짧은 연속 프레임 블록. 전체영상 아님. 원본 좌표 유지·제어 출력 비활성.</p>'+''.join(f'<h2>{f.stem}</h2><video width="1240" controls preload="none" src="{f.stem}.mp4"></video><br><img width="1240" src="{f.name}">' for f in sorted(a.output.glob('*.jpg'))))

if __name__=='__main__':main()
