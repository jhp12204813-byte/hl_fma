"""Offline ground-plane validation across recorded D435i sessions."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from fma_perception.ground_plane import estimate,intersect

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path.home()/'d435i_recordings');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
records=[]
for video in sorted(a.root.glob('*/color.mp4')):
 session=video.parent;m=json.loads((session/'session.json').read_text());camera=m['camera_info']['color'];depth_camera=m['camera_info']['depth']
 if camera['header']['frame_id']!=depth_camera['header']['frame_id'] or not np.allclose(camera['k'],depth_camera['k']):raise ValueError('Depth alignment mismatch')
 rows=[json.loads(l) for l in (session/'frames.jsonl').open()];dr=[json.loads(l) for l in (session/'depth_frames.jsonl').open()];stamps=np.array([r['timestamp_ns'] for r in dr],dtype=np.int64);cap=cv2.VideoCapture(str(video));panels=[]
 for index in np.linspace(0,len(rows)-1,6).astype(int):
  stamp=rows[index]['timestamp_ns'];j=int(np.argmin(np.abs(stamps-stamp)));delta=abs(int(stamps[j])-stamp);record=dict(session=session.name,index=int(index),timestamp_ns=stamp,depth_delta_ns=delta)
  if delta>20_000_000:record['error']='No aligned depth within 20ms';records.append(record);continue
  cap.set(1,int(index));ok,frame=cap.read()
  if not ok:raise ValueError('RGB decode error')
  depth=cv2.imread(str(session/dr[j]['filename']),cv2.IMREAD_UNCHANGED)
  try:
   plane,pixels,keep=estimate(depth,camera,dr[j]['depth_scale_m']);record['plane']=plane
   overlay=frame.copy()
   x0,y0,x1,y1=plane['roi_xyxy']
   cv2.rectangle(overlay,(x0,y0),(x1,y1),(255,150,0),2)
   for (x,y),good in zip(pixels.astype(int),keep):overlay[y,x]=(0,180,0) if good else (0,0,255)
   top=np.zeros((600,600,3),np.uint8)
   hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
   for color,low,high,paint in [('yellow',(5,25,60),(40,255,255),(0,255,255)),('white',(0,0,170),(179,65,255),(255,255,255))]:
    mask=cv2.inRange(hsv,low,high);mask[:int(frame.shape[0]*.35)]=0
    yy,xx=np.nonzero(mask);xy,valid=intersect(np.c_[xx,yy],camera,plane)
    valid &= (np.abs(xy[:,0])<3)&(xy[:,1]>0)&(xy[:,1]<6)
    if plane['accepted']:
     uv=np.rint(np.c_[300+xy[valid,0]*100,599-xy[valid,1]*100]).astype(int);uv=np.clip(uv,0,599);top[uv[:,1],uv[:,0]]=paint
    record[color+'_projected_pixels']=int(valid.sum())
   cv2.putText(top,'1px=1cm | X right, Y optical-ground forward',(8,20),0,.42,(0,180,255),1)
   cv2.putText(overlay,f'plane={plane["accepted"]} h={plane["camera_height_m"]:.3f}m inliers={plane["inlier_ratio"]:.2f}',(10,25),0,.5,(0,255,255),1)
   if not plane['accepted']:cv2.putText(top,'PLANE REJECTED',(20,70),0,.7,(0,0,255),2)
   cv2.putText(overlay,f'frame {index}',(10,45),0,.5,(0,255,255),1)
   panels.append(np.hstack([cv2.resize(overlay,(480,360)),cv2.resize(top,(360,360))]))
  except ValueError as e:record['error']=str(e)
  records.append(record)
 cap.release()
 if panels:cv2.imwrite(str(a.output/(session.name+'.jpg')),np.vstack(panels))
 print(session.name,sum(r.get('plane',{}).get('accepted',False) for r in records if r['session']==session.name),flush=True)
(a.output/'results.json').write_text(json.dumps(records,indent=2))
(a.output/'index.html').write_text('<meta charset="utf-8"><h1>Depth ground plane / metric top-down</h1><p>왼쪽: 지면 inlier 초록/outlier 빨강. 오른쪽: 차선 ray-plane 교점. 차량 좌표 아님. 기본 HSV 유지.</p>'+''.join(f'<h2>{p.stem}</h2><img width="840" src="{p.name}">' for p in sorted(a.output.glob('*.jpg'))))
