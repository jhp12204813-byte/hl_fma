"""Paired ground ROI comparison on identical D435i lane-colour observations."""
from pathlib import Path
import argparse,json
import cv2,numpy as np
from fma_perception.ground_plane import intersect


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reviews',type=Path,default=Path.home()/'d435i_vision_reviews');p.add_argument('--root',type=Path,default=Path.home()/'d435i_recordings');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
 old=json.loads((a.reviews/'ground_depth_v2/results.json').read_text());new=json.loads((a.reviews/'ground_depth_full_width/results.json').read_text());lookup={(r['session'],r['index']):r for r in old};records=[];sheets={}
 for r in new:
  if not r.get('plane',{}).get('accepted'):continue
  prev=lookup[r['session'],r['index']]
  if not prev.get('plane',{}).get('accepted'):continue
  root=a.root/r['session'];camera=json.loads((root/'session.json').read_text())['camera_info']['color'];cap=cv2.VideoCapture(str(root/'color.mp4'));cap.set(1,r['index']);ok,f=cap.read();cap.release()
  if not ok:raise ValueError('Decode failed')
  hsv=cv2.cvtColor(f,cv2.COLOR_BGR2HSV);mask=cv2.inRange(hsv,(5,25,60),(40,255,255));mask[:int(f.shape[0]*.35)]=0;mask[:,f.shape[1]//2:]=0
  yy,xx=np.nonzero(mask);pixels=np.c_[xx,yy];xy0,v0=intersect(pixels,camera,prev['plane']);xy1,v1=intersect(pixels,camera,r['plane'])
  valid=v0&v1&(xy0[:,1]>1)&(xy0[:,1]<6)&(xy1[:,1]>1)&(xy1[:,1]<6)&(abs(xy0[:,0])<3)&(abs(xy1[:,0])<3)
  dist=np.linalg.norm(xy1[valid]-xy0[valid],axis=1);item=dict(session=r['session'],frame=r['index'],yellow_left_image_pixels=len(xx),paired_ground_pixels=int(valid.sum()),shift_median_m=float(np.median(dist)) if len(dist) else None,shift_p95_m=float(np.percentile(dist,95)) if len(dist) else None)
  top=[]
  for label,xy,v in [('narrow',xy0,v0),('full width',xy1,v1)]:
   good=v&(xy[:,1]>0)&(xy[:,1]<6)&(abs(xy[:,0])<3);canvas=np.zeros((600,600,3),np.uint8)
   uv=np.rint(np.c_[300+xy[good,0]*100,599-xy[good,1]*100]).astype(int);uv=np.clip(uv,0,599);canvas[uv[:,1],uv[:,0]]=(0,255,255)
   bins=[]
   for y in np.arange(1,6,.1):
    points=xy[good&(xy[:,1]>=y)&(xy[:,1]<y+.1)]
    if len(points)>=5:bins.append((y+.05,float(np.median(points[:,0]))))
   stats=dict(supported_bins=len(bins),angle_deg=None,rms_m=None)
   if len(bins)>=15:
    pts=np.array(bins);coef=np.polyfit(pts[:,0],pts[:,1],1);stats.update(angle_deg=float(np.degrees(np.arctan(coef[0]))),rms_m=float(np.sqrt(np.mean((pts[:,1]-np.polyval(coef,pts[:,0]))**2))))
   item[label]=stats;cv2.putText(canvas,label,(10,25),0,.7,(0,200,255),1);top.append(cv2.resize(canvas,(360,360)))
  overlay=f.copy();overlay[mask>0]=(0,255,255);cv2.putText(overlay,f'frame {r["index"]}: left image yellow candidates',(10,25),0,.5,(0,200,255),1)
  panel=np.hstack([cv2.resize(overlay,(480,360)),*top]);sheets.setdefault(r['session'],[]).append(panel);records.append(item)
 for session,panels in sheets.items():cv2.imwrite(str(a.output/(session+'.jpg')),np.vstack(panels))
 summary=dict(note='Image-left yellow pixels, not labelled true lanes. Same pixels projected by both planes; no threshold changes.',records=records)
 (a.output/'results.json').write_text(json.dumps(summary,indent=2));(a.output/'index.html').write_text('<meta charset="utf-8"><h1>왼쪽 노란 도색: 좁은 ROI / 전체폭 지면 비교</h1><p>왼쪽 원영상 후보 / 가운데 좁은 ROI 평면 / 오른쪽 전체폭 평면. 차선 정답 라벨 아님. 1cm/pixel grid.</p>'+''.join(f'<h2>{f.stem}</h2><img width="1200" src="{f.name}">' for f in sorted(a.output.glob('*.jpg'))))
 print('paired frames',len(records));vals=[r['shift_median_m'] for r in records if r['shift_median_m'] is not None];print('median frame shift m',np.median(vals),'max',max(vals));print('support gained/lost',sum(r['full width']['supported_bins']>r['narrow']['supported_bins'] for r in records),sum(r['full width']['supported_bins']<r['narrow']['supported_bins'] for r in records))

if __name__=='__main__':main()
