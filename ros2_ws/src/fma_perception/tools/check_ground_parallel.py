import cv2,json,numpy as np
from pathlib import Path
from fma_perception.ground_plane import intersect
root=Path('/home/idp2/d435i_recordings');out=Path('/home/idp2/d435i_vision_reviews/ground_depth_v2');records=json.loads((out/'results.json').read_text());results=[]
for rec in records:
 if not rec.get('plane',{}).get('accepted'):continue
 p=root/rec['session'];cam=json.loads((p/'session.json').read_text())['camera_info']['color'];cap=cv2.VideoCapture(str(p/'color.mp4'));cap.set(1,rec['index']);ok,f=cap.read();cap.release()
 h=cv2.cvtColor(f,cv2.COLOR_BGR2HSV);mask=cv2.inRange(h,(5,25,60),(40,255,255));yy,xx=np.nonzero(mask);xy,valid=intersect(np.c_[xx,yy],cam,rec['plane']);valid &= (xy[:,1]>=2)&(xy[:,1]<=5)&(abs(xy[:,0])<3)
 sides=[]
 for side in [-1,1]:
  points=xy[valid & (xy[:,0]*side>.25)];med=[]
  for y in np.arange(2,5,.1):
   q=points[(points[:,1]>=y)&(points[:,1]<y+.1)]
   if len(q)>=5:med.append([y+.05,float(np.median(q[:,0]))])
  if len(med)<15:sides.append(None);continue
  a=np.array(med);fit=np.polyfit(a[:,0],a[:,1],1);err=np.sqrt(np.mean((a[:,1]-np.polyval(fit,a[:,0]))**2));sides.append(dict(angle_deg=float(np.degrees(np.arctan(fit[0]))),rms_m=float(err),bins=len(med)))
 if all(sides):results.append(dict(session=rec['session'],frame=rec['index'],sides=sides,angle_difference_deg=abs(sides[0]['angle_deg']-sides[1]['angle_deg'])))
(out/'parallel_diagnostic.json').write_text(json.dumps(dict(note='Unlabelled yellow paint split by camera-ground X, 2..5m. Includes curbs/curves. Not semantic lane accuracy.',frames=results),indent=2))
print(json.dumps(results,indent=2))
