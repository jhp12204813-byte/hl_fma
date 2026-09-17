"""Depth-derived local road plane; camera-ground coordinates, not vehicle frame."""
import cv2
import numpy as np


def rays(pixels, camera):
    if len(pixels)==0:return np.empty((0,3),float)
    k=np.asarray(camera['k'],float).reshape(3,3)
    d=np.asarray(camera['d'],float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or d.size not in (4,5,8,12,14) or min(k[0,0],k[1,1])<=0:
        raise ValueError('Invalid intrinsics')
    if camera['distortion_model'] not in ('plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported distortion model')
    xy=cv2.undistortPoints(np.asarray(pixels,np.float64).reshape(-1,1,2),k,d).reshape(-1,2)
    return np.c_[xy,np.ones(len(xy))]


def estimate(depth, camera, scale, tolerance=.025, iterations=400, roi_left=0., roi_right=1., max_depth_m=8.):
    if depth.dtype!=np.uint16 or depth.shape!=(camera['height'],camera['width']):
        raise ValueError('Expected aligned uint16 depth at CameraInfo resolution')
    if not np.isfinite(scale) or scale<=0:raise ValueError('Invalid depth scale')
    if not np.isfinite(tolerance) or tolerance<=0 or iterations<1:raise ValueError('Invalid RANSAC parameters')
    h,w=depth.shape
    if not 0 <= roi_left < roi_right <= 1:raise ValueError('Invalid horizontal ROI')
    # Full-width lower road ROI. RANSAC rejects curb/obstacle depth outliers.
    left,right=int(w*roi_left),int(w*roi_right)
    xs=np.unique(np.r_[np.arange(left,right,4),right-1])
    yy,xx=np.meshgrid(np.arange(int(h*.55),h,4),xs,indexing='ij')
    z=depth[yy,xx].ravel()*scale
    if not np.isfinite(max_depth_m) or max_depth_m<=.2:raise ValueError('Invalid depth range')
    valid=(z>.2)&(z<max_depth_m)
    pixels=np.c_[xx.ravel(),yy.ravel()][valid]
    points=rays(pixels,camera)*z[valid,None]
    if len(points)<200:raise ValueError('Insufficient road depth')
    rng=np.random.default_rng(7);best=None;score=0
    for _ in range(iterations):
        a,b,c=points[rng.choice(len(points),3,replace=False)]
        n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-8:continue
        n/=length
        if n[1]<0:n=-n
        # Road normal approximately optical down; rejects vertical cars/walls.
        if n[1]<.65:continue
        offset=-n@a
        if offset>=-.08:continue
        keep=np.abs(points@n+offset)<tolerance
        if keep.sum()>score:best=keep;score=keep.sum()
    if best is None:raise ValueError('No ground-like plane')
    for _ in range(3):
        centroid=points[best].mean(axis=0)
        _,_,v=np.linalg.svd(points[best]-centroid,full_matrices=False)
        n=v[-1]
        if n[1]<0:n=-n
        offset=-n@centroid;best=np.abs(points@n+offset)<tolerance
        if best.sum()<100:raise ValueError('Plane refinement failed')
    residual=np.abs(points@n+offset)
    occupied=[]
    for col in range(3):
        selected=(pixels[:,0]>=left+(right-left)*col/3)&(pixels[:,0]<left+(right-left)*(col+1)/3)
        occupied.append(int(np.sum(best&selected)))
    ok=best.mean()>=.5 and min(occupied)>=30 and n[1]>=.65 and offset<-.08
    return dict(normal=n.tolist(),offset_m=float(offset),camera_height_m=float(-offset),
                roi_xyxy=[left,int(h*.55),right-1,h-1],
                support_z_range_m=np.percentile(points[best,2],[5,95]).tolist(),
                inlier_ratio=float(best.mean()),rms_m=float(np.sqrt(np.mean(residual[best]**2))),
                p95_m=float(np.percentile(residual[best],95)),points=len(points),
                spatial_inliers=occupied,accepted=bool(ok)),pixels,best


def intersect(pixels,camera,plane):
    n=np.asarray(plane['normal']);d=plane['offset_m'];r=rays(pixels,camera)
    if not np.isfinite(n).all() or not np.isfinite(d) or not np.isclose(np.linalg.norm(n),1):
        raise ValueError('Invalid unit plane')
    denom=r@n
    distance=np.divide(-d,denom,out=np.full(len(r),np.nan),where=np.abs(denom)>1e-6)
    valid=np.isfinite(distance)&(distance>.1)&(distance<15)
    xyz=r*distance[:,None]
    # Forward = optical Z projected onto road; right-handed right/forward basis.
    forward=np.array([0.,0.,1.])-n*n[2];forward/=np.linalg.norm(forward)
    right=np.cross(n,forward);right/=np.linalg.norm(right)
    origin=-d*n
    coordinates=np.c_[(xyz-origin)@right,(xyz-origin)@forward]
    return coordinates,valid
