"""Offline ground-metric raster/tracker. Original pixel coordinates stay intact."""
from dataclasses import dataclass
import cv2
import numpy as np
from .ground_plane import estimate,rays
from .bev_lane import BEVConfig,sliding_curves,choose_pair
from .ground_candidates import CandidateTracks, pair_tracks


@dataclass(frozen=True)
class GroundConfig:
    min_track_frames: int = 3
    max_lane_heading_rad: float = .8
    max_lane_curvature_inv_m: float = .5
    roi_top: float = .35
    resolution_m: float = .01
    half_width_m: float = 3.
    near_m: float = .7
    far_m: float = 4.
    ground_tolerance_m: float = .08
    window_margin_m: float = .30
    min_marking_m: float = .025
    max_marking_m: float = .20
    fit_residual_m: float = .06
    support_ratio: float = .55
    # Acceptance envelope only, NEVER used to infer a missing lane.
    pair_width_min_m: float = .8
    pair_width_max_m: float = 4.5
    max_pair_heading_rad: float = .2
    max_pair_curvature_inv_m: float = .3
    max_center_jump_m: float = .2
    max_heading_jump_rad: float = .15
    max_curvature_jump_inv_m: float = .2

    def __post_init__(self):
        if not isinstance(self.min_track_frames,int) or self.min_track_frames<2:raise ValueError('Invalid persistence')
        if min(self.max_lane_heading_rad,self.max_lane_curvature_inv_m)<=0:raise ValueError('Invalid lane geometry')
        if not 0<=self.roi_top<1 or not 0<self.resolution_m<=.1:
            raise ValueError('Invalid ROI/resolution')
        if not 0<self.near_m<self.far_m or self.half_width_m<=0 or self.ground_tolerance_m<=0:
            raise ValueError('Invalid ground range')
        if not np.isfinite(list(vars(self).values())).all():raise ValueError('Nonfinite config')
        if not 0 < self.min_marking_m < self.max_marking_m or not 0 < self.pair_width_min_m < self.pair_width_max_m:
            raise ValueError('Invalid marking/pair envelope')
        if not 0 < self.support_ratio <= 1 or min(self.window_margin_m, self.fit_residual_m,
                self.max_pair_heading_rad, self.max_pair_curvature_inv_m,
                self.max_center_jump_m, self.max_heading_jump_rad, self.max_curvature_jump_inv_m) <= 0:
            raise ValueError('Invalid tracker limits')
        width=round(2*self.half_width_m/self.resolution_m)
        height=round((self.far_m-self.near_m)/self.resolution_m)
        if min(width,height)<22 or width*height>4_000_000:raise ValueError('Invalid raster dimensions')


def raster(frame,depth,camera,scale,plane,cfg,return_visibility=False,lane_colors='yellow'):
    h,w=frame.shape[:2]
    if depth.shape!=(h,w):raise ValueError('Aligned size mismatch')
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    # Existing thresholds, no colour widening. White retained as diagnostics only.
    yellow=cv2.inRange(hsv,(5,25,60),(40,255,255))
    white=cv2.inRange(hsv,(0,0,170),(179,65,255))
    n=np.asarray(plane['normal']);d=plane['offset_m']
    yy,xx=np.mgrid[:h,:w];rr=rays(np.c_[xx.ravel(),yy.ravel()],camera).reshape(h,w,3)
    z=depth.astype(float)*scale
    measured=(z>.1)&(z<15)
    residual=np.abs(np.sum(rr*n,axis=2)*z+d)
    excluded=measured&(residual>cfg.ground_tolerance_m)
    roi=yy>=int(h*cfg.roi_top)
    yellow[~roi|excluded]=0;white[~roi|excluded]=0
    forward=np.array([0.,0.,1.])-n*n[2];forward/=np.linalg.norm(forward)
    right=np.cross(n,forward);right/=np.linalg.norm(right)
    origin=-d*n
    width=int(round(2*cfg.half_width_m/cfg.resolution_m))
    height=int(round((cfg.far_m-cfg.near_m)/cfg.resolution_m))
    by,bx=np.mgrid[:height,:width]
    x=(bx-width/2)*cfg.resolution_m;y=cfg.far_m-by*cfg.resolution_m
    xyz=origin+x[:,:,None]*right+y[:,:,None]*forward
    uv,_=cv2.projectPoints(xyz.reshape(-1,3),np.zeros(3),np.zeros(3),
                           np.asarray(camera['k']).reshape(3,3),np.asarray(camera['d']))
    uv=uv.reshape(height,width,2).astype(np.float32)
    visible=(xyz[:,:,2]>.1)&(uv[:,:,0]>=0)&(uv[:,:,0]<=w-1)&(uv[:,:,1]>=int(h*cfg.roi_top))&(uv[:,:,1]<=h-1)
    # Nearest categorical lookup: no dilation/closing that joins nearby paint.
    yellow_bev=cv2.remap(yellow,uv[:,:,0],uv[:,:,1],cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
    white_bev=cv2.remap(white,uv[:,:,0],uv[:,:,1],cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)

    if lane_colors == 'yellow':
        mask = yellow_bev
    elif lane_colors == 'yellow_white':
        mask = cv2.bitwise_or(yellow_bev, white_bev)
    elif lane_colors == 'white':
        mask = white_bev
    else:
        raise ValueError('Invalid lane_colors')

    mask[~visible]=0
    debug=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    debug[(white_bev>0)&(mask==0)]=(80,80,80)
    debug[~visible]=(30,30,30)
    unknown=cv2.remap((~measured).astype(np.uint8),uv[:,:,0],uv[:,:,1],cv2.INTER_NEAREST)>0
    output=(mask,debug,dict(ground_rejected_pixels=int(np.sum(excluded&roi)),
        yellow_pixels=int(np.count_nonzero(yellow)),unknown_depth_bev_pixels=int(np.sum(unknown&(mask>0))),
        white_pixels=int(np.count_nonzero(white)),
        visible_bev_pixels=int(visible.sum()),
        occupied_rows=int(np.count_nonzero(np.any(mask>0,axis=1)))))
    return (*output,visible) if return_visibility else output


class GroundTracker:
    def __init__(self,cfg=GroundConfig()):
        self.cfg=cfg;self.previous=None;self.stamp=None;self.tracks=CandidateTracks()

    def process(self,frame,depth,camera,scale,stamp):
        cfg=self.cfg
        try:
            plane,_,_=estimate(depth,camera,scale,max_depth_m=cfg.far_m)
        except ValueError:
            self.previous=None;self.stamp=None;self.tracks.reset()
            raise
        result=dict(timestamp_ns=int(stamp),plane=plane,detected=False,steering_valid=False,
                    metric_control_enabled=False,center=None,confidence=0.,status='plane_rejected')
        if not plane['accepted']:
            self.previous=None;self.stamp=None;self.tracks.reset();return result,np.zeros((round((cfg.far_m-cfg.near_m)/cfg.resolution_m),round(2*cfg.half_width_m/cfg.resolution_m),3),np.uint8)
        mask,debug,stats,visible=raster(frame,depth,camera,scale,plane,cfg,return_visibility=True);result.update(stats)
        h,w=mask.shape
        # Ground-grid geometry: each pixel is 1cm with this default profile.
        # Experimental physical ranges; not measured actual lane width.
        params=BEVConfig(width=w,height=h,undistort=False,lane_colors='yellow',
            windows=11,multi_height_seeds=True,partial_support_ratio=cfg.support_ratio,margin_ratio=cfg.window_margin_m/(w*cfg.resolution_m),
            min_marking_width_ratio=cfg.min_marking_m/(w*cfg.resolution_m),
            max_marking_width_ratio=cfg.max_marking_m/(w*cfg.resolution_m),
            residual_ratio=cfg.fit_residual_m/(w*cfg.resolution_m),min_support_ratio=cfg.support_ratio,
            max_gap_ratio=.15,lane_width_min_ratio=cfg.pair_width_min_m/(w*cfg.resolution_m),
            lane_width_max_ratio=cfg.pair_width_max_m/(w*cfg.resolution_m),lane_width_variation_ratio=.2)
        baseline_logs=[];baseline_pair_logs=[]
        _,baseline_curves,_=sliding_curves(mask,params,baseline_logs)
        baseline_pair=choose_pair(baseline_curves,params,baseline_pair_logs)
        result['baseline']=dict(curve_count=len(baseline_curves),pair=baseline_pair is not None,
                                candidate_rejections=baseline_logs,pair_rejections=baseline_pair_logs)
        curves,logs,reset=self.tracks.process(mask,visible,cfg,stamp)
        result.update(curves=curves,lanes=[c for c in curves if c['confirmed']],
                      candidate_rejections=logs,temporal_reset=reset)
        for c in curves:
            ys=np.linspace(c['y_min'],c['y_max'],100);xs=np.polyval(c['coefficients'],ys/(h-1))
            cv2.polylines(debug,[np.c_[np.clip(xs,0,w-1),ys].astype(np.int32)],False,
                          (0,180,255) if c['confirmed'] else (120,100,0),2)
        pair,pair_logs=pair_tracks(curves,(h,w),cfg)
        result['pair_rejections']=pair_logs
        result['pair_failure_category']=('no_candidates' if not curves else 'one_candidate' if len(curves)==1
            else 'unconfirmed_candidates' if len(result['lanes'])<2 else 'pair_conditions_failed' if pair is None else 'accepted')
        if pair is None:
            self.previous=None;self.stamp=stamp
            result['status']='single_lane_tracking' if len(result['lanes'])==1 else 'lanes_without_pair' if result['lanes'] else 'candidates_pending' if curves else 'no_lane_candidates'
            return result,debug
        score,left,right,observed_low,observed_high=pair;center=(left+right)/2
        result['center_observed_y_range']=[observed_low,observed_high]
        if self.stamp is not None and (stamp<=self.stamp or stamp-self.stamp>300_000_000):self.previous=None
        if self.previous is not None:
            jump=np.max(np.abs(np.polyval(center-self.previous,np.linspace(0,1,20))))*cfg.resolution_m
            result['center_jump_m']=float(jump)
            def geometry(coeff):
                slope=np.polyval(np.polyder(coeff),np.linspace(0,1,20))/(h-1)
                return np.arctan(slope),-2*coeff[0]/((h-1)**2*cfg.resolution_m)/(1+slope*slope)**1.5
            heading,curvature=geometry(center);old_heading,old_curvature=geometry(self.previous)
            result['heading_jump_rad']=float(np.max(np.abs(heading-old_heading)))
            result['curvature_jump_inv_m']=float(np.max(np.abs(curvature-old_curvature)))
            if (jump>cfg.max_center_jump_m or result['heading_jump_rad']>cfg.max_heading_jump_rad
                    or result['curvature_jump_inv_m']>cfg.max_curvature_jump_inv_m):
                self.previous=None;self.stamp=stamp;result['status']='temporal_jump';result['pair_rejections'].append(dict(reason='center_temporal_failed'));return result,debug
            center=.4*center+.6*self.previous
        self.previous=center;self.stamp=stamp
        result.update(center=center.tolist(),status='visual_pair_only',
                      confidence=float(score*plane['inlier_ratio']*max(0,1-plane['rms_m']/.025)))
        # Camera-ground diagnostics only; NOT rear-axle e_y or controller output.
        t=(observed_low+observed_high)/2/(h-1);slope=np.polyval(np.polyder(center),t)/(h-1)
        result.update(camera_ground_offset_m=float((w/2-np.polyval(center,t))*cfg.resolution_m),
                      camera_ground_heading_rad=float(np.arctan(slope)),
                      camera_ground_curvature_inv_m=float(-2*center[0]/((h-1)**2*cfg.resolution_m)/(1+slope*slope)**1.5))
        ys=np.arange(observed_low,observed_high+1);xs=np.polyval(center,ys/(h-1));cv2.polylines(debug,[np.c_[np.clip(xs,0,w-1),ys].astype(np.int32)],False,(0,255,0),2)
        return result,debug
