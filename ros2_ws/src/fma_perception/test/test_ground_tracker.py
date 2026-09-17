import cv2
import numpy as np
import pytest
from fma_perception.ground_plane import rays
from fma_perception.ground_tracker import GroundConfig, GroundTracker, raster

CAM=dict(width=640,height=480,k=[606.,0.,322.,0.,606.,244.,0.,0.,1.],
         d=[0.]*5,distortion_model='plumb_bob')
PLANE=dict(normal=[0.,1.,0.],offset_m=-.46)


def fixture():
    yy,xx=np.mgrid[:480,:640]
    rr=rays(np.c_[xx.ravel(),yy.ravel()],CAM).reshape(480,640,3)
    z=np.divide(.46,rr[:,:,1],out=np.zeros((480,640)),where=rr[:,:,1]>.04)
    depth=np.rint(z*1000).astype(np.uint16)
    x=rr[:,:,0]*z
    paint=(np.abs(np.abs(x)-.6)<.04)&(z>1)&(z<4.2)
    frame=np.zeros((480,640,3),np.uint8);frame[paint]=(0,255,255)
    return frame,depth,paint


def test_inverse_continuity_and_original_coordinates():
    frame,depth,_=fixture();original=frame.copy();k=CAM['k'][:]
    mask,_,_=raster(frame,depth,CAM,.001,PLANE,GroundConfig())
    # A real continuous stripe remains continuous in its visible 1.5..4m range.
    assert np.mean(np.any(mask[:250,236:245]>0,axis=1))>.95
    assert np.array_equal(frame,original) and CAM['k']==k


def test_obstacle_rejected_but_missing_lane_depth_preserved():
    frame,depth,paint=fixture()
    baseline,_,_=raster(frame,depth,CAM,.001,PLANE,GroundConfig())
    depth[paint]=500
    rejected,_,_=raster(frame,depth,CAM,.001,PLANE,GroundConfig())
    assert np.count_nonzero(rejected)<.05*np.count_nonzero(baseline)
    depth[paint]=0
    missing,_,stats=raster(frame,depth,CAM,.001,PLANE,GroundConfig())
    assert np.count_nonzero(missing)>.95*np.count_nonzero(baseline)
    assert stats['unknown_depth_bev_pixels']>0


def test_upper_roi_and_config():
    frame,depth,_=fixture();frame[:]=0;frame[:168]=(0,255,255)
    assert not raster(frame,depth,CAM,.001,PLANE,GroundConfig())[0].any()
    with pytest.raises(ValueError):GroundConfig(resolution_m=1e-8)


def test_pair_then_loss_and_invalid_depth_clear_state():
    frame,depth,_=fixture();tracker=GroundTracker()
    for i in range(3):
        result,_=tracker.process(frame,depth,CAM,.001,1_000_000_000+i*33_000_000)
    assert result['center'] is not None
    assert not result['steering_valid'] and not result['detected']
    result,_=tracker.process(np.zeros_like(frame),depth,CAM,.001,1_099_000_000)
    assert result['center'] is None and tracker.previous is None
    tracker.previous=np.ones(3)
    with pytest.raises(ValueError):tracker.process(frame,np.zeros_like(depth),CAM,.001,2_000_000_000)
    assert tracker.previous is None


def test_temporal_center_jump_rejected_without_stale_center():
    frame,depth,_=fixture();tracker=GroundTracker()
    for i in range(3):
        tracker.process(frame,depth,CAM,.001,1_000_000_000+i*33_000_000)
    assert tracker.previous is not None
    # A previous fit displaced 30cm must not be silently smoothed into this frame.
    tracker.previous[2]+=30
    result,_=tracker.process(frame,depth,CAM,.001,1_099_000_000)
    assert result['status']=='temporal_jump'
    assert result['center'] is None and tracker.previous is None


def test_inverse_fills_sampling_holes_without_widening_paint():
    from fma_perception.ground_plane import intersect
    frame,depth,paint=fixture()
    mask,_,_=raster(frame,depth,CAM,.001,PLANE,GroundConfig())
    y,x=np.nonzero(paint);xy,valid=intersect(np.c_[x,y],CAM,PLANE)
    bx=np.rint(xy[:,0]/.01+300).astype(int);by=np.rint((4-xy[:,1])/.01).astype(int)
    valid&=(bx>=0)&(bx<600)&(by>=0)&(by<330)
    forward=np.zeros_like(mask);forward[by[valid],bx[valid]]=255
    assert np.count_nonzero(np.any(mask[:150]>0,axis=1))>np.count_nonzero(np.any(forward[:150]>0,axis=1))
    # Two eight-centimetre stripes; inverse sampling must not merge or broaden them.
    assert np.max(np.count_nonzero(mask[:150],axis=1))<=20
