import numpy as np
import pytest
from fma_perception.ground_plane import estimate,intersect,rays

CAM=dict(width=640,height=480,k=[606.,0.,322.,0.,606.,244.,0.,0.,1.],
         d=[0.]*5,distortion_model='plumb_bob')


def test_depth_plane_with_obstacles_and_holes():
    yy,xx=np.mgrid[:480,:640]
    normal=np.array([.01,1.,.025]);normal/=np.linalg.norm(normal)
    rr=rays(np.c_[xx.ravel(),yy.ravel()],CAM)
    z=np.divide(.46,rr@normal,out=np.zeros(len(rr)),where=rr@normal>.04).reshape(480,640)
    depth=np.rint(z*1000).astype(np.uint16)
    depth[310:360,200:280]=600
    depth[380:400,310:370]=0
    original=depth.copy()
    plane,_,_=estimate(depth,CAM,.001)
    assert plane['accepted']
    np.testing.assert_allclose(plane['normal'],normal,atol=.003)
    assert plane['camera_height_m']==pytest.approx(.46,abs=.003)
    assert plane['inlier_ratio']<1
    assert np.array_equal(depth,original)
    assert plane['roi_xyxy']==[0,264,639,479]


def test_invalid_roi_rejected():
    with pytest.raises(ValueError):
        estimate(np.zeros((480,640),np.uint16),CAM,.001,roi_left=.9,roi_right=.1)


def test_lane_ray_intersection_requires_no_lane_depth():
    plane=dict(normal=[0.,1.,0.],offset_m=-.5)
    xy,valid=intersect([[322,345],[423,345],[322,244],[322,200]],CAM,plane)
    assert valid.tolist()==[True,True,False,False]
    assert xy[0,0]==pytest.approx(0)
    assert xy[0,1]==pytest.approx(3)
    assert xy[1,0]==pytest.approx(.5)


def test_empty_and_invalid_depth_rejected():
    with pytest.raises(ValueError):estimate(np.zeros((480,640),np.uint16),CAM,.001)
    with pytest.raises(ValueError):estimate(np.zeros((240,320),np.uint16),CAM,.001)
    with pytest.raises(ValueError):estimate(np.zeros((480,640),np.uint16),CAM,-.001)
