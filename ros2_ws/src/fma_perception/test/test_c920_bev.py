import importlib.util
from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml
from fma_perception.c920_bev import C920BEV, DEFAULT_GRID, fit_calibration, ground_to_bev_matrix, project, warp_to_bev


def fixtures():
    groups=[('20260911_211455',[(0,1),(0,2),(0,3),(0,4)]),
            ('20260911_211722',[(-1,2),(-1,3),(-1,4)]),
            ('20260911_211856',[(1,2),(1,3),(1,4)])]
    g2i=np.array([[120,20,320],[0,20,420],[0,.2,1.]])
    points=[]
    for session,xy in groups:
        for ground,uv in zip(xy,project(xy,g2i)):
            points.append(dict(number=len(points)+1,session=session,image_pixel=uv.tolist(),
                ground_coordinate_m=list(ground),frame_index=1,selection='manual'))
    return points


def test_metric_axes_and_inverse_roundtrip(tmp_path):
    matrix,size=ground_to_bev_matrix(DEFAULT_GRID)
    assert size==(600,530)
    np.testing.assert_allclose(project([[0,6],[0,4],[-1,2],[1,2]],matrix),
                               [[300,0],[300,200],[200,400],[400,400]])
    points=fixtures();cal=fit_calibration(points,640,480)
    assert cal['reprojection_error_m']['max']<1e-5
    expected=np.array([p['image_pixel'] for p in points])
    ground=project(expected,cal['homography_image_to_ground'])
    np.testing.assert_allclose(project(ground,cal['homography_ground_to_image']),expected,atol=1e-6)
    path=tmp_path/'calibration.yaml';path.write_text(yaml.safe_dump(cal))
    image=np.zeros((480,640,3),np.uint8);original=image.copy()
    assert warp_to_bev(image,path).shape==(530,600,3)
    assert np.array_equal(image,original)
    with pytest.raises(ValueError,match='original image size'):
        C920BEV.from_yaml(path).warp_to_bev(image[:400])


def test_ransac_reports_outlier_in_all_point_errors():
    points=fixtures();points[-1]['image_pixel'][0]+=90
    cal=fit_calibration(points,640,480)
    assert not cal['calibration_points'][-1]['ransac_inlier']
    assert cal['reprojection_error_m']['per_point'][-1]['error_m']>.5
    assert cal['reprojection_error_m']['max']>.5


def test_degenerate_and_inconsistent_calibrations_rejected():
    points=fixtures()
    for i,p in enumerate(points):p['image_pixel']=[float(i),50.]
    with pytest.raises(ValueError,match='Collinear'):fit_calibration(points,640,480)
    cal=fit_calibration(fixtures(),640,480);cal['homography_ground_to_image']=np.eye(3).tolist()
    with pytest.raises(ValueError,match='disagree'):C920BEV(cal)


def tool():
    path=Path(__file__).resolve().parents[1]/'tools/calibrate_c920_bev.py'
    spec=importlib.util.spec_from_file_location('calibrate_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_artifact_export_has_points_and_preview(tmp_path):
    module=tool();cal=fit_calibration(fixtures(),640,480)
    chosen={session:(1,np.full((480,640,3),100,np.uint8)) for session,_ in module.SESSIONS}
    module.save_results(tmp_path,cal,chosen)
    reloaded=C920BEV.from_yaml(tmp_path/'config/c920_bev_calibration.yaml')
    assert reloaded.output_size==(600,530)
    for session in chosen:
        raw=cv2.imread(str(tmp_path/f'{session}_bev_raw.png'))
        preview=cv2.imread(str(tmp_path/f'{session}_bev_preview.png'))
        assert raw.shape==preview.shape==(530,600,3)
        assert not np.array_equal(raw[:200],preview[:200])
        assert (tmp_path/f'{session}_frame_1_debug.png').is_file()
    with pytest.raises(FileExistsError):module.save_results(tmp_path,cal,chosen)


def test_click_ui_navigation_and_original_pixel_coordinates(tmp_path,monkeypatch):
    module=tool();callbacks={};actions=[]
    class Capture:
        def __init__(self,*args):self.position=0
        def set(self,key,index):self.position=index
        def read(self):self.position+=1;return True,np.zeros((960,1280,3),np.uint8)
        def get(self,key):return self.position
        def release(self):pass
    monkeypatch.setattr(module.cv2,'VideoCapture',Capture)
    for name in ['namedWindow','imshow','destroyAllWindows']:
        monkeypatch.setattr(module.cv2,name,lambda *a,**kw:None)
    monkeypatch.setattr(module.cv2,'getWindowProperty',lambda *a:1)
    monkeypatch.setattr(module.cv2,'setMouseCallback',lambda name,callback:callbacks.update(click=callback))
    videos=[]
    for session,coordinates in module.SESSIONS:
        videos.append(dict(session=session,video='synthetic-ui-only',frames=5,fps=30.,coordinates=coordinates))
        actions.extend(['click',ord('n'),ord('p'),13])  # Unlocked click must be ignored.
        actions.extend(['click']*len(coordinates));actions.extend([8,'click',13])
    def key(delay):
        event=actions.pop(0)
        if event=='click':
            callbacks['click'](cv2.EVENT_LBUTTONDOWN,480,140+360,0,None)
            return -1
        return event
    monkeypatch.setattr(module.cv2,'waitKeyEx',key)
    font=module.ImageFont.truetype(module.font_path(None),18)
    points,chosen=module.collect(videos,tmp_path,font)
    assert len(points)==10 and len(chosen)==3 and not actions
    assert [p['number'] for p in points]==list(range(1,11))
    assert [p['ground_coordinate_m'] for p in points]==[list(xy) for _,group in module.SESSIONS for xy in group]
    assert all(p['frame_index']==2 and p['image_pixel']==[640.,480.] for p in points)
