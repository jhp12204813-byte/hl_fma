import json
import time
import threading
import numpy as np
from c920_recorder.writer import Writer,dump
from c920_recorder.verification import verify
from c920_recorder.stop_line import candidates


def record(tmp_path,n=8):
    w=Writer(tmp_path,dict(width=160,height=120,fps=30.),capacity=16)
    image=np.zeros((120,160,3),np.uint8)
    image[85:95,20:140]=255
    for i in range(n):
        assert w.submit(image,dict(timestamp_ns=1_000_000_000+i*33_333_333,frame_id='rgb',
            receive_monotonic_ns=2_000_000_000+i*33_333_333,receive_wall_ns=3_000_000_000+i*33_333_333))
    w.close(dict(automatic=False,monotonic_ns=2_000_000_000+n*33_333_333))
    return w


def test_rgb_only_integrity_and_no_overlay(tmp_path):
    import cv2
    w=record(tmp_path);r=verify(w.path)
    assert r['status']=='PASS' and r['decoded_frames']==8
    rows=[json.loads(x) for x in (w.path/'stop_lines.jsonl').read_text().splitlines()]
    assert rows[0]['distance_available'] is False and rows[0]['candidates']
    cap=cv2.VideoCapture(str(w.path/'color.mp4'));ok,img=cap.read();cap.release()
    assert ok and np.max(np.abs(img.astype(int)[:,:,0]-img.astype(int)[:,:,1]))<15
    assert not (w.path/'depth').exists()


def test_duplicate_and_corrupt_fail(tmp_path):
    w=record(tmp_path);p=w.path/'frames.jsonl';rows=[json.loads(x) for x in p.read_text().splitlines()]
    rows[1]['timestamp_ns']=rows[0]['timestamp_ns'];p.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    assert any('duplicate' in x for x in verify(w.path)['failures'])
    (w.path/'color.mp4').write_bytes(b'bad')
    assert verify(w.path)['status']=='FAIL'


def test_stop_schema_and_timeout_fail(tmp_path):
    w=record(tmp_path);(w.path/'stop_lines.jsonl').write_text('{}\n')
    assert verify(w.path)['status']=='FAIL'
    s=w.metadata;s['stop']=dict(automatic=True,message='AUTO STOP: RGB timeout');dump(w.path/'session.json',s)
    assert any('AUTO STOP' in x for x in verify(w.path)['failures'])


def test_yellow_excluded_white_kept():
    a=np.zeros((120,160,3),np.uint8);a[85:95,20:140]=(0,255,255)
    assert not candidates(a)
    a[85:95,20:140]=255
    assert candidates(a)


def test_writer_failure_drains_without_deadlock(tmp_path):
    w=Writer(tmp_path,dict(width=160,height=120,fps=30.),capacity=1)
    w.submit(np.zeros((1,1,3),np.uint8),{})
    t=threading.Thread(target=w.close,args=({'automatic':True},));t.start();t.join(5)
    assert not t.is_alive() and w.errors
    assert verify(w.path)['status']=='FAIL'


def test_queue_overflow_recorded(tmp_path,monkeypatch):
    gate=threading.Event();original=Writer._run
    def wait_run(self):gate.wait(5);original(self)
    monkeypatch.setattr(Writer,'_run',wait_run)
    w=Writer(tmp_path,dict(width=160,height=120,fps=30.),capacity=1)
    frame=np.zeros((120,160,3),np.uint8)
    meta=dict(timestamp_ns=1,frame_id='rgb',receive_monotonic_ns=1,receive_wall_ns=1)
    assert w.submit(frame,meta)
    assert not w.submit(frame,meta)
    gate.set();w.close(dict(automatic=True))
    assert w.overflow==1 and verify(w.path)['status']=='FAIL'


def test_monotonic_watchdog_and_startup_readiness(tmp_path):
    from c920_recorder.recording import Recorder
    class Node:
        def create_subscription(self,*args):return object()
        def create_timer(self,*args):return object()
        def count_publishers(self,*args):return 1
        def destroy_timer(self,*args):pass
        def destroy_subscription(self,*args):pass
    rec=Recorder(Node(),root=tmp_path,width=160,height=120)
    import pytest
    with pytest.raises(RuntimeError):rec.start()
    msg=rec.bridge.cv2_to_imgmsg(np.zeros((120,160,3),np.uint8),'bgr8')
    msg.header.stamp.sec=1;msg.header.frame_id='rgb'
    rec.receive(msg);rec.start();rec.receive(msg)
    rec.poll();assert rec.writer is not None
    rec.started=time.monotonic()-4;rec.last_seen=time.monotonic()-4
    rec.poll();rec.finalizer.join(5)
    assert not rec.finishing and rec.result['status']=='FAIL'
    assert rec.stop_details['automatic'] and 'RGB timeout' in rec.stop_details['message']
    rec.shutdown()


def test_long_gap_and_frame_count_mismatch(tmp_path):
    w=record(tmp_path);p=w.path/'frames.jsonl';rows=[json.loads(x) for x in p.read_text().splitlines()]
    for row in rows[4:]:row['timestamp_ns']+=3_000_000_000
    p.write_text(''.join(json.dumps(x)+'\n' for x in rows))
    assert any('gap >= 3' in x for x in verify(w.path)['failures'])
    p.write_text(''.join(json.dumps(x)+'\n' for x in rows[:-1]))
    assert any('count mismatch' in x for x in verify(w.path)['failures'])
