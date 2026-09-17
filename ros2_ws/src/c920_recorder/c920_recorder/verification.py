"""Offline RGB integrity checks. No absent-stream checks for RGB-only cameras."""
import json
from pathlib import Path
import cv2
import numpy as np
from .writer import dump


def verify(directory):
    path=Path(directory);failures=[];warnings=[];timing={}
    def read(name,lines=False):
        try:
            text=(path/name).read_text()
            return [json.loads(x) for x in text.splitlines()] if lines else json.loads(text)
        except Exception as exc:
            failures.append(f'{name}: {exc}');return [] if lines else {}
    session=read('session.json');frames=read('frames.jsonl',True);stops=read('stop_lines.jsonl',True)
    if not isinstance(session,dict):session={};failures.append('session object invalid')
    if session.get('state')!='closed':failures.append('session not closed')
    if session.get('writer_errors') or session.get('queue_overflow',0):failures.append('writer error/queue overflow')
    if session.get('stop',{}).get('automatic'):failures.append(session['stop'].get('message','automatic stop'))
    if not frames:failures.append('empty frames.jsonl')
    try:
        for i,row in enumerate(frames):
            if row['index']!=i or not isinstance(row['frame_id'],str):raise ValueError('index/frame_id')
            for key in ('timestamp_ns','receive_monotonic_ns','receive_wall_ns'):
                if type(row[key]) is not int or row[key]<=0:raise ValueError(key)
        stamps=np.array([r['timestamp_ns'] for r in frames],dtype=np.int64)
        received=np.array([r['receive_monotonic_ns'] for r in frames],dtype=np.int64)
        for label,values in [('timestamp',stamps),('receive',received)]:
            dt=np.diff(values)/1e9
            if np.any(dt<=0):failures.append(f'{label}: reversed/duplicate timestamp')
            maximum=float(dt.max()) if len(dt) else None
            span=float((values[-1]-values[0])/1e9) if len(values)>1 else 0
            timing[label]=dict(average_fps=(len(values)-1)/span if span>0 else None,max_gap_s=maximum)
            if maximum is not None and maximum>=3:failures.append(f'{label}: RGB gap >= 3 seconds')
            fps=float(session['fps'])
            if not np.isfinite(fps) or fps<=0:raise ValueError('configured FPS')
            timing[label]['intervals_over_1_5_periods']=int(np.sum(dt>1.5/fps))
            timing[label]['estimated_missing_periods']=int(np.maximum(np.rint(dt*fps)-1,0).sum())
            if np.any(dt>1.5/fps):warnings.append(f'{label}: frame gaps above 1.5 configured periods')
        stop=session.get('stop',{})
        if len(received) and stop.get('monotonic_ns',int(received[-1]))-int(received[-1])>=3_000_000_000:
            failures.append('RGB ended >= 3 seconds before stop')
    except (KeyError,TypeError,ValueError,OverflowError) as exc:failures.append(f'frame metadata invalid: {exc}')
    cap=cv2.VideoCapture(str(path/'color.mp4'));decoded=0
    try:
        while True:
            ok,image=cap.read()
            if not ok:break
            decoded+=1
            if image.shape!=(session.get('height'),session.get('width'),3):failures.append('MP4 resolution mismatch')
    finally:cap.release()
    if not decoded or decoded!=len(frames):failures.append('MP4 empty/corrupt/count mismatch')
    if session.get('stream_counts',{}).get('rgb')!=len(frames):failures.append('session RGB count mismatch')
    if len(stops)!=len(frames):failures.append('stop_lines count mismatch')
    for i,row in enumerate(stops):
        try:
            if row['index']!=i or row['timestamp_ns']!=frames[i]['timestamp_ns'] or row['distance_available'] is not False:
                raise ValueError('index/timestamp/distance flag')
            if any(k in row for k in ('distance_m','optical_z_m')):raise ValueError('unexpected distance')
            if not isinstance(row['candidates'],list):raise ValueError('candidate list')
            for box in row['candidates']:
                if len(box)!=4 or not all(type(x) is int for x in box):raise ValueError('bbox type')
                x,y,w,h=box
                if min(x,y)<0 or min(w,h)<=0 or x+w>session['width'] or y+h>session['height']:raise ValueError('bbox bounds')
        except (KeyError,TypeError,ValueError,IndexError) as exc:failures.append(f'stop_lines[{i}]: {exc}')
    result=dict(status='FAIL' if failures else 'PASS',failures=sorted(set(failures)),warnings=sorted(set(warnings)),
                decoded_frames=decoded,metadata_frames=len(frames),timing=timing,stop=session.get('stop'))
    dump(path/'verification.json',result);return result


def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    result=verify(args.directory);print(json.dumps(result,indent=2))
    if result['status']=='FAIL':raise SystemExit(1)
