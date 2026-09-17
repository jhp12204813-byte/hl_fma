"""Guarded usb_cam launcher. Only processes created here are stopped on exit."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from .recording import Recorder


def select_device(requested=None):
    paths=[Path(requested)] if requested else sorted(Path('/sys/class/video4linux').glob('video*'))
    valid=[]
    for path in paths:
        device=path.resolve() if requested else Path('/dev')/path.name
        name_path=Path('/sys/class/video4linux')/device.name/'name'
        if not device.exists() or not name_path.exists():continue
        if 'C920' not in name_path.read_text():continue
        output=subprocess.run(['v4l2-ctl','-d',str(device),'--list-formats-ext'],capture_output=True,text=True,check=True).stdout
        if "'MJPG'" in output or "'YUYV'" in output:valid.append(str(device))
    if len(valid)!=1:raise RuntimeError(f'Expected one verified C920 capture device; found {valid}. Use --device.')
    return valid[0]


def busy(device):
    result=subprocess.run(['fuser',device],capture_output=True,text=True)
    if result.returncode not in (0,1):raise RuntimeError('Unable to check device ownership')
    return result.returncode==0


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--existing-camera',action='store_true');parser.add_argument('--device')
    parser.add_argument('--width',type=int,default=640);parser.add_argument('--height',type=int,default=480)
    parser.add_argument('--fps',type=float,default=30.);parser.add_argument('--format',choices=['mjpeg2rgb','yuyv'],default='mjpeg2rgb')
    parser.add_argument('--topic',default='/c920/image_raw');parser.add_argument('--root',default=str(Path.home()/'c920_recordings_ku'))
    parser.add_argument('--seconds',type=float);parser.add_argument('--queue-capacity',type=int,default=128)
    options=parser.parse_args(args)
    if min(options.width,options.height,options.fps,options.queue_capacity)<=0 or (options.seconds is not None and options.seconds<=0):parser.error('Dimensions/FPS/duration/capacity must be positive')
    rclpy.init(args=[]);node=rclpy.create_node('c920_recorder_launcher')
    camera=gui=rec=lock=None
    settings=dict(root=options.root,topic=options.topic,width=options.width,height=options.height,fps=options.fps,capacity=options.queue_capacity)
    try:
        # Discovery is read-only; existing publishers are never terminated or replaced.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not options.existing_camera:
            get_package_prefix('usb_cam')
            device=select_device(options.device)
            lock=open(Path.home()/('.c920_'+Path(device).name+'.lock'),'a')
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if busy(device) or node.count_publishers(options.topic):raise RuntimeError('Camera/device/topic already active. Use --existing-camera; no process was stopped.')
            command=['ros2','run','usb_cam','usb_cam_node_exe','--ros-args','-r','__node:=c920_camera',
                '-r','__ns:=/c920','-r','image_raw:='+options.topic,
                '-p','video_device:='+device,'-p','pixel_format:='+options.format,
                '-p','image_width:='+str(options.width),'-p','image_height:='+str(options.height),
                '-p','framerate:='+str(options.fps),'-p','frame_id:=c920_optical_frame','-p','io_method:=mmap']
            print('Launching '+repr(command),flush=True)
            camera=subprocess.Popen(command,start_new_session=True)
        rec=Recorder(node,**settings)
        deadline=time.monotonic()+20
        while rec.ready() and time.monotonic()<deadline:
            if camera and camera.poll() is not None:raise RuntimeError('usb_cam exited before RGB became ready')
            rclpy.spin_once(node,timeout_sec=.05)
        if rec.ready():raise RuntimeError(rec.ready())
        print('RGB ready: '+options.topic,flush=True)
        if options.seconds is None:
            rec.shutdown();rec=None
            env=dict(os.environ,C920_RECORDER_OPTIONS=json.dumps(settings))
            gui=subprocess.Popen(['rqt','--force-discover','--standalone','c920_recorder.plugin.RecorderPlugin'],env=env,start_new_session=True)
            if gui.wait()!=0:raise RuntimeError('rqt exited with error')
        else:
            rec.start();print('Recording: '+str(rec.path),flush=True)
            deadline=time.monotonic()+options.seconds
            while rec.writer and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
            rec.stop(f'Scheduled duration {options.seconds:g}s reached')
            if rec.finalizer:rec.finalizer.join()
            print(json.dumps(rec.result,indent=2),flush=True)
            if rec.result['status']!='PASS':raise RuntimeError('Recording verification FAIL')
    except KeyboardInterrupt:pass
    finally:
        if rec:rec.shutdown()
        for process in (gui,camera):
            if process and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        if lock:lock.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
