# C920 RGB Recorder

Independent ROS 2 Humble / rqt package. Existing d435i_recorder is untouched.

```bash
cd /path/to/fma_autonomous_vehicle/ros2_ws
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
colcon build --packages-select c920_recorder --symlink-install
cd ..
./tools/record_c920.sh --device /dev/videoX
./tools/record_c920.sh --existing-camera --topic /your/image_raw
./tools/record_c920.sh --device /dev/videoX --seconds 15
```

Device is discovered from C920 sysfs names and capture formats, not a fixed index. The runner checks device ownership with `fuser`, a per-device launcher lock, and an existing ROS publisher. It refuses duplicate capture; it never kills existing camera processes. Only child processes launched by this invocation are stopped at exit. Requires installed `usb_cam`, `v4l2-ctl`, and `fuser`. `--existing-camera` does not open a video device.

Parameters: `--width 640 --height 480 --fps 30 --format mjpeg2rgb`, `--queue-capacity 128`, `--root ~/c920_recordings_ku`, `--topic /c920/image_raw`. Higher resolutions must be supported by the selected format. YUYV uses `--format yuyv` and may support lower frame rates at high resolutions. All selected parameters must match an existing publisher when reusing one.

Mouse controls: Start/Stop, open folder, play video, autoplay on PASS (enabled by default, can be disabled). Preview is refreshed at 10Hz; image recording is independent. No global keyboard shortcuts or vehicle-control topics. The preview has white-marking candidate bounding boxes only, no distances. RGB image data passed to MP4 is never annotated. Codec `mp4v` is lossy; this is not bit-exact raw RGB archival.

Each session `~/c920_recordings_ku/YYYYMMDD_HHMMSS/` contains `color.mp4`, `frames.jsonl`, `stop_lines.jsonl`, `session.json`, `verification.json`. A same-second directory collision is refused instead of overwriting data. Frames contain `index`, ROS `timestamp_ns`, `frame_id`, original encoding, `receive_monotonic_ns` and `receive_wall_ns`. Each stop-line row is associated with one saved frame, contains `candidates: [[x,y,w,h], ...]`, and `distance_available: false`. An empty candidates list is normal.

The RGB candidate function is copied from the existing Apache-2.0 d435i_recorder baseline: lower 60% ROI, white HSV S<=65/V>=170, 3x9 closing, width>=30% image width, aspect>=4. Yellow does not pass the white mask. These are candidates, not certified semantic stop lines; crosswalks/other white markings may also qualify. No depth, IMU or RealSense dependencies.

A bounded FIFO writer handles MP4, frame metadata and per-frame RGB analysis. Overflow and writer exceptions stop recording and fail verification, with reason preserved. There is no silent queue dropping. Recording starts only with a fresh RGB frame and exactly one publisher. A monotonic 3s RGB watchdog begins at recording start. Finalization drains accepted frames in a separate thread and then performs full MP4 decode, count/schema/timestamp/gap checks. Reversed or duplicate timestamps, >=3s RGB gaps, corrupt/empty videos and queue failures are FAIL. Smaller frame gaps are explicit warnings and missing-period estimates, not device frame-number assertions. No depth/IMU absence warnings. Standalone verification: `ros2 run c920_recorder verify SESSION`.

Installed rqt plugin: `c920_recorder.plugin.RecorderPlugin`. Direct rqt defaults to `/c920/image_raw`, 640x480@30; launch script passes custom settings via `C920_RECORDER_OPTIONS` JSON.
