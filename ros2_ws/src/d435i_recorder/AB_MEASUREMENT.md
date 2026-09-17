# D435i A/B measurement

## Protocol

Diagnostic harness: `test/ab_device.py`. This file is not collected by pytest.
All ablations are confined to a fresh diagnostic process using temporary method
wrappers; production feature defaults are unchanged for the initial A/B run.
Each condition measures 25 seconds after a 5-second subscriber warmup, using one
continuously running driver with the required serial/profile settings. Cases run
sequentially, so time/order and external host load remain possible confounders.

1. camera_only: subscribers only, no Recorder, media writes or preview.
2. preview_off: full recording, Qt refresh timer disabled.
3. preview_on: full recording, actual Qt preview timer (100ms), independent ROS spinner.
4. png_off: same as 3, only depth PNG encode/file write bypassed; depth reception,
   copying, queue, timestamp matching and JSONL remain enabled.
5. analysis_off: same as 3, candidate extraction bypassed in writer AND preview.
6. imu_off: same as 3, IMU writer submission bypassed; subscription, serialization
   and observation remain enabled. This isolates storage from receiving IMU.

Disabled-output trials are explicitly diagnostic, deliberately incomplete sessions.
Their automatic integrity checker may FAIL for missing PNG/IMU; that is not evidence
of a new runtime failure. Original recordings are never modified.

Capture receive count, ROS timestamp Hz, monotonic receive Hz, max timestamp gap,
timestamp-inferred missing frame periods, metadata frame-number gaps, RMW message_lost
events, process CPU percent, camera CPU percent and whole-host CPU busy percent.
Process 100% means one logical CPU. Saved frame rates/counts, queue peak and MP4/PNG
latencies come from the normal verification.json and writer_metrics.

PNG wrapper measures thread CPU time separately from wall time. A 5ms heartbeat
thread samples scheduling delay. Heartbeat latency includes both OS scheduling and
GIL contention and cannot by itself prove the GIL is the cause. Current PNG encoder
is `depth_png.py` (NumPy + zlib + file writes), not OpenCV imwrite; OpenCV is used
for MP4 and candidate/preview processing. Absence of an RMW loss event does NOT
prove absence of DDS/SDK drops. Metadata frame-number gaps are measured separately.

The ROS spinner is separate from Qt event pumping, avoiding the artificial callback
backlog introduced by earlier single-loop UI smoke tests.

## Executor control

The installed `/opt/ros/humble/lib/python3.10/site-packages/rqt_gui_py/rclpy_spinner.py`
creates `MultiThreadedExecutor()` with its default thread count. The primary six-case
run therefore uses the same executor. `--executor single` supports a separate control
measurement with everything else held fixed. Earlier interrupted diagnostic directories
`ab_20260909` and `ab_20260909_isolated` are excluded from the six-case comparison.
An existing idle rqt window was discovered and its recorder-owned launcher/camera were
shut down before the controlled run; it must not contribute background preview load.

## Initial six conditions (25s each, multi executor)

| Condition | RGB rx Hz | Depth rx Hz | IMU rx Hz | process CPU % | camera CPU % | queue peak | MP4 mean ms | PNG mean ms | metadata missing | Early stop |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| camera_only | 28.89 | 28.41 | 192.38 | 108.9 | 88.0 | — | 0 | 0 | 0 | False |
| preview_off | 27.20 | 28.18 | 76.29 | 139.5 | 90.9 | 512 | 15.09 | 46.12 | 0 | True |
| preview_on | 27.31 | 27.99 | 74.42 | 142.7 | 96.1 | 512 | 16.52 | 39.61 | 1 | True |
| png_off | 27.29 | 27.46 | 76.38 | 153.2 | 94.9 | 264 | 14.26 | 0.11 | 0 | False |
| analysis_off | 26.79 | 27.16 | 75.69 | 163.0 | 100.5 | 9 | 16.29 | 33.02 | 2 | False |
| imu_off | 26.32 | 26.78 | 60.99 | 163.4 | 93.5 | 244 | 15.74 | 38.33 | 1 | False |

Raw JSON: `/home/idp2/d435i_recordings/ab_20260909_rqt/`. Receive Hz uses
monotonic clock; source timestamp Hz and stored frame counts/rates are in each JSON.
Early-stop cases still observe reception for 25s, but storage covers only time until
automatic error stop; their write rates must not be divided by the full 25s.
RMW counters in these diagnostic files cover warmup through finalization; timestamp
and metadata-gap statistics cover the active 25s receive window.

## Additional controls and evidence-based change

Single executor, all features ON, 25s: source RGB/depth 25.97/25.62Hz,
IMU 199.66Hz (multi executor ~75Hz); queue peak 195, no auto stop. This supports
executor scheduling contention for IMU, but does not explain RGB loss alone.

Single executor + reliable images, all features ON, 25s observation: RGB source
29.71Hz, depth 27.62Hz. Both publisher endpoints actually offered RELIABLE.
Higher accepted frame volume overloaded the writer (automatic stop at 20.804s,
queue peak 512). RMW reported RGB loss 3, depth loss 16 over full trial lifecycle.
Reliable QoS substantially improved RGB delivery but did not remove all loss.

PNG OFF did not restore receive rates, and measured PNG thread CPU time (~9ms)
was much shorter than wall time (~39ms). Thus PNG-only GIL saturation is NOT proved.
System load/order remain confounders; this is not a dedicated GIL profiler trace.
Analysis OFF lowered queue peak to 9 without fixing IMU. This implicates analysis
in storage pipeline blocking, separate from delivery and executor issues.

Minimal combined change after controls:
- Image subscriptions request RELIABLE, matching tested camera endpoints; IMU stays
  sensor-data best effort. No sensor data resampling or timestamp changes.
- rqt plugin owns a separate ROS node/SingleThreadedExecutor instead of adding
  recorder callbacks to the framework's MultiThreadedExecutor.
- Candidate extraction runs in a one-thread worker, bounded to eight
  outstanding analyzed frames, with one OpenCV internal thread per process. RGB/depth writer
  no longer performs the candidate extraction synchronously. FIFO analysis results
  are retained, matched by timestamps, and drained on stop.
- Recording preview reuses candidate boxes instead of recomputing detection on the
  same live image. Preview remains 10Hz; MP4 remains configured 30FPS.

Initial A/B files record old behavior. Running the same diagnostic harness after
this patch uses the plugin's dedicated executor, regardless of its unused context
node executor. Those results are explicitly post-fix, not a repeat of old controls.


## Additional per-case storage and loss results

Stored Hz below is ROS timestamp-based over the stored portion. Early-stop trials
are shorter than 25s and cannot be treated as 25s successful recordings.
RMW loss counters cover the trial lifecycle including warmup/finalization; these
are not to be added to timestamp-inferred frame gaps (they may overlap).

| Condition | Stored RGB count / Hz | Stored depth count / Hz | RMW RGB / depth / IMU lost | Whole-host busy % | PNG thread CPU / wall ms |
|---|---:|---:|---:|---:|---:|
| camera_only | n/a | n/a | 36 / 27 / 13 | 55.4 | n/a |
| preview_off | 230 / 26.49 | 237 / 27.30 | 88 / 61 / 22 | 62.1 | 13.64 / 46.00 |
| preview_on | 193 / 27.14 | 197 / 27.84 | 102 / 69 / 19 | 66.0 | 8.97 / 39.48 |
| png_off | 687 / 26.85 | disabled | 125 / 82 / 23 | 67.0 | 0.01 / 0.01 |
| analysis_off | 671 / 26.39 | 677 / 26.56 | 171 / 123 / 48 | 69.6 | 8.57 / 32.91 |
| imu_off | 660 / 25.88 | 667 / 26.16 | 193 / 113 / 35 | 69.6 | 8.89 / 38.22 |


The first process-based analysis attempt (ab_20260909_final60) failed after 2.70s:
analysis pending reached 8 during spawn/import startup and the main queue filled.
That implementation was replaced with a one-thread analysis worker, preserving the
bounded queue/drain semantics without process startup and image IPC cost. This
change does not claim to remove all Python GIL contention.

## Post-fix 60-second test 1

Raw result: `/home/idp2/d435i_recordings/ab_20260909_thread60/preview_on.json`.
All data streams, PNG, IMU, analysis and preview enabled. Real camera; Qt offscreen
plugin owns the same single executor as production rqt. Context uses rqt-style multi
executor but recorder subscriptions are on the dedicated node. No user vehicle process
was started/stopped and no vehicle commands were issued.

Session: `/home/idp2/d435i_recordings/ab_20260909_thread60/20260909_182323_979219`. Early stop: False. Result: WARN, failures [].

| Stream | Receive Hz | Stored count | Stored ROS timestamp Hz | Max ROS gap ms |
|---|---:|---:|---:|---:|
| rgb | 30.090 | 1808 | 29.906 | 66.694 |
| depth | 28.974 | 1740 | 28.781 | 200.079 |
| imu | 200.193 | 12018 | 199.073 | 15.023 |

Writer metrics:
```json
{
  "queue_capacity": 512,
  "queue_peak": 19,
  "overflow_events": [],
  "analysis_pending_capacity": 8,
  "analysis_pending_peak": 4,
  "depth_workers": 2,
  "depth_pending_capacity": 32,
  "depth_pending_peak": 7,
  "png_encoding": "16-bit grayscale, filter None, zlib level 0",
  "latency_ms": {
    "queue_wait": {
      "count": 17378,
      "mean": 9.677123509840019,
      "max": 94.584422,
      "p95_recent": 22.74508705
    },
    "mp4_write": {
      "count": 1808,
      "mean": 13.953648210730094,
      "max": 65.239082,
      "p95_recent": 19.524598949999998
    },
    "depth_png_write": {
      "count": 1740,
      "mean": 39.15337516091954,
      "max": 89.99774,
      "p95_recent": 56.03604785
    }
  },
  "latency_note": "wall duration; p95 uses last <=10000 samples; writes include OS cache, not fsync"
}
```

RMW lifecycle lost counters:
```json
{
  "/camera/camera/imu": 78,
  "/camera/camera/aligned_depth_to_color/image_raw": 38,
  "/camera/camera/aligned_depth_to_color/camera_info": 86,
  "/camera/camera/color/image_raw": 16,
  "/camera/camera/color/metadata": 1,
  "/camera/camera/color/camera_info": 1
}
```

Metadata frame-number gaps in active window: 1. Process CPU 167.9%, camera CPU 96.5%, whole-host busy 65.3%.

Receive observer window starts just after Recorder.start and ends just before stop;
therefore observer counts can differ slightly from stored counts at boundaries.
`session.received_streams` counts, unlike external observer counts, cover the exact
recording lifecycle and should equal the saved counts in the absence of writer loss.

Depth max gap 200.079ms exceeds the unchanged 200ms long-gap threshold. The result
remains WARN; it is not rounded down or hidden. Timestamp-inferred missing periods
RGB=5, depth=73 remain visible despite average rates meeting 28.5–30Hz. IMU calibration
is a separate warning. A second 60s run checks repeatability without changing settings.

## Post-fix 60-second repeat

No further tuning between successful 60s tests. Same all-features-on/preview condition.

Session: `/home/idp2/d435i_recordings/ab_20260909_repeat60/20260909_182600_081273`

| Stream | Exact received / saved count | Stored timestamp Hz | Max gap ms |
|---|---:|---:|---:|
| rgb | 1806 / 1806 | 29.8786 | 100.0686 |
| depth | 1770 / 1770 | 29.2827 | 200.1370 |
| imu | 11855 / 11855 | 199.4226 | 10.0301 |

Writer metrics:
```json
{
  "queue_capacity": 512,
  "queue_peak": 34,
  "overflow_events": [],
  "analysis_pending_capacity": 8,
  "analysis_pending_peak": 4,
  "depth_workers": 2,
  "depth_pending_capacity": 32,
  "depth_pending_peak": 10,
  "png_encoding": "16-bit grayscale, filter None, zlib level 0",
  "latency_ms": {
    "queue_wait": {
      "count": 17242,
      "mean": 11.701298212562355,
      "max": 91.90565,
      "p95_recent": 41.98451139999998
    },
    "mp4_write": {
      "count": 1806,
      "mean": 12.95712768936878,
      "max": 68.727163,
      "p95_recent": 18.28336125
    },
    "depth_png_write": {
      "count": 1770,
      "mean": 29.844690025423702,
      "max": 88.620186,
      "p95_recent": 50.63920365
    }
  },
  "latency_note": "wall duration; p95 uses last <=10000 samples; writes include OS cache, not fsync"
}
```

RMW lifecycle loss counters:
```json
{
  "/camera/camera/imu": 49,
  "/camera/camera/aligned_depth_to_color/image_raw": 23,
  "/camera/camera/aligned_depth_to_color/camera_info": 49,
  "/camera/camera/color/image_raw": 12,
  "/camera/camera/color/metadata": 1,
  "/camera/camera/color/camera_info": 1
}
```

Metadata frame-number missing count=1 in active 60s observation. Estimated missing
RGB/depth periods=6/42. These overlap with RMW events and must not be summed as
independent losses. Observer windows differ slightly from exact session counts.

Result: WARN, failures=[], automatic=false. Full MP4 decoded frame count=1806,
all 1770 PNGs uint16 640x480, receive/save counts identical. Queue overflow=0,
input peak=34/512, analysis pending peak=4/8, PNG pending peak=10/32.
Mean MP4/PNG latency=12.957/29.845ms; p95=18.283/50.639ms.
Process CPU=166.18% (one core=100%). No frame resampling or timestamp modification.

**Acceptance not fully met:** source rates meet 28.5–30Hz in both successful 60s
trials, but long-gap-free acquisition does NOT: depth 200.137ms is still above
unchanged 200ms threshold. RMW losses persist despite no writer loss/backlog.
No claim of lossless sensor delivery or perfect synchronization is made.
IMU calibration warning remains separate from data-delivery warnings.

## Final checks

34 tests passed, including camera-only connection boundaries, reliable image QoS,
recorder-owned executor lifecycle, keyboard key independence, asynchronous analysis
not blocking IMU, full uint16 lossless PNG, writer overflow/errors, timestamp matching
and verification. Build and git diff --check passed; new untracked files checked
individually for whitespace. No vehicle/keyboard/STM32 code modifications and no
git add/commit/push. Test-owned cameras were shut down; existing failed recordings
and all diagnostic results retained.
