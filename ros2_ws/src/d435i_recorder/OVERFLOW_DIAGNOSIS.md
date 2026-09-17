# Writer overflow diagnosis — 2026-09-09

## Evidence before modification

Analyzed existing session `/home/idp2/d435i_recordings/20260909_173250_404081`
without changing its files. All six recent sessions had `writer_errors: ["queue overflow"]`
and empty `recording_errors`. Latest dropped counts: `depth: 1`.

The queue is ONE shared FIFO, capacity 512 messages, for RGB, depth, IMU and metadata.
It is not a dedicated depth queue. A rejected depth enqueue proves occupancy reached
512 at that instant; no historical peak trace or per-write latency was recorded.

`~/.ros/log/python3_12562_1788942763140.log` records:
`[1788942776.786029278] Writer rejected depth; recording will FAIL`.
This is 2026-09-09 17:32:56.786029 KST, 6.381 seconds after session creation.
The precise failed put time is slightly earlier than the log call and was not stored.
The associated camera log has no disconnect/exception at this time.

Auto stop path: `Writer.submit -> queue.Full -> writer.errors += queue overflow ->
Recorder.poll -> if self.errors or self.writer.errors -> stop -> drain queue -> verify FAIL`.
There was no recorded watchdog timeout. The session end time includes queue drain:
17:33:01.441757, approximately 4.59 seconds after the last received RGB.
This drain time must not be mistaken for a 3-second stream timeout.

| Stream | Count | ROS timestamp Hz | Receive monotonic Hz | Last receive wall ns |
|---|---:|---:|---:|---:|
| RGB | 185 | 26.7745 | 26.7482 | 1788942776853297267 |
| Depth | 170 | 26.5246 | 26.5838 | 1788942776841113180 |
| IMU | 607 | 95.5435 | 95.5473 | 1788942776842955732 |

These are saved/accepted messages, not a complete historical DDS receive counter.
Last messages were received 55–67 ms AFTER the first overflow log, consistent with
poll-based error stop and inconsistent with a 3-second lack of all streams.
RGB largest ROS gap 300.279 ms ended 6.372 seconds BEFORE overflow; its receive
interval was 440.192 ms. Depth max gap 100.098 ms occurred before overflow.
IMU largest ROS gap 175.793 ms ended 56.9 ms AFTER overflow; its receive interval
was 173.584 ms, so it started before overflow. Gaps therefore are not all caused by
the final rejected message; callback/DDS scheduling and writer backlog coexist.

No historical CPU or I/O counters exist. A later host snapshot found rqt 124% CPU,
camera 90.4%, and other applications/vehicle processes active. These are process
lifetime averages, not CPU samples from the failed recording.

## Same-host sample benchmarks (not historical write latency)

Using original session depth/000100.png and decoded RGB, temporary output only:

| Operation | mean wall ms | p95 wall ms | mean CPU ms |
|---|---:|---:|---:|
| Original OpenCV PNG | 32.84 | 35.48 | 32.59 |
| OpenCV PNG compression 0 | 36.02 | 42.63 | 35.64 |
| OpenCV PNG compression 1 | 54.48 | 64.87 | 54.30 |
| MP4 write | 6.98 | 11.87 | not measured |
| New PNG None filter + zlib 0 | 10.65 | 11.59 | not measured |

Original PNG + MP4 already costs ~39.8 ms per pair, exceeding the 33.3 ms
30 FPS budget before analysis/JSONL. PNG wall ~= CPU suggests encoding CPU is the
major measured bottleneck, not disk waiting. Benchmarks use OS page cache and do
not prove sustained physical-disk bandwidth. USB disconnect and IMU calibration
are not evidenced causes of this overflow. IMU calibration remains a separate WARN.

## Minimal fix

Keep the bounded 512-entry queue and 3-second monotonic watchdog; do not hide overload
by enlarging the queue or relaxing timeout. Keep all media file I/O on writer thread.
Replace depth encoder with standards-compatible grayscale uint16 PNG using filter
None and zlib compression 0. All 65,536 uint16 values round-trip exactly through
OpenCV decode. No quantization. File size ~615 KB means ~18.5 MB/s depth at 30 FPS
and ~1.1 GB/minute; this is an explicit storage/CPU tradeoff.

Record queue capacity/peak, bounded overflow event history with monotonic/wall times,
queue wait and MP4/PNG mean/max/recent p95 latency, and per-frame write latency.
Latency covers encode/write into OS cache, not fsync. Preserve overflow as FAIL.
Auto stop reason and stop-time last-receive ages/ROS timestamps are now in UI,
session.json, verification.json and ROS log. Manual/scheduled/shutdown stops are distinct.
Watchdog age uses callback-entry monotonic receive time matching JSONL metadata.

Existing configured-rate tolerance was already +/-3 Hz (27–33), producing WARN,
not FAIL. Keep it: normal 29–30 Hz is accepted. Also report estimated missing frame
periods and intervals over 1.5 nominal periods, even when average rate is within
range. These are timestamp inferences, not confirmed driver frame counters.


## Iterative hardware evidence

1. PNG-only optimization: session 20260909_175030_165837 still overflowed at 5.503s.
   PNG mean 25.60ms + MP4 mean 11.21ms under live load exceeded serial throughput.
   Stop-time RGB/depth/IMU ages were 10.3/6.6/5.1ms: direct proof of writer-error
   auto stop while required streams were fresh. Overflow was IMU at capacity 512.
2. Separate one depth worker: 20260909_175218_907582 completed 32.002s, overflow 0,
   input queue peak 117/512, depth pending peak 28/32. All files decoded, no FAIL.
   RGB 25.27 Hz, depth 24.96 Hz, IMU 186.69 Hz; estimated missing RGB/depth periods
   151/161 were reported, not hidden. PNG mean 39.64ms was still too slow for 30Hz.
3. Final design uses TWO depth workers, at most 32 outstanding PNGs, preserving
   JSONL order and draining every job before verification. OpenCV internal threads
   are limited to one in the recorder process to avoid nested thread-pool contention.
   This does not change vehicle processes, camera settings or watchdog timeout.

4. Two depth workers with one OpenCV internal thread:
   20260909_175400_053016 completed 32.002s, overflow 0, queue peak 94,
   depth pending peak 4. RGB/depth 27.00/27.12Hz, max gaps 200.5/133.9ms.
   Writer no longer had a large PNG backlog, but receive loss remained.
   Reduced IMU callback reflection/recursive conversion to fixed-field serialization
   preserving every original field, and enlarged bounded sensor-data DDS history
   from 5 to 30 image/metadata messages and 400 IMU messages. Best-effort QoS remains;
   this absorbs scheduling bursts rather than concealing lost samples. No timing
   warning thresholds or writer queue capacities were relaxed.


## Final recorder and Qt 32-second test

`20260909_175813_982169`: Qt offscreen preview, start button, recording held for
32.003 seconds, then test clicked Stop. stop_details.automatic=false, Manual stop.
No vehicle command was issued and existing keyboard/vehicle processes were not stopped.

- RGB 880, depth 868, IMU 5599, device metadata 967; every received message in
  received_streams was written (counts agree), overflow 0.
- RGB ROS timestamp 27.309 Hz / receive 27.505 Hz; depth 26.964 / 27.141 Hz.
- IMU ROS timestamp 185.348 Hz / receive 175.087 Hz. The Qt test pumps ROS and Qt
  sequentially and accumulated IMU delay (~2 seconds by last ROS timestamp);
  this does not certify the real rqt spinner's scheduling. Separate headless test
  20260909_175655_952356 measured IMU 198.869 Hz over 32 seconds.
- Queue peak 23/512, depth pending peak 3/32, overflow_events empty.
- MP4 mean/p95/max 11.323/15.706/27.965 ms.
- PNG mean/p95/max 25.656/36.075/59.446 ms, two concurrent workers.
- Queue wait mean/p95/max 8.563/28.886/64.778 ms.
- Full MP4 decode 880 frames; every depth PNG uint16 640x480; no file errors.
- Max ROS gap RGB/depth/IMU 166.819/166.817/35.133 ms, below unchanged long-gap
  thresholds 200/200/100 ms. This is NOT a claim of no missing frame periods:
  88 RGB and 99 depth missing periods are estimated and WARN is retained.
- Device startup additionally logged time_diff_keeper Device or resource busy and
  Depth stream start failure; despite these notifications streams continued. These
  are separate from the original overflow and are not silently treated as calibrated
  device operation. Original failed session had no such notification at overflow.

The original 5–6-second automatic-stop issue is resolved in repeated 32-second
recordings. The approximately 29–30 Hz acceptance target is NOT yet demonstrated
under the current host load; measured RGB/depth remains around 27 Hz.


## Receive-only control measurement

A 12-second measurement after 6-second camera warmup, using only timestamp callbacks
(no Recorder, no writer, no OpenCV/preview), produced RGB 28.888 Hz, depth 29.055 Hz,
IMU 199.436 Hz. Max ROS gaps 100.132/66.768/10.024 ms respectively.
Log: `/tmp/d435i_receive_baseline.log` (temporary; numerical evidence retained here).
This indicates additional recorder load contributes to the residual ~27 Hz;
not all gaps can be assigned solely to upstream camera failure. Current fixes
remove writer overflow but do not fully satisfy 29–30 Hz recording at this host load.
A read-only CPU frequency observation was ~800 MHz; it was not changed, and a single
sample does not establish thermal throttling or a power-management root cause.

## Final checks and scope

33 tests passed, including uint16 full-range PNG round trip, queue peak/overflow,
async depth worker failure finalization, IMU progress during blocked PNG writes,
fast IMU serialization equivalence, fresh-stream overflow auto stop and stop reason
persistence. `colcon build --packages-select d435i_recorder --symlink-install` passed.
`git diff --check` plus untracked package whitespace checks passed.

Only d435i_recorder source/docs/tests changed. No vehicle/keyboard/STM32/control code,
no motor messages, no git add/commit/push. Existing recorder/camera process tree was
restarted to load changes after network-interface change prevented discovery;
all test-owned camera processes were subsequently shut down. User vehicle/teleop
processes were not stopped. Existing failed recordings were preserved.

Modified: writer.py, recording.py, plugin.py, runner.py, verification.py, setup.py,
README.md, test/test_recorder.py, test/hardware_ui_smoke.py, VALIDATION.md.
Added: depth_png.py, test/test_writer_diagnostics.py, OVERFLOW_DIAGNOSIS.md.
