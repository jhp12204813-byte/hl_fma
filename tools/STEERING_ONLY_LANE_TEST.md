# Steering-only lane field diagnostic

This tool bypasses the production ROS command path for field diagnostics only.
It reuses C920BEV, replay make_mask, PaperLaneTracker, calculate_command (including
adaptive lookahead), and vehicle_controller steering calibration unchanged.
Default dry-run opens the camera but never opens serial. No ROS publisher exists.
The production 3.50 m minimum remains unchanged; --lane-min-width-m applies only
to this diagnostic tracker. Stop-line and GPS control are not connected.

Close all other camera users and all vehicle command writers before running.
Exclusive serial access helps prevent another cooperating serial opener; this
program cannot block commands from an already-running bridge or another source.
The enabled mode physically moves the steering wheels. Propulsion commands are
not supported: the serial write allowlist accepts only X and Tdddd. The existing
ADC range is 150..3950. X is sent at serial startup, on invalid samples, periodically
while acquiring/stale, and before serial close at exit. A disconnected/broken
serial link can prevent delivery; cleanup still attempts X and closes the port.

Five consecutive fresh valid frames are required for Tdddd. PAIR_WEAK, missing
center, or an invalid calculate_command result cannot steer. Read-start timestamps
expire after 0.25 s; sequence gaps also reset acquisition. Steering is capped at
10 Hz. A separate camera thread lets the main loop send X during blocked capture.
Driver-internal exposure age is not known; buffer size 1 is requested.

From the repository root, source the existing built ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 tools/steering_only_lane_test.py --device /dev/video14 --lane-min-width-m 3.0
```

Explicit physical steering (no propulsion commands):

```bash
python3 tools/steering_only_lane_test.py \
  --device /dev/video14 \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505055877267173020-if02 \
  --lane-min-width-m 3.0 --enable-steering
```

Ctrl+C exits. Logs report STATE=ACQUIRING/STEERING/INVALID, lane_valid,
lateral_error_m, heading_error_deg, steering_cmd_rad, steering_adc and FPS once
per second. DRY-RUN STEERING means the gate would permit steering, not serial I/O.

Software-only tests:

```bash
python3 -m pytest -q tools/tests/test_steering_only_lane.py
```
