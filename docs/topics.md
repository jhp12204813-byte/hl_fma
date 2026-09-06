# ROS2 Topic Contract

## 1. Purpose and status

This document defines the canonical ROS2 topic contract for the FMA autonomous
vehicle. Nodes must use these names and interfaces so that implementations can
be developed independently.

The `Status` column applies to the topic name and message type contract:

- `FIXED`: the canonical topic name and message type are fixed.
- `TBD`: the topic contract still requires a design decision.

A `FIXED` row may still contain a `TBD` unit or `frame_id` when only that part of
the contract is unresolved. Those unresolved details are collected in
[Open design decisions](#10-open-design-decisions).

## 2. Canonical naming and driver remapping

Application nodes consume only the canonical topics below. If a USB camera,
RealSense, RPLIDAR, or GPS driver publishes another name, `fma_bringup` must map
the driver topic to the canonical name in a launch file. Driver-specific names
must not leak into perception, localization, mission, or control nodes.

## 3. Sensor and driver topics

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/camera/lane/image_raw` | `sensor_msgs/msg/Image` | USB lane camera driver (remapped) | `lane_detection_node`, `stopline_detection_node` | Pixels; encoding-defined | TBD sensor optical frame | Lane and stop-line source image | FIXED |
| `/camera/signal/image_raw` | `sensor_msgs/msg/Image` | USB signal camera driver (remapped) | `traffic_light_detection_node`, `lane_signal_detection_node` | Pixels; encoding-defined | TBD sensor optical frame | Traffic-light and lane-control-signal source image | FIXED |
| `/front/color/image_raw` | `sensor_msgs/msg/Image` | Front RealSense driver (remapped) | `obstacle_detection_node`, `parking_perception_node` | Pixels; encoding-defined | TBD front color optical frame | Front RGB data | FIXED |
| `/front/depth/image_raw` | `sensor_msgs/msg/Image` | Front RealSense driver (remapped) | `obstacle_detection_node`, `parking_perception_node` | TBD from image encoding; perception outputs use m | TBD front depth optical frame | Front depth data | FIXED |
| `/rear/color/image_raw` | `sensor_msgs/msg/Image` | Rear RealSense driver (remapped) | `parking_perception_node` | Pixels; encoding-defined | TBD rear color optical frame | Rear RGB data | FIXED |
| `/rear/depth/image_raw` | `sensor_msgs/msg/Image` | Rear RealSense driver (remapped) | `parking_perception_node` | TBD from image encoding; perception outputs use m | TBD rear depth optical frame | Rear depth data | FIXED |
| `/scan` | `sensor_msgs/msg/LaserScan` | RPLIDAR driver (remapped) | `obstacle_detection_node`, `parking_perception_node` | Range: m; angles: rad | TBD lidar frame | Planar LiDAR scan | FIXED |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS driver (remapped) | `waypoint_manager_node` | Latitude/longitude: degree; altitude: m | TBD GPS frame | Global position fix used for mission-zone approach detection | FIXED |

`NavSatFix` uses geographic latitude and longitude in degrees by definition;
this is the intentional exception to the project's SI preference.

## 4. Perception topics

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/perception/lane` | `fma_interfaces/msg/Lane` | `lane_detection_node` | `lane_controller_node`, mission nodes that require lane context | Lateral error: m; heading: rad; curvature: 1/m; confidence: dimensionless | TBD | Detected lane geometry and confidence | FIXED |
| `/perception/stopline` | `fma_interfaces/msg/StopLine` | `stopline_detection_node` | `mission_stopline_node`, `mission_traffic_light_node` | Distance: m; confidence: dimensionless | TBD | Stop-line detection and distance | FIXED |
| `/perception/traffic_light` | `fma_interfaces/msg/TrafficLight` | `traffic_light_detection_node` | `mission_traffic_light_node` | State: enum; confidence: dimensionless | TBD | Classified traffic-light state | FIXED |
| `/perception/obstacles` | `fma_interfaces/msg/ObstacleArray` | `obstacle_detection_node` | `safety_manager`, `mission_s_curve_node` | Position/distance: m; bearing: rad; confidence: dimensionless | `base_link` | Obstacles expressed in the vehicle body frame | FIXED |
| `/perception/parking` | `fma_interfaces/msg/ParkingInfo` | `parking_perception_node` | `mission_perpendicular_parking_node`, `mission_parallel_parking_node` | Clearances: m | TBD | Parking validity and directional clearances | FIXED |
| `/perception/lane_signal` | `fma_interfaces/msg/LaneSignal` | `lane_signal_detection_node` | `mission_lane_control_node` | State: enum; confidence: dimensionless | TBD | Left/right lane availability | FIXED |

Confidence fields are dimensionless. Their exact valid range and invalid-value
handling are not encoded in the current messages and remain TBD.

## 5. Localization topics

`/gps/fix` is the fixed localization input defined in the sensor table. GPS is
used to determine that the vehicle is approaching a mission zone; it does not
directly execute a mission.

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `odometry_node` | `mission_manager_node`, parking and motion-related mission nodes, control nodes as needed | Position: m; linear velocity: m/s; angular velocity: rad/s | TBD; conceptually `odom` to `base_link` | Local vehicle pose and motion estimate | FIXED |
| `/mission/zone` | TBD | `waypoint_manager_node` | `mission_manager_node` | TBD | TBD | Indicates the mission zone being approached | TBD |

The `/mission/zone` type will be decided before implementing the waypoint
manager. No new custom message is introduced by this document.

## 6. Mission topics

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/mission/current` | `fma_interfaces/msg/MissionState` | `mission_manager_node` | Individual mission nodes and nodes that gate behavior by mission | Mission enum and booleans | TBD; non-spatial message | Publishes the currently selected overall mission | FIXED |
| `/mission/status` | `fma_interfaces/msg/MissionState` | `mission_manager_node` | Monitoring, recording, and system-level consumers | Mission enum and booleans | TBD; non-spatial message | Publishes aggregate mission activity/completion status | FIXED |

Individual mission nodes perform mission-specific behavior. The exact topic and
interface by which each mission node reports completion to
`mission_manager_node` are still TBD. `/mission/status` does not implicitly
settle that input-side contract.

## 7. Control topics

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/cmd/lane` | `fma_interfaces/msg/DriveCommand` | `lane_controller_node` | `command_arbiter` | Speed: m/s; steering: rad | TBD | Normal lane-following command candidate | FIXED |
| `/cmd/mission` | `fma_interfaces/msg/DriveCommand` | Active mission node | `command_arbiter` | Speed: m/s; steering: rad | TBD | Mission-specific command candidate | FIXED |
| `/cmd/emergency` | `fma_interfaces/msg/DriveCommand` | `safety_manager` | `command_arbiter` | Speed: m/s; steering: rad | TBD | Highest-priority safety command candidate | FIXED |
| `/cmd/final` | `fma_interfaces/msg/DriveCommand` | `command_arbiter` | `vehicle_controller` | Speed: m/s; steering: rad | TBD | The only selected command sent into vehicle control | FIXED |

```text
lane_controller -- /cmd/lane ---------+
                                       |
mission nodes ---- /cmd/mission -------+--> command_arbiter -- /cmd/final
                                       |                             |
safety_manager --- /cmd/emergency -----+                             v
                                                         vehicle_controller
                                                                   |
                                                                   v
                                                              stm32_bridge
```

The following boundaries are mandatory:

- Perception nodes must not control the STM32 directly.
- Mission nodes must not control the STM32 directly.
- `lane_controller_node` must not send commands directly to the STM32.
- Every final drive command must pass through `command_arbiter`.

The fixed arbitration priority is:

1. `/cmd/emergency`
2. `/cmd/mission`
3. `/cmd/lane`

`command_arbiter` publishes only `/cmd/final`.

## 8. Vehicle feedback topics

`/vehicle/feedback` is the canonical integrated feedback topic for information
actually available in the current STM32 firmware telemetry. Its publisher is
the future `stm32_bridge_node`; localization, control, and diagnostics are
candidate subscribers.

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/vehicle/feedback` | `fma_interfaces/msg/VehicleFeedback` | `stm32_bridge_node` | localization / control / diagnostics (candidates) | Encoder: count; measured speed: m/s; steering: raw ADC; drive state: enum | TBD | Integrated STM32 encoder, measured speed, steering raw ADC, and drive state feedback | FIXED |

The fields map to current firmware telemetry as follows:

- `encoder_count`: STM32 telemetry `ENC` value.
- `speed_mps`: STM32 telemetry `SPEED`, converted from mm/s to m/s by the
  ROS2 bridge.
- `steering_adc`: steering potentiometer raw ADC value, not a steering angle.
- `drive_state`: firmware state `0 = STOP`, `1 = FORWARD`, `2 = REVERSE`,
  represented by `DRIVE_STOP`, `DRIVE_FORWARD`, and `DRIVE_REVERSE`.

The feedback direction is:

```text
STM32
  ↓
stm32_bridge_node
  ↓
/vehicle/feedback
  ↓
localization / control / diagnostics
```

This contract adds only the message definition; `stm32_bridge_node` is not
implemented in this phase. Steering angle, emergency state, communication
status, fault bitfields, motor PWM, encoder/ADC faults, and heartbeat are not
available in current firmware telemetry and are not fields of this message.
Final steering-angle, status, and fault interfaces remain TBD.

The previously identified separate topics below remain TBD; the integrated
feedback contract does not finalize or require these separate interfaces.

| Topic | Message Type | Publisher | Subscriber | Unit | frame_id | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `/vehicle/encoder` | TBD | `stm32_bridge` | `odometry_node` | TBD: raw count or derived quantity | TBD | Wheel encoder feedback | TBD |
| `/vehicle/speed` | TBD | TBD: `stm32_bridge` or ROS-side estimator | `odometry_node`, `vehicle_controller`, `safety_manager` | m/s after conversion; wire representation TBD | TBD | Current vehicle speed | TBD |
| `/vehicle/steering_angle` | TBD | TBD: `stm32_bridge` or ROS-side converter | `odometry_node`, `vehicle_controller`, `safety_manager` | rad after conversion; raw ADC representation TBD | TBD | Current steering feedback | TBD |
| `/vehicle/status` | TBD | `stm32_bridge` | `safety_manager`, monitoring nodes | TBD | TBD | STM32 connection, fault, and vehicle state | TBD |

Further design must determine whether separate topics are needed, their final
message types, encoder configuration, raw-ADC-to-angle conversion, any future
status/fault fields, and the final serial packet contract. The current integrated
feedback uses firmware-provided encoder count and speed plus raw steering ADC
and drive state as defined above.

## 9. Coordinate, unit, and TF rules

The project follows ROS REP-103:

- Distance: meter (`m`)
- Speed: meters per second (`m/s`)
- Angle: radian (`rad`)
- Vehicle body frame: `base_link`
- `+X`: vehicle forward
- `+Y`: vehicle left
- `+Z`: vehicle upward

`Obstacle.x_m`, `Obstacle.y_m`, and `Obstacle.bearing_rad` use `base_link`.
Bearing is zero toward `+X` and positive counterclockwise toward `+Y`, following
the REP-103 right-handed convention.

The conceptual TF tree is:

```text
map
└── odom
    └── base_link
        ├── lane_camera_frame
        ├── signal_camera_frame
        ├── front_depth_frame
        ├── rear_depth_frame
        ├── lidar_frame
        └── gps_frame
```

Only `base_link` and the REP-103 axis convention are fixed here. The displayed
sensor-frame names and the final `map`/`odom` TF design are examples and remain
TBD until sensor mounting positions and TF publishers are designed.

## 10. QoS baseline recommendations

These are ROS2 Humble design candidates, not code settings. Publisher and
subscriber QoS must be compatible, and final values require measurement on the
vehicle.

| Topic group | Candidate policy | Reason | Status |
|---|---|---|---|
| Camera, depth, LiDAR | `SensorDataQoS` or best effort with a small depth | Low latency is more useful than retransmitting stale high-rate samples | TBD |
| GPS | Sensor-data-oriented QoS; best effort/reliability to be tested with the actual driver | The driver rate is low, but compatibility and reconnection behavior must be measured | TBD |
| Perception results | Reliable as the default candidate, with a small queue | Consumers need coherent results while stale perception should not accumulate | TBD |
| Control commands | Reliable with a small keep-last queue | Delivery matters, but only the newest command should affect the vehicle | TBD |
| Mission state | Reliable | State transitions should not be silently lost | TBD |

Control topics are freshness-sensitive. The design must avoid processing a
backlog of old commands; queue depth, deadline, liveliness, and command timeout
values remain to be tuned.

## 11. Open design decisions

The following items are explicitly TBD:

- STM32 serial packet format
- `/vehicle/encoder` message type and encoder configuration
- `/vehicle/speed` separate message type and any ROS-side speed-estimation role
- `/vehicle/steering_angle` message type and steering feedback format
- `/vehicle/status` message type and required status/fault fields
- `/mission/zone` message type
- Individual mission completion signaling topic/interface
- Final TF frame names and TF publisher ownership
- Actual sensor-driver topic names and launch remapping
- Depth-image encoding/unit handling
- Confidence-field range and invalid-value semantics
- Control command timeout/freshness rules
- Final QoS tuning
