# ROS2 System Architecture

## 1. Scope

This document fixes the system boundaries, package ownership, command path, and
current high-level mission model. It does not define node implementations,
algorithms, launch files, or firmware protocols. Canonical names and message
types are defined in [topics.md](topics.md).

## 2. System overview

The primary flow is:

```text
SENSOR / DRIVER
      ↓
PERCEPTION
      ↓
LOCALIZATION
      ↓
MISSION
      ↓
CONTROL
      ↓
COMMAND ARBITER
      ↓
VEHICLE CONTROLLER
      ↓
STM32 BRIDGE
      ↓
STM32
      ↓
MOTOR / STEERING
```

This diagram describes responsibility layers, not a strictly serial data
pipeline. Perception and localization consume sensor data independently;
mission nodes may consume perception, localization, and vehicle-state topics;
control nodes may consume the state needed for feedback and safety. Data moves
through declared topics rather than by embedding one layer's logic in another.

## 3. Safety path

Safety is an independent, higher-priority path:

```text
Sensors / Vehicle State
         ↓
   safety_manager
         ↓
  /cmd/emergency
         ↓
  command_arbiter
```

An emergency command always outranks mission and lane commands. Safety input
may include processed obstacle information and, after its interface is defined,
vehicle connection/fault state. The safety path must remain effective regardless
of the active mission.

## 4. Package responsibilities

| Package | Responsibility | Must not own |
|---|---|---|
| `fma_interfaces` | Project-wide ROS2 messages, services, and actions | Perception, mission, or control behavior |
| `fma_vehicle` | ROS2 interface to the STM32 and vehicle hardware | Lane detection or mission decisions |
| `fma_perception` | Convert raw sensor data into perception results | Mission decisions or direct hardware control |
| `fma_localization` | Odometry, GPS, waypoint, and position-related processing | Mission execution or hardware control |
| `fma_control` | Lane control, safety management, command arbitration, and vehicle-control conversion | Sensor-driver or mission-state-machine ownership |
| `fma_mission` | 2026 competition mission state machine and mission-specific behavior | Direct motor/steering hardware access |
| `fma_bringup` | Launch orchestration, YAML configuration, namespaces, and topic remapping | Runtime perception, mission, or control algorithms |

## 5. Node responsibility boundaries

The following rules are fixed:

1. A sensor driver publishes raw sensor data only.
2. A perception node performs recognition only and publishes a perception result.
3. A mission node decides which behavior to perform in the current situation.
4. A mission node never controls motor or steering hardware directly.
5. Only `command_arbiter` selects the final command from multiple
   `fma_interfaces/msg/DriveCommand` candidates.
6. `vehicle_controller` converts `speed_mps` and `steering_angle_rad` targets
   from `/cmd/final` (`DriveCommand`) into `drive_state` and `steering_adc`
   on `/vehicle/command` (`VehicleCommand`).
7. `stm32_bridge_node` translates `VehicleCommand` into the STM32 serial
   protocol `W`/`S`/`X`/`Tdddd`; it does not own physical-unit conversion.
8. Lane detection and mission decisions must not be placed inside
   `stm32_bridge`.
9. The `safety_manager` emergency command always has the highest priority.

These boundaries preserve testability: perception outputs, mission decisions,
arbitration, vehicle conversion, and serial transport can each be tested without
requiring another layer's implementation details.

## 6. Command and vehicle-control path

```text
lane_controller -- /cmd/lane ---------+
                                       |
mission nodes ---- /cmd/mission -------+--> command_arbiter
                                       |            |
safety_manager --- /cmd/emergency -----+            v
                                                 /cmd/final
                                                DriveCommand
                                                    |
                                                    v
                                           vehicle_controller
                                                    |
                                                    v
                                             /vehicle/command
                                              VehicleCommand
                                                    |
                                                    v
                                            stm32_bridge_node
                                                    |
                                                    v
                                                  STM32
```

The fixed command priority is:

1. Emergency: `/cmd/emergency`
2. Mission: `/cmd/mission`
3. Lane following: `/cmd/lane`

`command_arbiter` is the only publisher of `/cmd/final` and selects exactly one
effective command. `vehicle_controller` consumes `/cmd/final`; it does not
arbitrate competing behaviors. The future implementation must define freshness,
timeout, inactive-publisher, and emergency-release behavior before vehicle use.

`vehicle_controller` publishes the low-level `VehicleCommand` on
`/vehicle/command`; `stm32_bridge_node` is its subscriber. Drive state is an enum
(`0 = STOP`, `1 = FORWARD`, `2 = REVERSE`), and steering is a raw ADC target in
the current firmware range `50..4040`. If `emergency_stop` is true, the bridge
must prioritize `X` and suppress forward, reverse, and steering commands.
This does not imply a latched emergency feature in firmware.

Current firmware has no numeric speed command. Physical steering-angle
calibration is also incomplete. `VehicleCommand` therefore contains neither
physical speed/angle targets nor PWM/motor-percentage fields; conversion from
the high-level `DriveCommand` remains `vehicle_controller`'s responsibility.

The implemented bridge consumes `/vehicle/command` and publishes
`/vehicle/feedback`. It validates drive state and steering ADC range, and sends
only `X` for STOP, emergency, invalid input, or stale input. Its independent
`vehicle_command_timeout_sec` defaults to 0.5 seconds since the last valid
non-emergency command; startup also remains STOP. Drive and steering refreshes
share one serial TX path, with one command per write and no steering during STOP.

## 7. Perception, localization, and mission interaction

The important relationships are:

```text
raw camera/depth/LiDAR ──> perception ──> mission and safety
encoder/vehicle feedback ─> odometry ───> mission and control
GPS fix ──────────────────> waypoint manager ─> mission-zone indication
mission manager ──────────> active mission selection
active mission ───────────> /cmd/mission
```

GPS is supporting information for determining that the vehicle is approaching
a mission zone. GPS does not execute a mission and must not directly produce a
vehicle command. Actual mission execution combines the appropriate camera,
depth, LiDAR, odometry, and other perception/state results.

## 8. 2026 mission architecture

The current high-level states match the constants in
`fma_interfaces/msg/MissionState`:

| Value | Mission state | Intent |
|---:|---|---|
| 0 | `START` | Initial state |
| 1 | `NORMAL_DRIVE` | Normal lane-following operation |
| 2 | `STOP_LINE` | Stop-line mission |
| 3 | `S_CURVE` | S-curve obstacle mission |
| 4 | `TRAFFIC_LIGHT` | Traffic-light intersection mission |
| 5 | `PERPENDICULAR_PARKING` | Perpendicular parking mission |
| 6 | `EMERGENCY_ZONE` | Emergency-stop zone behavior |
| 7 | `PARALLEL_PARKING` | Parallel parking mission |
| 8 | `LANE_CONTROL` | Lane-control-signal mission |
| 9 | `FINISH` | End-of-run state |

This is a state vocabulary, not a state-machine implementation. Transition
conditions, completion signaling, retry/failure behavior, and timeouts remain
to be designed before code is written.

## 9. QoS architecture principles

QoS values are not implemented or finalized in this phase. The baseline is:

- Camera, depth, and LiDAR: evaluate ROS2 `SensorDataQoS`/best effort with a
  small queue to favor fresh samples and low latency.
- GPS: evaluate sensor-data-style QoS against the selected driver.
- Perception results: use reliable delivery as the initial candidate with a
  small queue so stale results do not accumulate.
- Control commands: use reliable delivery and a small keep-last queue. Only the
  newest valid command should affect the vehicle.
- Mission state: use reliable delivery so transitions are not silently missed.

Publisher/subscriber compatibility, queue depth, deadline, liveliness, and
timeout behavior require final tuning with the actual sensors and computer load.

## 10. Fixed architecture decisions

- All application nodes use canonical topics; driver-specific names are handled
  by `fma_bringup` remapping.
- The project follows REP-103 and uses `base_link` with `+X` forward, `+Y` left,
  and `+Z` upward.
- Distance, speed, and angle outputs use meters, meters per second, and radians
  wherever their standard ROS interface permits.
- Perception, mission, control, vehicle conversion, and STM32 transport remain
  separate responsibilities.
- All motion commands pass through `command_arbiter`.
- Emergency commands have priority over mission commands, which have priority
  over lane-following commands.
- `command_arbiter` alone publishes `/cmd/final`.
- GPS supplies mission-zone approach information rather than direct control.

## 11. Existing interface gaps and contract risks

No existing interface is changed in this documentation phase. The review found
the following gaps or ambiguities:

1. There are no fixed interfaces for encoder, speed, steering, or STM32 status
   feedback, so odometry and closed-loop vehicle control cannot yet have a final
   input contract.
2. `/mission/zone` has no message type. Waypoint-manager output cannot be
   implemented without choosing one.
3. `/mission/current` and `/mission/status` can represent overall state with
   `MissionState`, but the direction and semantics of individual mission
   completion reporting are not defined. Multiple mission publishers on one
   status topic would be ambiguous without ownership rules.
4. `sensor_msgs/msg/Image` does not by itself fix depth scale; RealSense depth
   encoding and conversion to meters must be specified in configuration or the
   perception contract.
5. Confidence fields do not encode a valid range or invalid-value convention.
6. `DriveCommand.header.stamp` can support freshness checks, but timeout,
   validity, and emergency-release semantics are not fixed.
7. Header `frame_id` semantics for most custom messages and all final sensor TF
   names are not yet fixed.
8. Recommended QoS classes exist, but exact compatible profiles and queue sizes
   are not fixed.

## 12. Open design decisions

The following items remain TBD and must not be inferred by node implementations:

- STM32 serial packet format
- `/vehicle/encoder` message type
- `/vehicle/speed` message type
- `/vehicle/steering_angle` message type
- `/vehicle/status` message type
- Encoder configuration and raw-count representation
- Steering feedback format and raw-ADC/angle conversion location
- Speed-calculation location
- `/mission/zone` message type
- Individual mission completion signaling
- Mission transition, retry, failure, and timeout behavior
- Final TF frame names and TF publisher ownership
- Actual sensor-driver topic remapping
- Depth-image encoding and scale
- Confidence-field semantics
- Command freshness, timeout, and emergency-release behavior
- Final QoS tuning
