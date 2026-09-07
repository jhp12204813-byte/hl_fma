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

Safety is an independent path below fresh manual and above autonomous commands:

```text
Sensors / Vehicle State
         ↓
   safety_manager
         ↓
  /cmd/emergency
         ↓
  command_arbiter
```

Fresh valid manual commands outrank emergency; emergency outranks mission and lane. Safety input
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
9. Fresh valid manual commands have highest priority; emergency is next.

These boundaries preserve testability: perception outputs, mission decisions,
arbitration, vehicle conversion, and serial transport can each be tested without
requiring another layer's implementation details.

## 6. Command and vehicle-control path

```text
lane_controller -- /cmd/lane ---------+
                                       |
mission nodes ---- /cmd/mission -------+--> command_arbiter
keyboard_teleop -- /cmd/manual --------+            |
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

1. Manual override: `/cmd/manual`
2. Emergency: `/cmd/emergency`
3. Mission: `/cmd/mission`
4. Lane following: `/cmd/lane`

`command_arbiter` is the only publisher of `/cmd/final` and selects exactly one
effective command. `vehicle_controller` consumes `/cmd/final`; it does not
arbitrate competing behaviors. `command_arbiter` publishes at 20 Hz by default
(`output_rate_hz`) using a steady timer. Lane, mission and manual candidates expire at
local monotonic receive age >= `lane_timeout_sec` / `mission_timeout_sec` /
`manual_timeout_sec` (all default 0.5 s); sender header timestamps are ignored.
Fresh valid manual wins over emergency, then mission, then lane. NaN/Inf speed or
steering invalidates that source's
previous candidate, with throttled warnings; arbitration falls back to another
valid source. No valid manual/mission/lane candidate, including startup, produces a
fail-safe STOP: speed and steering zero, `emergency_stop=true`.

Emergency latch starts false. A newly received `/cmd/emergency` with
`emergency_stop=true` sets the latch regardless of numeric fields. While latched and without fresh valid manual input,
output is the standardized STOP. Fresh valid manual is selected without clearing the latch. Only an explicit newly received
`emergency_stop=false` on `/cmd/emergency` releases the latch, after which
current manual/mission/lane freshness is evaluated again. Stale emergency input alone
never releases it. `emergency_timeout_sec` (default 0.5 s) only controls stale
emergency warning diagnostics. Emergency numeric fields are never selected.
Selected manual/mission/lane commands retain all three command fields, including an
intentional `emergency_stop=true`; that flag does not set the arbiter latch.
Output headers use current ROS publish time and an empty `frame_id`.

Keyboard teleop publishes only `/cmd/manual`; autonomous lane control uses
`/cmd/lane`. The first fresh manual STOP takes over autonomous commands.
On Q/Ctrl+C or handled errors, teleop retains its repeated STOP cleanup.
After publishing ends (including a crash), manual expires at its receive-time
timeout: a preserved emergency latch immediately selects STOP; otherwise fresh
mission resumes, then fresh lane, else STOP. Stale manual is never retained.

`vehicle_controller` publishes the low-level `VehicleCommand` on
`/vehicle/command`; `stm32_bridge_node` is its subscriber. Drive state is an enum
(`0 = STOP`, `1 = FORWARD`, `2 = REVERSE`), and steering is a raw ADC target in
the current firmware range `150..3950`. If `emergency_stop` is true, the bridge
must prioritize `X` and suppress forward, reverse, and steering commands.
This does not imply a latched emergency feature in firmware.

Measured physical steering ADC endpoints are approximately RIGHT=7 and LEFT=4095.
Operational safe targets are RIGHT=150, CENTER=2132, LEFT=3950 (ADC increases left).
Firmware steering PWM is 520/800 = 65%. Measured operational angle calibration
is enabled by default in `vehicle_controller`: RIGHT=-17.5 degrees (-0.3054 rad)
at ADC 150, CENTER=0 rad at ADC 2132, LEFT=+16.1 degrees (+0.2810 rad) at ADC 3950.
REP-103 positive is left, negative is right; the measured asymmetry is preserved.
Each side uses piecewise linear interpolation and integer rounding, retaining
the 0.001 rad center tolerance. Finite angles beyond these endpoints clamp to
150/3950; NaN/Inf, invalid calibration, emergency and timeout retain fail-safe STOP.
Explicitly disabling calibration still rejects nonzero angles outside center tolerance.
At zero speed the controller publishes DRIVE_STOP with the calculated ADC;
the bridge continues to suppress steering TX during STOP.
Topics, message definitions, serial packet shape and raw telemetry are unchanged.

Current firmware has no numeric speed command. `VehicleCommand` contains neither
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
| 0 | `START` | Start |
| 1 | `NORMAL_DRIVE` | Normal Drive |
| 2 | `RAMP` | Ramp |
| 3 | `INTERSECTION_STRAIGHT_1` | Intersection Straight 1 |
| 4 | `S_CURVE` | S Curve |
| 5 | `INTERSECTION_STRAIGHT_2` | Intersection Straight 2 |
| 6 | `PERPENDICULAR_PARKING` | Perpendicular Parking |
| 7 | `INTERSECTION_LEFT` | Intersection Left |
| 8 | `CHILD_DUMMY` | Child Dummy |
| 9 | `PARALLEL_PARKING` | Parallel Parking |
| 10 | `INTERSECTION_RIGHT` | Intersection Right |
| 11 | `SIGNAL_CAR` | Signal Car |
| 12 | `LANE_CHANGE` | Lane Change |
| 13 | `FINISH` | Terminal state |

`mission_manager` loads the 11 ordered GPS targets from
`fma_mission/config/waypoints.yaml` (default activation radius 3.0 m each).
START transitions to NORMAL_DRIVE. Only the current target is checked using
Haversine distance; active missions ignore GPS. `/mission/status` reports from
mission nodes advance the course only when the mission matches and completed=true.
WP10 chains SIGNAL_CAR to LANE_CHANGE without a new GPS check. WP11 enters terminal
FINISH. Stop lines and traffic lights are perception inputs, not standalone missions.
GPS only triggers missions; maneuver nodes and their timeout/retry policies are not implemented.

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
- Command priority is MANUAL > EMERGENCY > MISSION > LANE.
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
3. `/mission/current` is published by mission_manager; mission nodes report
   matching completed missions on `/mission/status`. Status has no execution ID;
   authentication and restart/replay handling remain future work.
4. `sensor_msgs/msg/Image` does not by itself fix depth scale; RealSense depth
   encoding and conversion to meters must be specified in configuration or the
   perception contract.
5. Confidence fields do not encode a valid range or invalid-value convention.
6. Command arbitration freshness, validity, and emergency-release semantics
   are defined in section 6; sender stamps are not used for freshness.
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
- Mission maneuver retry, failure, and timeout behavior
- Final TF frame names and TF publisher ownership
- Actual sensor-driver topic remapping
- Depth-image encoding and scale
- Confidence-field semantics
- Final QoS tuning
