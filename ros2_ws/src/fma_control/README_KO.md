# Keyboard teleop

`keyboard_teleop`은 `/cmd/manual`에 `DriveCommand`를 발행하고
`/vehicle/feedback`의 `VehicleFeedback`을 표시한다. Serial이나
`/vehicle/command`에 직접 명령을 보내지 않는다.

ROS 환경과 workspace `install/setup.bash`를 source한 후 각 터미널에서 실행한다:

```bash
# 터미널 1
ros2 run fma_control command_arbiter
# 터미널 2
ros2 run fma_vehicle vehicle_controller
# 터미널 3 (기본 port: /dev/fma_stm32)
ros2 run fma_vehicle stm32_bridge_node
# 터미널 4 (인터랙티브 TTY 필요)
ros2 run fma_control keyboard_teleop
```

우선순위는 **MANUAL > EMERGENCY > MISSION > LANE**이다.
Teleop 전용 `/cmd/manual`과 autonomous lane controller의 `/cmd/lane`은 분리된다.
다른 `/cmd/manual` publisher를 동시에 실행하지 않는다.
첫 fresh manual STOP부터 mission/lane보다 우선하므로 시작 시 수동 정지로 인계된다.
Fresh valid manual은 emergency latch보다 우선하지만 latch 상태를 지우지 않는다.
Manual timeout 직후 latch가 true이면 즉시 emergency STOP을 선택한다.
Teleop은 latch를 설정·해제하지 않는다.

Arbiter의 startup-only `manual_topic` 기본값은 `/cmd/manual`,
`manual_timeout_sec` 기본값은 0.5초이다. Sender timestamp 대신 local monotonic
수신 시각으로 freshness를 판단하며 age >= timeout이면 stale이다.
NaN/Inf manual 입력은 해당 source의 이전 명령도 무효화하고 다음 유효 source로 전환한다.
**Teleop 종료 후 마지막 manual 수신으로부터 timeout이 지나면 자율주행이 자동 재개될 수 있다.**
Q/Ctrl+C 종료 시 STOP 반복 발행은 유지한다. 프로세스 종료/crash 등으로 발행이
끊기면 emergency latch → fresh mission → fresh lane → 모두 없으면 fail-safe STOP
순으로 복귀하며,
stale manual 명령을 계속 유지하지 않는다. Emergency latch는 timeout만으로 해제되지 않고
`/cmd/emergency`의 명시적인 `emergency_stop=false` 수신으로만 해제된다.

| 키 (Enter 불필요, 대소문자 허용) | 동작 |
|---|---|
| W | 전진/STOP에서 전진 PWM +5%, 후진 중에는 PWM -5% |
| S | 전진 중에는 PWM -5% 브레이크, 후진/STOP에서 후진 PWM +5% |
| A / D | STOP 상태에서도 조향 target +0.05 / -0.05 rad |
| C | STOP 상태에서도 조향 target 0 rad |
| Space / X | STOP + PWM 0% (조향 target 보존) |
| Q / Ctrl+C | STOP + PWM 0% 반복 발행 후 종료 |

시작은 STOP, PWM 0%, 조향 0 rad이다. 조향은 REP-103 양수=LEFT, 음수=RIGHT이며
실측 비대칭 범위 -0.3054..+0.2810 rad로 clamp한다.
STOP 상태에서도 A/D/C로 실제 조향을 요청할 수 있다. Bridge는 `X`로 traction을
정지시킨 뒤 최소 20 ms 간격으로 `Tdddd`를 보낸다. 이때 W/S는 보내지 않는다.
Emergency, invalid command, timeout은 조향도 정지시키며 X만 전송한다.

전진 +0.2, 후진 -0.2, STOP 0.0을 `speed_mps`로 사용하지만 이는 현재
controller의 방향 표시이며 실제 0.2 m/s 속도 목표가 아니다.
**PWM duty는 실제 속도 km/h와 다르다.** W/S를 같은 방향으로 누를 때마다
0 → 5 → 10 → … → 50%로 증가하고 최대 50%에서 clamp한다.
전진 중 S, 후진 중 W는 현재 방향을 유지하며 PWM을 5%씩 낮춘다.
예: FORWARD 20% → S: 15% → S: 10% → S: 5% → S: STOP 0% → S: REVERSE 5%.
후진 중 W도 대칭적으로 동작한다. PWM이 0%에 도달하면 STOP이 되고,
STOP 상태에서 다음 W/S 입력이 있어야 해당 방향 5%로 출발한다.
Startup-only ROS parameter `manual_pwm_step_percent`는 기본 5 (정수 1..100),
`manual_pwm_max_percent`는 기본 50 (정수 0..50)이며 범위 밖 값은 시작 시 거부한다.
예: `ros2 run fma_control keyboard_teleop --ros-args -p manual_pwm_max_percent:=50`.

`DriveCommand`와 `VehicleCommand`의 `use_pwm_override` 기본값은 false,
`drive_pwm_percent` 기본값은 0이다. Keyboard는 override=true를 발행하며,
arbiter는 manual 선택 시에만 override를 전달한다. Controller는 0..100을 검증한다.
Bridge는 `Vddd`로 duty를 설정한 뒤 최소 20 ms 후 W/S를 보낸다.
Autonomous lane/mission은 override=false로 **기본 15%**를 사용한다
(`DRIVE_PWM=120`, 주기 800 count). Manual timeout 후 autonomous로 복귀할 때도
`V015`로 복원하므로 마지막 manual duty가 남지 않는다.
메시지 정의가 확장되었으므로 관련 publisher/subscriber를 모두 rebuild하고
새 `install/setup.bash` 환경에서 재시작해야 한다.
구동은 키를 놓아도
유지되므로 정지하려면 Space/X를 누른다. UI는 command target과 실제 feedback을
구분하며 속도는 `abs(speed_mps) * 3.6`의 km/h, ADC와 drive 상태를 표시한다.
Feedback 미수신 시 `--`, 1초 이상 갱신이 없으면 STALE로 표시한다.

10 Hz steady timer와 키 입력 시 명령을 발행한다. STOP은 `emergency_stop=false`,
속도 0, 현재 조향 target이며, 종료/exception 때 timer를 취소하고 50 ms 간격으로
3회 발행을 시도한 뒤 터미널 설정을 복원한다. 이미 ROS context가 종료된 경우
발행은 실패할 수 있다. 프로세스 crash/강제 종료 시 반복 발행·터미널 복원을
보장할 수 없으며, 다른 유효 source가 없을 때 기존 arbiter/controller timeout이
STOP으로 전환한다. 이 노드는 별도 dead-man 스위치를 구현하지 않는다.

하드웨어 없는 검증:

```bash
python3 -m pytest ros2_ws/src/fma_control/test ros2_ws/src/fma_vehicle/test
```

## D435i lane controller

lane_controller는 /perception/lane을 구독하고 /cmd/lane에만 DriveCommand를
발행한다. keyboard_teleop의 /cmd/manual과 분리되며 priority는 그대로
**MANUAL > EMERGENCY > MISSION > LANE**이다.

~~~bash
# 기본 비활성화: 항상 lane STOP
ros2 run fma_control lane_controller
# 실차 검증 단계에서만 사용자가 명시적으로 활성화
ros2 run fma_control lane_controller --ros-args -p enabled:=true
~~~

모든 파라미터는 startup-only다.

| 파라미터 | 기본값 |
|---|---:|
| enabled | false |
| k_lateral | 1.0 |
| k_heading | 1.0 |
| confidence_threshold | 0.6 |
| lane_timeout_sec | 0.5 |
| forward_speed_mps | 0.2 |
| output_rate_hz | 20.0 |

P 제어: steering = k_lateral × lateral_error_m + k_heading × heading_error_rad.
LEFT 양수/RIGHT 음수이고 -0.3054..+0.2810 rad로 clamp한다.
Gain은 음수가 아닌 유한값이어야 한다. Derivative는 구현하지 않는다.

Startup, enabled=false, 미검출, confidence 부족/범위 오류, NaN/Inf,
local monotonic 수신 age >= lane_timeout_sec일 때
speed=0, steering=0, emergency_stop=false STOP을 발행한다.
20Hz steady timer에서 재평가하므로 timeout은 다음 tick(기본 최대 약 50ms)에 반영된다.
이전 명령을 유지하지 않으며 입력 header timestamp를 freshness에 사용하지 않는다.
출력 header는 현재 ROS 시간과 빈 frame_id이다.

Forward +0.2는 실제 0.2m/s 속도 목표가 아니라 방향 명령이다.
Autonomous 기본 DRIVE_PWM=120/800=15%이며 실제 속도는 /vehicle/feedback encoder로
측정된다. Detector 물리 보정 미완료 상태에서는 detected=false이므로 STOP이다.
이 노드의 STOP은 다른 우선순위 source를 정지시키는 전역 emergency가 아니다.
