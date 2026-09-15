# C920 lane follower: DRY-RUN 우선

기본 `enable_drive=false`에서는 제어 publisher 자체가 없다. 카메라와 기존 lane
perception만 실행하여 터미널에 계산 결과를 1초마다 출력한다. GPS나 정지선은 제어에
사용하지 않는다. 실제 카메라/차량 실행은 소프트웨어 자동 테스트에 포함하지 않는다.

## 입력 및 제어 계산

카메라 하나를 V4L2/MJPG/640x480/30fps로 열고 기존 replay의 `make_mask()`와
`C920BEV`, `PaperLaneTracker.process()`를 그대로 사용한다. 빌드 시 기존 replay 소스와
`config/c920_bev_calibration.yaml`을 수정 없이 fma_control share에 설치한다.
알고리즘, HSV, calibration, lane tracker parameter는 변경하지 않는다.
기존 standalone left_lane_follow는 자체 검출/serial 경로가 있어 사용하지 않는다.

`PAIR_VALID`와 `pair_quality.valid=true` 및 실제 `center`가 모두 있어야 계산한다.
단일 차선 temporary_center/PAIR_WEAK는 사용하지 않는다. 기본 preview는 차량 전방
desired 2 m이며, 관측 범위와 BEV가 겹치는 거리 구간의 양 끝에서 0.10 m씩
제외한 내부로 clamp한다. 관측 길이 0.30 m 미만 또는 양쪽 margin을 확보할 수
없는 구간은 INVALID다. Fit의 관측 범위 밖으로 extrapolate하지 않는다.

BEV는 x=오른쪽, y=전방, pixel v=아래쪽이다. center가 u(v)일 때:

```text
used_lookahead = clamp(lookahead_m, observed_min + 0.10, observed_max - 0.10)
v_target = ground_to_bev_pixel(0, used_lookahead).v
lateral_error_m = -bev_pixel_to_ground(u(v_target), v_target).x
heading_error_rad = atan(du/dv at v_target)
steering = lateral_gain * lateral_error_m + heading_gain * heading_error_rad
```

오차와 조향은 왼쪽 양수/오른쪽 음수다. 기본 gain은 0.30 rad/m, 0.60 rad/rad.
이는 새 제어기의 초기값이며 실차에서 튜닝/주행 안정성을 검증한 값은 아니다.
기존 ConversionConfig의 -0.3054..+0.2810 rad로 clamp한다.
`steering_angle_to_adc()`로 예상 ADC를 계산한다. ADC 예상은 기본 controller 보정 기준으로,
실제 controller parameter를 따로 바꾼 경우 달라질 수 있다. 조향 보정 로직은 수정하지 않는다.

## 안전과 명령 경로

- DRY-RUN: 제어 topic publisher 없음. 기존에 실행 중인 다른 제어 프로세스를 정지시키는
  기능은 아니므로 현장 preview 전 다른 차량 제어 프로그램은 별도로 종료해야 한다.
- DRIVE: 명시적 `enable_drive:=true`로만 허용. 유효 차선에서 고정 raw CCR=40(5% duty).
  `.2 speed_mps`는 현재 시스템의 전진 방향 표시이며 실제 속도 제한/속도제어가 아니다.
  PWM만으로 실제 속도를 보증할 수 없고 낮은 duty에서 차량이 움직이지 않을 수도 있다.
- `/cmd/lane → command_arbiter → /cmd/final → vehicle_controller → /vehicle/command
  → stm32_bridge → 기존 F0040/Tdddd` 경로. Lane node는 serial에 접근하지 않는다.
- 기본 arbiter는 lane의 raw-PWM 요청을 거부한다. `allow_lane_pwm=true`를 명시해야
  통과한다. 이 opt-in 없이 5% 요청이 legacy W(15%)로 바뀌는 것을 방지한다.
  기존 raw-PWM 없는 lane 명령과 manual/mission 우선순위는 유지한다.
- 신규 launch는 DRIVE일 때만 arbiter/controller/bridge를 시작하며 mission 입력은 분리한다.
  기존 MANUAL > EMERGENCY > MISSION > LANE 우선순위에서 manual은 계속 인계 가능하다.
  다른 arbiter/최종 명령 publisher를 동시에 실행하지 않는다.
- invalid/PAIR_WEAK, NaN, 목표 관측 범위 부족, 카메라 실패/프레임 지연 시 DRIVE는
  PWM=0/emergency_stop=true를 전송한다. DRY-RUN은 no-command다.
- 카메라 작업은 worker thread에서 하고 steady timer는 20 Hz로 계속 최신 결과를 검사한다.
  read 시작 이후 0.25초 이상 경과한 결과는 거부한다. 카메라 read가 막혀도 기존
  결과를 계속 신뢰하지 않는다. 다만 V4L2 내부 버퍼의 실제 exposure timestamp까지
  보증하지는 않는다. 요청한 buffer size=1을 backend가 무시할 수 있다.
- 종료 시 DRIVE만 STOP 3회 best-effort 발행한다. ROS context 종료/통신 단절에는 기존
  arbiter/controller/bridge timeout과 MCU 700 ms timeout/IWDG가 필요하다.
- 정지선 자동정지는 이번 node에 연결하지 않는다. 기존 stop detector/tracker/live 도구는
  그대로 남는다. 같은 C920을 live 도구와 follower가 동시에 열지 않는다.

## 로그와 warnings

`DRY-RUN` 또는 `DRIVE ENABLED`, LANE VALID, reason, lateral_error_m,
heading_error_deg, steering_cmd_rad, expected_adc, FPS, RankWarnings_total과
 desired_lookahead / used_lookahead / observed_min / observed_max를 출력한다.
예: 관측 1.7..3.8 m이면 2.0 m, 관측 2.3..4.0 m이면 2.4 m를 사용한다.
INVALID 또는 오래된 프레임의 used_lookahead는 N/A다.
이번 node는 GUI를 만들지 않는다. RankWarning만 fitting 호출 범위에서 수집/집계하고,
다른 warning은 다시 출력한다. 실제 fitting/배열/결과는 변경하지 않는다.

## 빌드

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fma_control
```

## 현장 첫 실행: DRY-RUN

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fma_control lane_follow.launch.py enable_drive:=false device:=/dev/video0
```

Ctrl+C로 종료한다. Node만 직접 실행해도 기본 DRY-RUN이다:
`ros2 run fma_control lane_follow --ros-args -p enable_drive:=false`.
직접 실행 시 `config`, `lookahead_m`, `lateral_gain`, `heading_gain`,
`max_frame_age_sec` ROS parameter를 startup-only로 지정할 수 있다.

## 명시적 DRIVE (자동 테스트에서는 실행하지 않음)

```bash
ros2 launch fma_control lane_follow.launch.py enable_drive:=true device:=/dev/video0 \
  port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505055877267173020-if02
```

이는 차량 구동을 허용하는 명령이다. 앞서 확인된 장치 고유 by-id를 사용하며,
잘못된 GNSS 장치를 가리켰던 `/dev/fma_stm32`에는 의존하지 않는다.
기존 F/B raw-PWM 지원 firmware를 그대로 사용하며 새 flash/protocol이 필요 없다.

## 하드웨어 없는 테스트

```bash
python3 -m pytest -q ros2_ws/src/fma_control/test ros2_ws/src/fma_vehicle/test
```

ROS와 빌드한 overlay를 source한 후 저장소 루트에서 실행한다. 새 테스트는 부호,
steering saturation, invalid 차단, stale timeout, DRY-RUN publisher 부재, warning 결과
동일성 및 실제 ROS callback 경로의 F0040/STOP을 mock serial로 검증한다.

## 좁은 도로의 DRY-RUN 전용 폭 override

`lane_min_width_m` 기본값은 3.50 m이며 실제 BEV resolution으로 pixel로 변환한다.
production tracker 기본값은 그대로 유지한다. DRY-RUN에서만 다른 값을 지정할 수 있다:

```bash
ros2 launch fma_control lane_follow.launch.py enable_drive:=false device:=/dev/video14 lane_min_width_m:=3.0
```

DRIVE에서 3.50 이외의 값을 지정하면 launch는 어떤 node도 시작하기 전에 오류로 종료한다.
node 직접 실행에도 동일한 검사가 적용된다. 실행 중 parameter 변경은 허용하지 않는다.
이 override는 해당 node의 최소 폭에만 적용되며 ego pair selection, BEV, HSV, stop-line,
조향 gain과 single-line fallback의 기존 확정 폭 조건은 유지한다.
