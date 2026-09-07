# Keyboard teleop

`keyboard_teleop`은 `/cmd/lane`에 `DriveCommand`를 발행하고
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

**manual teleop 사용 중에는 다른 /cmd/lane publisher를 실행하지 마세요.**
기존 emergency > mission > lane 우선순위를 유지한다. 따라서 mission 입력이나
emergency latch가 활성화되어 있으면 teleop 명령(정지 포함)보다 우선한다.
수동 조작 중에는 mission publisher도 중지해야 하며 emergency latch는
기존 safety 절차로 관리한다. Teleop은 emergency latch를 설정·해제하지 않는다.

| 키 (Enter 불필요, 대소문자 허용) | 동작 |
|---|---|
| W / S | 전진 / 후진 상태 유지 |
| A / D | 조향 target +0.05 / -0.05 rad |
| C | 조향 target 0 rad |
| Space / X | STOP (조향 target 보존) |
| Q / Ctrl+C | STOP 반복 발행 후 종료 |

시작은 STOP, 조향 0 rad이다. 조향은 REP-103 양수=LEFT, 음수=RIGHT이며
실측 비대칭 범위 -0.3054..+0.2810 rad로 clamp한다.
**STOP 상태에서는 실제 조향이 움직이지 않을 수 있음**: bridge가 STOP 동안
조향 TX를 억제한다. 조향 target은 다음 구동 명령에도 유지된다.

전진 +0.2, 후진 -0.2, STOP 0.0을 `speed_mps`로 사용하지만 이는 현재
controller의 방향 표시이며 실제 0.2 m/s 속도 목표가 아니다. W/S는 STM32의
고정 15% PWM 구동 (`DRIVE_PWM=120`, 주기 800 count)의 전진/후진 방향 명령이다.
15% 설정은 사용자가 실차 키보드 수동주행에서 적절한 속도로 확인했다.
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
