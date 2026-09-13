# Keyboard PWM teleop

`keyboard_teleop → /cmd/manual → command_arbiter → /cmd/final →
vehicle_controller → /vehicle/command → stm32_bridge_node → STM32`를 사용한다.
Teleop은 serial에 직접 접근하지 않는다. 기존 조향 calibration, encoder telemetry,
700 ms firmware timeout, UART 오류 STOP, IWDG를 유지한다.

## PWM 및 키 동작

초기 throttle=0%, 조향=0 rad. 키보드 throttle은 **0..100%**, 한 번 입력에
**5 percentage point**씩 증감한다. 50% 제한은 없고 95 → 100 → 100으로 clamp한다.
상한에서 반대 키를 누르면 95, 90, …, 5, 0%가 된다.

현재 구동 timer는 PSC=0, ARR(`PWM_PERIOD`)=799, 주기 800 count/20 kHz이다.
키보드 메시지 발행 직전 `throttle_to_ccr()`에서만 아래 변환을 수행한다:

```text
CCR = min(799, round(clamp(throttle_percent, 0, 100) × 800 / 100))
```

| Throttle | Raw CCR |
|---|---:|
| 0% | 0 |
| 5% | 40 |
| 10% | 80 |
| 15% | 120 |
| 20% | 160 |
| 50% | 400 |
| 100% | 799 |

100% 입력은 기존 hardware-safe CCR 상한 799에 대응한다(실제 duty 99.875%).
CCR=800인 연속 HIGH까지 확장하지 않으며 timer와 펌웨어 상한을 유지한다.

| 키 | 동작 |
|---|---|
| W | STOP에서 forward 5%. Forward면 +5%p, reverse면 -5%p |
| S | STOP에서 reverse 5%. Reverse면 +5%p, forward면 -5%p |
| A / D | 조향 target +0.05 / -0.05 rad |
| C | 조향 target 0 rad |
| Space / X | 즉시 PWM=0, STOP 발행; 조향 target 보존 |
| Q / Ctrl+C | PWM=0, STOP 발행 및 3회 반복 후 종료 |

반대 키는 PWM을 0까지 낮추기만 한다. 0에서 다음 반대 키를 눌러야 역방향 5%가 된다.
키를 놓아도 설정은 유지되며 키 자동 반복은 반복 입력으로 처리한다.
조향 범위는 기존 -0.3054..+0.2810 rad (오른쪽/왼쪽)로 clamp하고,
controller가 기존 비대칭 보정으로 ADC 150/2132/3950에 대응시킨다.
STOP에서는 bridge가 조향 TX를 억제하므로 A/D/C는 target만 바꾼다.
UI의 THROTTLE TARGET은 요청 duty %이며 실제 속도나 CCR feedback이 아니다.

저장소에 명시된 MDD20A의 PWM 입력은 20 kHz까지 지원한다
([제조사 사양](https://my.cytron.io/p-20amp-6v-30v-dc-motor-driver-2-channels)).
0..799는 전기적 PWM 명령 범위이며 차량의 모든 부하/전류 조건에서 검증한 운용
속도 범위라는 뜻은 아니다. 기존 W/S 고정 120 count는 그대로 유지한다.

## 메시지와 wire protocol

`DriveCommand`와 `VehicleCommand`에 `pwm_control`, `drive_pwm`,
`DRIVE_PWM_MAX=799`를 추가했다. Teleop은 pwm_control=true로 발행한다.
`speed_mps`의 +0.2/-0.2/0은 기존 방향 표시로만 쓰고 PWM magnitude와 섞지 않는다.
ROS 메시지의 drive_pwm은 계속 raw CCR count다. Percentage 변환은 teleop의
발행 경계에서만 수행하며, arbiter/controller/bridge/STM32에서 다시 scaling하지 않는다.
Arbiter는 manual의 PWM 필드를 전달하고 lane/mission은 기존 방향 모드를 유지한다.
Controller/bridge는 799 초과를 STOP 처리하고 0 PWM도 STOP 처리한다.

- Forward 5% → `F0040`; reverse 15% → `B0120`; 최대 → `F0799`/`B0799`.
- 정확히 4자리 decimal의 패킷이 완성돼야 적용한다. CCR 값은 1:1 대응한다.
- 0/STOP/emergency → `X`. Steering은 기존 `Tdddd`.
- `pwm_control=false`이면 기존 `W`/`S`/`X`, 고정 120 count 동작.
- Firmware는 새 패킷에도 50 ms 부분 패킷 timeout, X 선점, 범위/문법 오류 STOP,
  arrival timestamp 및 queue overflow/USART 오류 보호를 적용한다.
- 같은 방향의 PWM 변경은 CCR만 갱신한다. 방향 변경은 기존대로 양쪽 CCR=0을
  update event로 적용하고 기존 delay 후 DIR를 바꾼다. Timer 설정은 바꾸지 않는다.
- Bridge는 TX 간격(20 ms) 중 받은 STOP을 보관하여 후속 입력이 덮어쓰지 못하게 한다.
  STOP 발행은 키 callback에서 즉시 하지만 DDS/arbiter 및 serial 전송 지연은 존재한다.

이번 percentage 변경은 메시지 타입과 firmware를 바꾸지 않는다. **앞서 F/B raw PWM
firmware를 flash한 보드는 재flash가 필요 없다.** fma_control을 재빌드하고 teleop을
재시작한다. F/B 지원 이전의 구 firmware는
F/B 숫자를 무시하며, 이 프로토콜에는 firmware 버전 협상/ACK가 없다.

## Build / run

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fma_control
```

터미널 1:

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fma_control manual_drive.launch.py port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0671FF505055877267173020-if02
```

현재 `/dev/fma_stm32`가 GNSS를 가리키는 상태를 확인했으므로 위 명령은 보드 고유
by-id 경로를 명시한다.

터미널 2 (인터랙티브 TTY):

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run fma_control keyboard_teleop
```

위 launch는 controller/bridge/arbiter만 시작하고 lane/mission 입력을 사용하지 않는
별도 topic으로 설정한다. 동일 graph에 다른 arbiter/vehicle command publisher를
함께 실행하지 않는다. Manual timeout(0.5초) 뒤 STOP을 선택하며 자율주행으로
넘기지 않는다. `receive_only:=true`는 serial을 열어 수신하되 모든 쓰기를 억제한다.
하드웨어 없는 자동 테스트에는 이 launch를 실행하지 않고 mocked serial을 사용한다.

기본 arbiter를 별도로 실행하면 기존 MANUAL > EMERGENCY > MISSION > LANE 정책을
유지한다. 이 경우 manual 종료 후 fresh mission/lane이 있으면 자동 복귀할 수 있다.
Steady/monotonic timer, controller/bridge 0.5초 timeout, MCU 700 ms timeout과
약 1초 IWDG는 유지한다. 종료 STOP은 best effort이며 프로세스 강제 종료나 통신
단절 시 delivery ACK를 보장하지 않는다.

## Software validation

```bash
# ROS 및 방금 빌드한 install/setup.bash source 후, 저장소 루트에서:
python3 -m pytest -q ros2_ws/src/fma_control/test ros2_ws/src/fma_vehicle/test
python3 firmware/stm32/tools/test_command_parser.py
```

펌웨어 빌드에는 CMSIS Core/STM32F4 Device/HAL 헤더가 필요하다. 현재 저장소에는
Makefile 기본 경로의 Drivers가 없으므로 외부 헤더 경로를 지정한다. 이번 검증은
ARM CMSIS_5 5.9.0, ST cmsis-device-f4 v2.6.10, stm32f4xx-hal-driver v1.8.3을
`/tmp/fma-pwm-deps`에 받아 다음 명령으로 ELF/HEX/BIN을 생성했다:

```bash
make -C firmware/stm32 all BUILD=/tmp/fma-pwm-firmware \
  'INCLUDES=-IInc -I/tmp/fma-pwm-deps/cmsis_core/CMSIS/Core/Include -I/tmp/fma-pwm-deps/cmsis_device/Include -I/tmp/fma-pwm-deps/hal/Inc'
```

실제 MCU flash, serial 접속, 차량 구동은 수행하지 않았다. Host C harness는 실제
firmware 소스를 fake register로 컴파일하고 parser/IRQ/timeout/방향 변경 전 PWM=0을
검증한다. 실제 IWDG 경과 시간과 전기적 PWM 파형은 이 테스트로 측정하지 않는다.
