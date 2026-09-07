# NUCLEO-F401RE Encoder Test

## 연결

| 엔코더 | NUCLEO-F401RE |
|---|---|
| A상 | PB6 (TIM4_CH1) |
| B상 | PB7 (TIM4_CH2) |
| GND | GND |
| 전원 | 센서 정격에 맞게 연결 |

PB6/PB7 입력은 내부 풀업이 활성화되어 있습니다. MCU 핀에 5 V 신호를 직접
입력하지 마십시오. 오픈 컬렉터 출력이라면 3.3 V 기준으로 사용하십시오.

## STM32CubeIDE

1. `File > Import > Existing Projects into Workspace`를 선택합니다.
2. `EncoderTest/STM32CubeIDE` 폴더를 지정합니다.
3. 프로젝트를 빌드한 뒤 ST-LINK로 실행합니다.
4. ST-LINK Virtual COM 포트를 `115200, 8-N-1`로 엽니다.

출력 예시는 `ENC=123 SPEED=0mm/s STEER=2132 DRIVE=0\r\n` 형식입니다. 회전 방향이 반대라면 PB6과 PB7을
서로 바꾸거나 소프트웨어에서 카운트 부호를 반대로 처리하십시오.

## 고정 serial 경로 설정 (Ubuntu / Jetson)

ROS bridge의 기본 serial device는 `/dev/fma_stm32`이다. USB 연결 순서에 따라
달라지는 `/dev/ttyACM0`, `/dev/ttyACM1` 대신 동일 보드를 식별한다.
`udev/99-fma-stm32.rules`는 실기에서 검증된 rule이며,
serial `0671FF505055877267173020`은 현재 프로젝트 NUCLEO/ST-LINK 보드 고유값이다.
다른 보드로 교체하면 해당 보드의 vendor/product/serial에 맞게 rule을 수정해야 한다.

저장소 root에서 설치한다:

```bash
sudo cp firmware/stm32/udev/99-fma-stm32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

NUCLEO USB를 재연결한 뒤 확인한다:

```bash
ls -l /dev/fma_stm32
```

rule은 장치 권한을 `dialout` 그룹, `0660`으로 설정한다. 실행 사용자가
`dialout`에 속하지 않았다면 `sudo usermod -aG dialout "$USER"` 후 다시 로그인한다.
기존 `tools/setup_stlink_permissions.sh`는 ST-LINK 권한 설정용이며 이 고정
symlink rule 설치를 대신하지 않는다.

ROS 환경을 source한 뒤 다음 명령은 기본적으로 `/dev/fma_stm32`를 사용한다:

```bash
ros2 run fma_vehicle stm32_bridge_node
```

다른 장치를 명시하는 startup parameter도 유지한다:

```bash
ros2 run fma_vehicle stm32_bridge_node --ros-args -p port:=/some/device
```

기존 standalone firmware 도구의 기본 경로는 변경하지 않았다. 아래 예시는
고정 경로를 명시적으로 전달한다.

## 키보드 명령

현재 구동 설정은 `DRIVE_PWM=120`, 주기 800 count (timer ARR `PWM_PERIOD=799`),
즉 고정 duty 15%이다. 사용자가 실차 키보드 수동주행에서 속도가 적절함을
확인한 설정이다. W/S는 이 고정 PWM의 방향 명령이며 실제 속도 목표가 아니다.
조향은 기존 `STEERING_PWM=520`, duty 65%를 유지한다.

| 키 | 동작 |
|---|---|
| W | 앞/뒤 구동모터 전진(15%) |
| S | 앞/뒤 구동모터 후진(15%) |
| A | 왼쪽 조향 |
| D | 오른쪽 조향 |
| C | 조향 중앙 복귀 |
| X 또는 Space | 구동 및 조향 즉시 정지 |

키 입력은 아래 udev 설정 후 `/dev/fma_stm32`, `115200 baud`, `8-N-1` 조건으로 전송합니다.
부팅 시 구동 PWM과 조향 PWM은 모두 0%이며, 조향 목표는 현재 센서값으로
설정되므로 명령 전에는 조향모터가 움직이지 않습니다.

```bash
python3 -m pip install pyserial
./tools/keyboard_control.py /dev/fma_stm32
```

펌웨어는 마지막 유효 명령 후 0.7초 동안 새 키 메시지가 없으면 구동과
조향을 자동으로 정지합니다. 터미널 키 반복을 이용하므로 움직이려는 키를
누르고 있어야 합니다.

구동과 조향 타임아웃은 서로 독립적입니다. 약 1초의 MCU 독립 워치독도
활성화되어 메인 루프가 멈추면 MCU가 재부팅되고 PWM 타이머가 정지합니다.
리셋 중 출력이 뜨지 않도록 MDD20A PWM/DIR 및 L298N ENA/IN 입력에는
외부 풀다운 저항을 추가하는 것이 안전합니다.

상태 출력의 `SPEED`는 앞 구동모터 엔코더를 200ms 간격으로 측정한 현재
속도이며 단위는 `mm/s`입니다. 23.2:1 감속비, 바퀴 둘레 880mm,
바퀴 1회전당 371.2 count를 기준으로 계산합니다. 음수는 엔코더의 반대
회전 방향을 뜻합니다.

## USB 카메라 차선 인식

USB 카메라는 NUCLEO가 아니라 노트북 또는 Jetson에 연결합니다. 호환되는
OpenCV 환경은 프로젝트의 `.venv-camera`에 설치되어 있습니다.

```bash
cd /home/idp2/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Projects/STM32F401RE-Nucleo/EncoderTest
.venv-camera/bin/python tools/camera_lane_control.py --port /dev/fma_stm32 --camera 2
```

카메라 창을 선택한 상태에서 WASD를 사용합니다. 초록색 선은 검출 차선,
빨간 점은 추정 차선 중앙, `offset`은 카메라 중심에 대한 차선 중심 오차입니다.
`L`을 누르면 차선 보조를 켜거나 끕니다. 차선 보조는 조향만 담당하며 구동은
계속 W/S 키 메시지가 들어올 때만 유지됩니다. 차선을 잃으면 조향 출력을
즉시 정지하고, 구동 명령도 0.7초 이상 끊기면 펌웨어가 정지합니다.

카메라는 정면 중앙에 단단히 고정하고 화면 아래쪽에 차량 자체가 너무 많이
보이지 않도록 각도를 맞춥니다. 실제 주행 전에는 구동바퀴를 띄운 상태에서
수동 모드와 차선 검출 화면을 먼저 확인하십시오.

## 왼쪽 차선 80cm 추종 및 정지선 정지

단안 카메라 영상만으로 절대 거리 80cm를 바로 계산할 수 없으므로 현장에서
기준점을 한 번 저장해야 합니다.

1. 차량을 왼쪽 노란 차선에서 실제 80cm 떨어진 위치에 평행하게 놓습니다.
2. 아래 명령으로 프로그램을 실행하고 초록색 검출선이 노란 차선과 일치하는지 확인합니다.
3. `K`를 눌러 현재 위치를 80cm 기준으로 저장합니다.
4. 바퀴를 띄운 시험 후 지면에서 `G`를 눌러 15% PWM 추종을 시작합니다.
5. `X` 또는 Space를 누르면 즉시 정지합니다.

```bash
cd /home/idp2/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Projects/STM32F401RE-Nucleo/EncoderTest
.venv-camera/bin/python tools/left_lane_follow.py --camera 2 --port /dev/fma_stm32
```

정지선은 화면 높이 60%보다 가까운 위치에서 가로로 넓은 흰색 선이 3프레임
연속 검출되면 확정하며, 확정 즉시 구동과 조향을 정지하고 재출발을 잠급니다.
주변을 확인한 뒤 `R`로 잠금을 해제할 수 있지만 차량은 정지 상태를 유지하며,
다시 출발하려면 별도로 `G`를 눌러야 합니다. 왼쪽 차선이 0.25초 이상
사라져도 즉시 정지합니다.

NUCLEO 펌웨어는 `Txxxx` 형식의 조향 목표값 명령을 처리하므로 수정된
펌웨어를 다시 빌드하고 업로드해야 합니다.


## Serial parser 안전 규칙

- `W`/`S`는 구동, `X`는 구동·조향 정지, `Tdddd`는 조향 ADC target이다.
  `P` telemetry 요청과 `A/D/C/H/Q`, Space 명령도 유지한다.
- 숫자 shortcut `1/3/7/9` 및 `0` STOP alias는 제거했다. IDLE에서 모든
  `0..9`를 무시하므로 `2182`, `2300`만 도착해도 동작을 시작하지 않는다.
  키보드/카메라 도구의 숫자 구동 shortcut도 제거했다.
- `T` 수신 시 기존 조향 출력을 끄고 payload 수집 상태로 전환한다.
  decimal 4자리 완성 후 `150..3950` 범위일 때만 target을 적용한다.
  범위 오류/non-digit는 구동·조향 STOP 및 IDLE reset으로 처리하며,
  오류를 일으킨 문자를 새 구동 명령으로 재해석하지 않는다.
- `STEERING_PACKET_TIMEOUT_MS=50` ms: 수신 바이트 사이 간격이 이 값 이상이면
  불완전 payload를 폐기하고 구동·조향을 정지한다. 후속 바이트가 없어도
  main loop가 timeout을 검사한다. `X/x`는 수집 중에도 즉시 STOP/abort한다.
- 현재 실행 코드는 HAL UART가 아닌 CMSIS 레지스터 방식이다. 따라서
  HAL UART/clock 초기화를 새로 도입하지 않고 `USART2_IRQHandler`에서
  single-byte RX와 수신 시각을 64칸 ring buffer(사용 가능 63칸)에 저장한다.
  main loop가 이를 파싱하며, UART TX 도중에도 RX 인터럽트는 활성 상태다.
- ring overflow 또는 UART ORE/FE/NE/PE 발생 시 이후 수신을 폐기하고,
  main loop가 fault를 감지하면 backlog 폐기, parser reset, 구동·조향 STOP을
  수행한다. queue 조작만 짧게 interrupt mask하며 TX/명령 실행은 mask하지 않는다.
  50 ms 이상 오래된 queue 항목도 실행하지 않고 STOP 처리한다.
- 명령별 FORWARD/REVERSE/STOP 등 verbose ACK는 제거했다. 기존 도구와 ROS
  bridge는 ACK를 기다리지 않는다. 5 Hz telemetry와 `P` 응답 형식은 유지한다:
  `ENC=<signed_decimal> SPEED=<signed_decimal>mm/s STEER=<unsigned_decimal> DRIVE=<0|1|2>\r\n`.
  부팅/700 ms timeout 안내는 유지한다. IWDG 및 독립 구동·조향 700 ms timeout도 유지한다.
- 이 변경은 CRC/프레이밍을 추가하지 않는다. 모든 종류의 바이트 손상 검출이나
  실제 하드웨어 동작을 보증하지 않으며 ROS bridge의 기존 분리 TX를 유지한다.

### 하드웨어 없는 검증

`python3 tools/test_command_parser.py`는 실제 `Src/main.c`를 가짜 레지스터와
함께 호스트 C compiler로 컴파일하여 parser/ring-buffer 회귀 검증을 수행한다.
serial 포트나 MCU에 접근하지 않는다. IRQ 타이밍과 물리 PWM 검증은 포함하지 않는다.

Makefile의 기본 `FW_ROOT=../../..`는 원래 Cube 디렉터리 배치를 가정한다.
현재 저장소에서 CMSIS/HAL headers를 찾지 못하면 설치된 Cube F4 경로를
`make all FW_ROOT=<STM32Cube_FW_F4 경로> BUILD=/tmp/fma-stm32-build`로 지정할 수 있다.
`all`은 빌드만 수행하며 `flash`는 별도 명령이다.


## 실차 조향 ADC calibration

사용자가 실차 측정·확인한 값이며, physical endpoint와 운용 target을 구분한다.

| 구분 | RIGHT | CENTER | LEFT |
|---|---:|---:|---:|
| 물리 ADC 끝점 (근사) | 7 | — | 4095 |
| 안전 운용 target | 150 | 2132 | 3950 |

ADC 증가가 LEFT, 감소가 RIGHT이다. 중앙 2132는 직진으로 확인되었다.
`Tdddd` 명령 허용 범위는 `150..3950`이며 물리 끝점 7/4095는 거부한다.
`STEERING_PWM=520`, timer ARR(`PWM_PERIOD`)=799로 주기는 800 count,
즉 duty는 `520/800=65%`이다.
ROS vehicle_controller와 bridge 및 ADC target을 만드는 도구도 이 운용값을 사용한다.
실측 운용 조향각은 RIGHT=-17.5° (약 -0.3054 rad)/ADC 150,
CENTER=0° (0 rad)/ADC 2132, LEFT=+16.1° (약 +0.2810 rad)/ADC 3950이다.
ROS vehicle_controller는 이 비대칭 angle calibration을 기본 활성화한다.
REP-103 양의 각도=LEFT, 음의 각도=RIGHT이며 각 구간은 선형 보간한다.
유한 범위 밖 각도는 운용 endpoint로 clamp하고, NaN/Inf·잘못된 calibration·
emergency·timeout은 기존 fail-safe STOP을 유지한다. 명시적으로 calibration을
끄면 중앙 허용 오차(0.001 rad)를 벗어난 nonzero angle은 계속 STOP 처리한다.
이 각도 설정은 ROS 변환 계층에만 적용되며 STM32 firmware 변경은 없다.
