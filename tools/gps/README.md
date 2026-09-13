# ZED-F9P 2대 Moving Base 재현 및 진단

이 문서는 사용자가 2026-09-13 야외 검증을 완료했다고 제공한 설정/결과를 저장한다.
GPS에는 이미 Flash 저장이 완료되어 있다. **정상 장치에는 설정 도구를 다시 실행할
필요가 없다.** 이번 구현/테스트에서는 실제 GPS에 접속하거나 설정을 쓰지 않았다.
ROS, perception, vehicle control과 독립된 USB CLI 도구다.

## 배선과 역할

- REAR GPS: Moving Base, 차량 뒤쪽 안테나.
- FRONT GPS: Rover, 차량 앞쪽 안테나.
- REAR UART2 TX → FRONT UART1 RX; GND 공통.
- 두 수신기의 USB를 메인 컴퓨터에 각각 연결.
- 검증한 안테나 간 물리적 baseline: **0.92 m**.
- RELPOSNED heading은 REAR → FRONT baseline 방향이며 북쪽 기준 시계 방향 각도다.
  ROS ENU yaw나 차량 제어 명령으로 변환하지 않는다.

포트는 장치/케이블 역할을 직접 확인해 지정한다. `/dev/ttyACM*` 번호는 재연결 때
달라질 수 있다. `/dev/serial/by-path`는 USB 물리 포트나 허브 위치를 바꾸면 달라진다.
동일한 모델의 USB by-id 이름도 고유하지 않을 수 있으므로 이름만으로 앞/뒤를
추측하지 않는다. 이 도구에는 자동 포트 선택이나 앞/뒤 역할 자동 판별이 없다.
다른 GPS reader/configuration 프로그램을 종료한 뒤 한 도구씩 실행한다.

## 저장된 설정값

설정의 유일한 코드 원본은 `f9p_common.py`의 `COMMON`, `REAR`, `FRONT`다.
아래 목록에 없는 key는 설정하지 않는다. Survey-in, 다른 GNSS, 추가 RTCM 메시지,
다른 인터페이스 설정 등을 임의로 초기화/비활성화하지 않는다. 따라서 이것은
제공된 성공 설정의 재현이며, 미제공 설정까지 포함한 factory-reset 복구 파일은 아니다.

공통:

| Key | Value |
|---|---:|
| CFG_RATE_MEAS | 1000 |
| CFG_RATE_NAV | 1 |
| CFG_SIGNAL_GLO_ENA | 0 |
| CFG_SIGNAL_GLO_L1_ENA | 0 |
| CFG_SIGNAL_GLO_L2_ENA | 0 |
| CFG_SIGNAL_BDS_ENA | 1 |
| CFG_SIGNAL_BDS_B1_ENA | 1 |
| CFG_SIGNAL_BDS_B2_ENA | 1 |

REAR / Moving Base:

| Key | Value |
|---|---:|
| CFG_UART2_BAUDRATE | 38400 |
| CFG_UART2OUTPROT_UBX | 0 |
| CFG_UART2OUTPROT_NMEA | 0 |
| CFG_UART2OUTPROT_RTCM3X | 1 |
| CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2 | 1 |
| CFG_MSGOUT_RTCM_3X_TYPE1074_UART2 | 1 |
| CFG_MSGOUT_RTCM_3X_TYPE1094_UART2 | 1 |
| CFG_MSGOUT_RTCM_3X_TYPE1124_UART2 | 1 |

FRONT / Rover:

| Key | Value |
|---|---:|
| CFG_UART1_BAUDRATE | 38400 |
| CFG_UART1INPROT_RTCM3X | 1 |
| CFG_NAVHPG_DGNSSMODE | 3 |
| CFG_USBOUTPROT_UBX | 1 |
| CFG_MSGOUT_UBX_NAV_RELPOSNED_USB | 1 |
| CFG_MSGOUT_UBX_RXM_RTCM_USB | 1 |

## 준비

저장소 루트에서:

```bash
python3 -m pip install -r tools/gps/requirements.txt
ls -l /dev/serial/by-path/
# 실제 확인한 앞/뒤 USB 경로로 아래 두 값을 교체한다.
REAR_PORT=/dev/serial/by-path/REPLACE_WITH_REAR
FRONT_PORT=/dev/serial/by-path/REPLACE_WITH_FRONT
```

`--baudrate` 기본 115200은 호스트 USB serial open 인수이며, 안테나 수신기 사이의
UART link는 위 설정값 38400이다. USB 호스트 옵션을 바꿔도 UART 설정값은 바뀌지 않는다.

## 설정 재현 (필요한 경우에만)

기본 또는 `--ram-only`는 CFG-VALSET RAM layer(mask=1)만 쓴다:

```bash
python3 tools/gps/configure_f9p_moving_base.py \
  --rear-port "$REAR_PORT" --front-port "$FRONT_PORT" --ram-only
```

명시적 `--save`만 RAM+BBR+FLASH(mask=7)를 쓴다:

```bash
python3 tools/gps/configure_f9p_moving_base.py \
  --rear-port "$REAR_PORT" --front-port "$FRONT_PORT" --save
```

두 포트를 모두 열고 MON-VER의 `MOD=ZED-F9P`를 확인한 뒤 각 수신기에 제공된 key만
한 CFG-VALSET 패킷으로 쓴다. 해당 명령의 ACK를 기다리고 VALGET으로 선택된 모든
layer를 대조한다. NAK, ACK timeout, readback 누락/불일치는 실패(exit 1)다.
Flash를 지원하지 않거나 firmware가 key를 거부해도 저장 성공으로 보고하지 않는다.
앞/뒤에 대한 쓰기는 원자적이지 않다. 중간 실패/종료 시 한 장치만 적용되었을 수
있으며 자동 retry, rollback, factory reset은 하지 않는다. 읽기로 현재 상태를 확인한다.

## 전원 재부팅 후 영구 설정 확인

정상 시스템에서는 아래 **읽기 확인부터** 한다. `--verify-only`는 MON-VER와
CFG-VALGET 요청만 전송하며 CFG-VALSET/CFG-CFG/reset을 전송하지 않는다.

```bash
# 전원을 다시 넣은 후 현재 적용 중인 RAM 설정 확인
python3 tools/gps/configure_f9p_moving_base.py \
  --rear-port "$REAR_PORT" --front-port "$FRONT_PORT" --verify-only --layer ram
# 저장된 Flash 설정 확인 (VALGET Flash layer ID는 2, SET mask 4와 다름)
python3 tools/gps/configure_f9p_moving_base.py \
  --rear-port "$REAR_PORT" --front-port "$FRONT_PORT" --verify-only --layer flash
```

필요하면 `--layer bbr`도 확인한다. RAM 일치만으로 Flash 저장을 증명하지 않는다.
Flash와 RAM을 각각 확인하고 다음 passive heading/RTCM 진단을 실행한다.

## Heading 진단

```bash
python3 tools/gps/check_f9p_heading.py --front-port "$FRONT_PORT" \
  --baseline 0.92 --tolerance 0.05 --duration 30
```

기존 USB UBX 출력을 읽기만 하며 메시지 출력 설정을 변경하거나 poll하지 않는다.
기본 1초마다 JSON 한 줄을 출력하고 30초 후 최종 상태와 종료 코드(0=PASS, 1=FAIL)를
반환한다. `--interval`, `--duration`으로 변경할 수 있다.

필드: `relPosLength_m`, `heading_deg`, `carrSoln`, `relPosValid`, `isMoving`,
`relPosHeadingValid`, `accHeading_deg`, `baseline_error_m`, `pass`, `reasons`,
`valid_heading_deg`. PASS는 다음을 모두 만족해야 한다:

- carrSoln=2 (RTK fixed).
- relPosValid=1, isMoving=1, relPosHeadingValid=1.
- |relPosLength−0.92| ≤ 0.05 m. **±0.05 m는 도구의 기본 진단 허용오차**이며,
  사용자가 제공한 GPS 설정값이나 측정 정확도 보증이 아니다. `--tolerance`로 변경한다.
- 새로운 navigation epoch를 받은 지 2.5초 미만 (`--stale-seconds`).

carrSoln=1(Float)은 항상 FAIL이며 `valid_heading_deg=null`이다. Raw `heading_deg`는
원인 분석용으로만 표시한다. 데이터 미수신/stale도 FAIL이다. 동일 iTOW 반복 메시지는
freshness를 연장하지 않는다. accHeading은 표시하되 제공되지 않은 임계값은 추가하지 않는다.
`isMoving`은 moving-base 동작 플래그이며 차량이 실제 이동 중이라는 판정이 아니다.
최종 PASS는 최신 샘플 판정으로, 관측 구간 전체의 fixed 유지율을 보증하지 않는다.

pyubx2 1.3.6은 고정밀 길이 성분을 relPosLength에 이미 합산한다. cm→m만 적용하고
heading/accHeading의 라이브러리 degree scaling을 다시 적용하지 않는다.

## RTCM 수신 진단

```bash
python3 tools/gps/check_f9p_rtcm.py --front-port "$FRONT_PORT" --duration 30
```

FRONT USB의 **UBX-RXM-RTCM 수신 보고**를 세므로 FRONT가 UART에서 받은 메시지를
관찰한다. REAR 송신이나 FRONT USB의 raw RTCM 출력과 혼동하지 않는다.
4072/1074/1094/1124 각각 `received`, `used`, `not_used`, `unknown`을 누적한다.
CRC 실패 보고는 message type도 신뢰할 수 없어 별도 `crc_failed_unattributed`로 센다.
4072의 subtype도 별도 표시하며 4072.0 수신 여부를 확인한다.

이 도구의 최종 PASS는 네 종류를 각각 CRC 정상으로 한 번 이상 받았고 4072.0도
관찰했다는 뜻이다. Fix나 heading PASS를 의미하지 않는다. `received`와 실제 솔루션에
`used`된 수는 다를 수 있다. USB 보고 손실이 있으면 실제 UART 수신량보다 작을 수 있다.
메시지가 전혀 없으면 FAIL이며 설정을 자동으로 켜지 않는다.

## 사용자 제공 실측 결과

- carrSoln=2, relPosValid=1, isMoving=1, relPosHeadingValid=1.
- relPosLength 약 0.90~0.93 m; 물리 baseline 0.92 m.
- 차량 오른쪽 약 90° 회전 시 heading 약 73° → 약 164° (약 +91°).
- accHeading 약 0.62°.

위 결과는 사용자 제공 현장 검증 기록이다. 이번 코드 테스트에서 다시 측정한 결과가 아니다.

## 오프라인 검증

```bash
python3 -m pytest -q tools/gps/tests
python3 -m compileall -q tools/gps
python3 tools/gps/check_f9p_heading.py --input recorded_front.ubx
python3 tools/gps/check_f9p_rtcm.py --input recorded_front.ubx
```

`--input`은 binary UBX 또는 mixed stream을 재생하며 serial을 열지 않는다.
파일 재생은 수신 당시 wall-clock timestamp를 복원하지 않으므로 현장 freshness 검증은
live passive 진단으로 한다. Synthetic unit tests에는 GPS 하드웨어 접속이 없다.

프로토콜/라이브러리 참고:
[pyubx2 configuration API](https://www.semuconsulting.com/pyubx2/pyubx2.html),
[u-blox ZED-F9P integration manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf).
