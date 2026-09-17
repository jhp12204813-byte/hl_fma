# Validation — 2026-09-09, Asia/Seoul

## IMPLEMENTED

독립 rqt RGB preview, start/stop, 상태/세션 경로/검증 결과, 폴더/MP4 열기,
PASS 자동 재생 checkbox. MP4 원본에 preview overlay를 저장하지 않는다.
RGB MP4 및 frame JSONL, aligned depth uint16 PNG 및 JSONL, 통합 IMU와
원문 color metadata, session metadata, 흰색 정지선 baseline 및 optical Z 계산,
제한 queue와 writer thread, 3초 watchdog, 자동 PASS/WARN/FAIL 검증.
카메라 포함/기존 카메라 재사용 실행, 중복 publisher 검사와 시작 프로세스 정리.
제어 명령 publisher나 기존 차량 패키지 연결 없음.

## DEVICE VERIFIED

호스트에서 SDK enumerate 및 driver parameter/topic/device_info 서비스를 실제 확인했다.

- D435I SDK serial 142122070689 / firmware 5.16.0.1 / USB 3.2.
- wrapper 4.58.3, 실행 중 librealsense 2.58.3.
- 실제 `depth_module.depth_profile`, `rgb_camera.color_profile`: `640,480,30`.
- align_depth=true, sync=false, accel/gyro=true, unite_imu_method=2.
- SDK 원시 scale: 0.0010000000474974513 m/unit. 스트림 시작 없이 조회.
- ROS Z16 저장 scale: wrapper 4.58.3의 mm 정규화 구현을 확인해 0.001 m/unit 사용.
- CameraInfo 2개, RGB, aligned depth, IMU, color metadata 모두 수신.
- color metadata에는 frame_number/frame_counter, hw/sensor/backend timestamp,
  auto_exposure, actual_fps 등의 키가 실제 포함됐다. 별도의 exposure/gain 숫자 키는
  이번 샘플에 없었으며 임의로 만들어 넣지 않았다.
- IMU orientation_covariance[0] = -1. 자세 추정 값으로 해석하지 않는다.

| 세션 (/home/idp2/d435i_recordings/) | 경로 검증 | RGB / depth / IMU / metadata | 실제 RGB/depth/IMU Hz | 결과 |
|---|---|---|---|---|
| 20260909_165700_792652 | existing camera, 8초 | 240 / 241 / 1601 / 244 | 29.25 / 29.25 / 199.64 | WARN |
| 20260909_170102_456253 | Qt UI 버튼, 5초 | 154 / 151 / 1004 / 154 | 29.59 / 29.20 / 199.56 | WARN |
| 20260909_170313_224909 | --with-camera가 기존 driver 재사용, 5초 | 153 / 153 / 1003 / 155 | 29.38 / 29.38 / 199.46 | WARN |
| 20260909_170531_120068 | --with-camera 신규 driver 시작/종료, 5초 | 151 / 151 / 1003 / 154 | 29.46 / 29.46 / 200.09 | WARN |

모든 세션에서 MP4 전체 decode count가 frames.jsonl과 일치했고, 모든 depth PNG가
640×480 uint16로 검증됐다. writer error/overflow 0, verification failures 0.
WARN 사유는 알려진 장치 IMU calibration 부재이다.
첫 8초 세션 stop_lines 82개, 마지막 세션 44개. 실제 장면에 후보가 없어서
no_candidate가 저장됐으며 실제 광학 Z 거리 정확도를 검증한 것은 아니다.

rqt --force-discover --standalone으로 실제 plugin 로드를 확인했다.
Qt offscreen에서 실시간 RGB pixmap 렌더링, UI start/stop, 자동 검증 상태 표시,
폴더/재생 버튼 활성화를 검증했다.
증빙: `/home/idp2/d435i_recordings/20260909_170102_456253/preview_ui.png`.
해당 이미지에는 현장 RGB가 포함된다.

새 driver 초기화 중 parameter 서비스가 빈 응답을 먼저 반환하는 실제 문제를 발견했고,
1초 주기 재조회와 회귀 테스트로 수정했다. 최종 신규 driver 실행 테스트는 종료 코드 0.
작업에서 시작한 driver는 종료했으며 마지막 ROS node 조회는 비어 있었다.

## NOT VERIFIED

- 실제 대회 차선/정지선 검출 정확도, optical Z 실측 정확도.
- 실제 모니터에서 사용자 조작 및 기본 OS 동영상 player/폴더 앱 실행.
- PASS 자동 재생의 실제 외부 player 재생 (실제 세션은 모두 calibration WARN).
- 장시간 녹화/디스크 고갈/물리적 USB 분리 시험. watchdog/overflow/codec 오류는 단위 테스트로 검증.
- 데이터 무결성 PASS는 검출 정확도나 완벽한 sensor 동기화 보장이 아니다.

## WARNINGS

- SDK startup: IMU Calibration is not available; default intrinsic/extrinsic 사용.
- 첫 startup에서 HID set_power 경고도 관측됐지만 통합 IMU는 약 200 Hz로 정상 수신됐다.
- 사용자 site NumPy 2.2.6과 시스템 OpenCV/cv_bridge ABI 충돌: PYTHONNOUSERSITE=1 필요.
- 실제 ROS timestamp는 MP4 fixed 30 FPS와 차이가 있으므로 시간 분석은 JSONL을 사용한다.
- best-effort DDS 유실은 queue overflow count에 포함되지 않는다. timestamp gap으로 별도 평가한다.
- 강제 종료/전원 차단 복구 기능 없음. 긴 세션의 offline 검증은 JSONL 크기에 비례하는 RAM 필요.

## TESTS

`PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest src/d435i_recorder/test -q`

23 passed. 세션 생성, MP4 decode 및 overlay 비포함, uint16 PNG, RGB metadata/JSONL,
IMU serialization, white/yellow candidate, 20ms 경계/미래 nearest timestamp,
median/valid ratio/minimum pixels, queue overflow, codec fatal drain, corrupt MP4/PNG,
NaN/역전/중복 timestamp, stop result 불일치, CameraInfo 누락, PASS/WARN/FAIL,
watchdog startup grace 및 3초 결손, package manifest, 초기 parameter 빈 응답 재시도 포함.
하드웨어 검증은 위 DEVICE VERIFIED와 별도 수행했다.

## BUILD / GIT DIFF CHECK / GIT STATUS

`PYTHONNOUSERSITE=1 colcon build --packages-select d435i_recorder --symlink-install`

새 ament_python 패키지 빌드 성공. rqt plugin 검색 목록 등록 확인.
`git diff --check` 통과. untracked 새 패키지 파일들도 `git diff --no-index --check`로
별도 whitespace 검사했다.

기존 미커밋 파일을 수정하지 않았다. 변경 범위는 새 `ros2_ws/src/d435i_recorder/`이다.
빌드 산출물과 새 실측 세션은 각각 workspace build/install/log 및 지정 recording root에 생성했다.
git add/commit/push/reset/clean/checkout은 실행하지 않았다.

## RUN COMMAND

```bash
cd /home/idp2/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
source install/local_setup.bash
ros2 run d435i_recorder record --with-camera
# 이미 실행 중인 카메라만 사용:
# ros2 run d435i_recorder record
```

## FILES CREATED/MODIFIED

기존 파일 수정 없음. 다음 파일을 새 패키지 안에 생성했다.

- `.gitignore`
- `README.md`
- `VALIDATION.md`
- `d435i_recorder/__init__.py`
- `d435i_recorder/device.py`
- `d435i_recorder/plugin.py`
- `d435i_recorder/recording.py`
- `d435i_recorder/runner.py`
- `d435i_recorder/stop_line.py`
- `d435i_recorder/verification.py`
- `d435i_recorder/writer.py`
- `package.xml`
- `plugin.xml`
- `resource/d435i_recorder`
- `setup.cfg`
- `setup.py`
- `test/conftest.py`
- `test/hardware_ui_smoke.py`
- `test/test_recorder.py`

## keyboard_teleop independence follow-up

Recorder의 전역 키 shortcut/grab은 없으며, 버튼과 checkbox를 Qt.NoFocus로 설정했다.
W/A/S/D/SPACE/X가 recorder 기능을 실행하지 않는 Qt 테스트와 마우스 클릭 동작을 확인했다.
PASS 자동 재생은 포커스 이동을 피하기 위해 기본 OFF로 변경했다.
카메라 6개 topic 구독과 2개 읽기 서비스만 사용하는 연결 경계 테스트를 추가했다.
차량 publisher는 만들지 않으며 기존 keyboard_teleop 및 차량 코드는 수정하지 않았다.

추가 파일: `test/test_independence.py`.
전체 단위 테스트: 25 passed. 이번 추가 검증에서 실제 차량/teleop은 실행하지 않았다.
운전 키 수신에는 teleop 터미널의 OS 포커스가 필요하다. Recorder 창을 클릭한 상태에서도
터미널이 입력을 받는다고 보장하지 않는다.

## Overflow follow-up

See [OVERFLOW_DIAGNOSIS.md](OVERFLOW_DIAGNOSIS.md) for original failed-session evidence,
writer/PNG changes and repeated 32-second device results. Final regression suite: 33 passed.
Original automatic stop due to writer overflow resolved; 32-second Qt session overflow 0,
queue peak 23/512, full file verification without errors. RGB/depth ~27 Hz remains below
the requested 29–30 Hz target; missing-period warnings are retained. Camera receive-only
baseline ~29 Hz shows recorder load still contributes. IMU calibration is a separate warning.

## A/B and 60-second follow-up

[AB_MEASUREMENT.md](AB_MEASUREMENT.md) contains six 25s A/B cases and additional
executor/reliability controls. Final combined fix: reliable image subscriptions,
rqt-owned single executor, bounded asynchronous candidate extraction, preview bbox reuse.
Two 60s recordings completed with no automatic stop/overflow; latest RGB/depth
29.879/29.283Hz, exact receive/save counts1806/1770, queue peak34/512. All files decoded.
Depth ~200ms long gap and timestamp-inferred frame losses persist (WARN); the
long-gap-free target is NOT met. Regression suite34 passed; build/whitespace pass.
