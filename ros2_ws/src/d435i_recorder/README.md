# D435i recorder

독립 ROS 2 Humble / rqt recorder. 제어 명령을 발행하지 않는다. 기존 패키지와 root scripts에 의존하지 않는다.

## 실행

사용자 site-packages NumPy 2와 시스템 OpenCV/cv_bridge 충돌을 피하기 위해 다음 환경을 사용한다.

```bash
cd /home/idp2/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
colcon build --packages-select d435i_recorder --symlink-install
source install/local_setup.bash
# 카메라 포함 (이미 있으면 재사용)
ros2 run d435i_recorder record --with-camera
# 또는 이미 실행 중인 카메라 사용
ros2 run d435i_recorder record
# UI 없는 8초 실제 녹화, 기존 카메라 사용
ros2 run d435i_recorder record --seconds 8
# UI 없는 카메라 포함 8초 녹화
ros2 run d435i_recorder record --with-camera --seconds 8
```

launcher는 ROS graph를 2초 조회하고 camera node 또는 대상 topic publisher가 있으면 새 driver를 시작하지 않는다. ROS graph에 없더라도 호스트의 RealSense driver/launch process가 있으면 중복 시작을 거부한다. 패키지 launcher끼리는 파일 잠금으로 중복 시작을 막는다. 외부 도구가 동시에 driver를 시작하는 경우까지 원자적으로 막을 수는 없으며 녹화 시작/도중에는 필수 topic publisher가 정확히 하나인지 검사한다. launcher 종료 시 자신이 시작한 driver만 종료한다. 기존 카메라 설정은 변경하지 않으며 불일치하면 녹화를 거부한다. ROS_DOMAIN_ID 및 RMW 설정은 카메라와 같아야 한다.

rqt 단독 접근: `rqt --force-discover --standalone d435i_recorder.plugin.RecorderPlugin`. UI 기본 저장 위치는 `~/d435i_recordings_ku`; headless는 `--root`로 변경 가능. 종료 검증이 완료되면 MP4 재생 버튼을 사용할 수 있다. 시스템 기본 동영상 player를 사용한다. PASS 자동 재생은 체크박스로 제어한다.

## 실제 확인한 설정

전면 SDK serial `142122070689`, firmware `5.16.0.1`, USB `3.2`.
설치된 `/opt/ros/humble/share/realsense2_camera/launch/rs_launch.py`와 실제 파라미터 서비스를 확인했다.
wrapper `4.58.3`, librealsense `2.58.3`.

- `rgb_camera.color_profile=640,480,30`
- `depth_module.depth_profile=640,480,30` (`stereo_module` 아님)
- `enable_accel=true`, `enable_gyro=true`, `unite_imu_method=2`
- `enable_sync=false`, `align_depth.enable=true`
- 실제 기본 IMU profile: accel 100 Hz, gyro 200 Hz

RGB와 aligned depth 및 두 CameraInfo, color metadata, 통합 IMU를 `/camera/camera` 아래에서 구독한다. QoS는 sensor-data best effort이며 ROS/DDS 단계 유실까지 writer queue로 측정할 수는 없다. timestamp gap으로 의심 구간을 보고한다.

## Depth 단위

`depth_module.depth_units`는 이 장치의 실행 중 node에서 노출되지 않았다.
녹화 시작 때 SDK `rs2_get_depth_scale`로 장치 원시 scale을 **스트림을 시작하지 않고** 읽어 session에 기록한다. 장치 정보 서비스의 serial/firmware와 함께 확인한다.
ROS 저장 단위는 원시 scale과 구분한다. [wrapper 4.58.3 소스의 fix_depth_scale](https://github.com/realsenseai/realsense-ros/blob/4.58.3/realsense2_camera/src/base_realsense_node.cpp)은 Z16 값을 mm로 정규화한다. 따라서 검증한 이 wrapper 버전에서만 ROS scale `0.001 m/unit`을 사용한다. 다른 wrapper 버전이면 녹화를 거부한다. 0은 invalid이다.

## 파일과 의미

각 세션은 마이크로초까지 포함한 새 디렉터리에 저장하며 기존 세션을 덮어쓰지 않는다.

| 파일 | 내용 |
|---|---|
| color.mp4 | overlay 없는 BGR→mp4v, 640×480, configured 30 FPS |
| frames.jsonl | 0-based index, ROS timestamp ns, frame_id, 수신 monotonic/wall ns, encoding |
| depth/000000.png | aligned depth uint16 무손실 PNG |
| depth_frames.jsonl | filename, index, ROS timestamp, frame_id, 수신 시간, scale |
| imu.jsonl | ROS timestamp, frame_id, acceleration/angular velocity, orientation 및 각 covariance |
| device_metadata.jsonl | driver `json_data` 문자열 그대로 보존, header/수신 시간 |
| stop_lines.jsonl | 후보 bbox [x,y,width,height], RGB/depth timestamp, 차이, optical_z_m, 유효 pixel 수/비율, 상태 |
| session.json | 장치/설정/CameraInfo, 시각, counts/errors/overflow, scale, 검증 요약 |
| verification.json | PASS/WARN/FAIL, 원인, 전체 decode count, 각 stream timing (first/last ROS ns 포함) |

MP4 시간은 fixed 30 FPS 재생 시간이다. 실제 시간 분석에는 frames.jsonl을 사용하고 RGB와 depth는 줄 번호가 아닌 timestamp로 대응시킨다. JSON timestamp는 정수 ns이다. orientation은 원문 보존일 뿐 자세 추정 기능이 아니다.

## 정지선 후보와 거리

OpenCV HSV S≤65, V≥170, 영상 하단 60%, 가로폭≥30%, w/h≥4, closing 3×9 baseline. 노란 차선은 saturation 조건으로 제외한다. 약 10 Hz 이하 분석. UI bbox overlay는 저장 원본에 들어가지 않는다.
writer는 최근 90 depth frame을 보관하고 후보를 약 120 ms 기다려 뒤늦게 도착한 depth도 대응시킨다. 가장 가까운 timestamp 차이가 20 ms 이하일 때만 거리 계산한다. 한 depth가 여러 RGB와 대응할 수 있다. 매우 늦게 도착하거나 버퍼 밖인 depth는 no_match로 남는다. bbox 내 0.1~10 m 유효값이 20 pixel 이상이고 30% 이상이면 중앙값을 optical_z_m로 기록한다. 이 값은 카메라 optical axis의 Z이며 범퍼/지면/정지 거리가 아니다. 실제 정확도는 미검증이다.

## 오류/검증

512개 제한 입력 queue와 MP4/JSONL writer, 2개 depth worker(최대 32개 pending 작업). callback은 복사/직렬화/queue 입력만 하며 MP4/PNG I/O는 하지 않는다. overflow는 count와 오류를 기록하며 watchdog이 FAIL 종료한다. RGB/depth/IMU 중 3초 이상 미수신, 중복 publisher, 변환 오류도 안전 종료한다. 시작 전 필수 stream 준비 확인으로 startup 지연을 구분한다. Stop은 새 입력을 차단하고 queue를 비운 뒤 background thread에서 검증한다. 긴 녹화의 검증은 JSONL 목록/타임스탬프를 메모리에 읽으므로 세션 크기에 비례하는 RAM이 필요하다. 강제 종료/전원 차단 복구는 구현하지 않았고 미완료 세션에는 state=recording이 남는다.

전체 MP4 decode 및 RGB JSONL count 비교, 모든 PNG의 uint16/해상도/존재, IMU NaN/Inf/구조, timestamp 역전/중복/span/interval 평균·95백분위·gap/실제 Hz, CameraInfo, metadata, queue 오류와 stop line 구조를 검증한다. metadata 누락, 중복 timestamp, FPS 편차 및 짧은 gap은 WARN; 필수 데이터 누락/손상, timestamp 역전/3초 gap, writer 오류는 FAIL.

실제 장치 startup에서 `IMU Calibration is not available` 경고가 관측되었다. 이 알려진 장치 경고는 session 및 verification에 남기므로 파일 무결성에 문제가 없어도 WARN이 될 수 있다. PASS는 정지선 정확도 또는 완벽한 센서 동기화를 보장하지 않는다.

## 테스트

```bash
cd /home/idp2/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest src/d435i_recorder/test -q
PYTHONNOUSERSITE=1 colcon build --packages-select d435i_recorder --symlink-install
cd /home/idp2/fma_autonomous_vehicle
git diff --check
```

실제 Qt UI smoke test (기존 camera 필요, 새 5초 세션 생성):

```bash
PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen /usr/bin/python3 src/d435i_recorder/test/hardware_ui_smoke.py
```

검증 기록은 [VALIDATION.md](VALIDATION.md)를 참조한다.

## keyboard_teleop과 동시 실행

Recorder와 keyboard_teleop은 별도 프로세스로 실행한다. Recorder는 카메라 topic 6개만
구독하고 camera device_info/get_parameters 서비스만 읽는다. 차량 제어 topic의
publisher/subscriber를 만들지 않으며 teleop, STM32, mission, lane_controller와 연결하지 않는다.

W/A/S/D/SPACE/X 전역 단축키, keyboard grab, 전역 key event filter를 등록하지 않는다.
Recorder 버튼/체크박스는 마우스 조작용(Qt.NoFocus)으로 SPACE가 버튼 동작으로 이어지지 않는다.
외부 player 실행이 teleop 포커스를 바꾸지 않도록 PASS 자동 재생은 기본 OFF이며 필요할 때만 켠다.

터미널 기반 keyboard_teleop은 **해당 터미널에 포커스가 있을 때** 키 입력을 받는다.
Recorder 창을 클릭하거나 폴더/MP4 앱을 열었다면 운전 키 입력 전에 teleop 터미널로
포커스를 돌린다. Recorder가 다른 창으로 키를 전달하거나 차량 제어를 대신하지 않는다.
GUI 조작 없이 녹화하려면 별도 터미널에서 `ros2 run d435i_recorder record --seconds 60`
(카메라가 없으면 `--with-camera` 추가)을 실행하고 teleop 터미널에 포커스를 유지한다.

## Writer overflow 수정 및 진단

[OVERFLOW_DIAGNOSIS.md](OVERFLOW_DIAGNOSIS.md)에 기존 실패 세션의 원인과 측정값을 기록했다.
Depth PNG는 16-bit 무손실을 유지하며 filter None/zlib 0으로 인코딩한다. 파일 크기는
약 615 KB/장(30 FPS에서 depth만 약 1.1 GB/분)으로 증가한다. Queue 용량 512와
watchdog 3초는 유지한다. session/verification의 writer_metrics에 queue peak, overflow
시각/stream, MP4/PNG write latency 및 queue wait를 기록한다. stop_details에는
자동/수동/시간 지정 종료 이유와 마지막 stream 수신 경과 시간이 남고 UI에도 표시된다.
평균 FPS가 허용 범위여도 timestamp 기반 missing frame 추정치는 별도 WARN으로 보고한다.

OpenCV 내부 thread는 recorder 프로세스에서 1개로 제한한다. rqt는 문서의 standalone 명령으로 실행하는 것을 권장한다. Depth 작업은 병렬 처리하되 JSONL index/filename 순서는 입력 순서를 유지하며 종료 시 전부 drain한다.

DDS history는 RGB/depth/metadata 각각 30개, IMU 400개로 제한해 짧은 executor 지연을 흡수한다.
IMU callback은 고정 필드 직렬화로 원래 값/공분산을 그대로 보존한다. Writer overflow와
DDS 수신 누락은 별개이며, writer overflow가 없어도 timestamp gap은 경고로 남는다.

## A/B 성능 진단 후 변경

[AB_MEASUREMENT.md](AB_MEASUREMENT.md)에 6가지 25초 실제 장치 비교, 추가 executor/QoS
대조시험 및 최종 60초 검증 결과를 기록한다. `test/ab_device.py`는 진단 전용으로,
OFF 조건의 불완전한 세션을 정상 데이터로 사용하면 안 된다.

영상 구독은 실제 wrapper publisher와 맞춘 RELIABLE이며 IMU/metadata는 sensor-data
best effort를 유지한다. 따라서 위 기존 문서의 모든 topic best effort 설명은
이 변경 이후 영상에는 적용되지 않는다. BEST_EFFORT만 제공하는 외부 camera 설정은
호환되지 않으며 시작 시 영상 수신 대기에서 확인할 수 있다.

rqt plugin은 전용 ROS node와 SingleThreadedExecutor를 소유하며 shared rqt executor와
분리한다. 차량 topic 연결은 없다. 정지선 후보 추출은 별도 한 개 thread와 최대 8개
대기 분석 작업으로 분리하고 종료 시 결과를 모두 저장한다. 녹화 중 10Hz preview는
최근 완료된 후보 bbox를 재사용하므로 원본 영상보다 지연될 수 있다. 저장 영상에는
overlay가 없다. 분석을 기다리는 동안에도 depth는 timestamp로 매칭하며 오래되어
매칭되지 않는 경우 no_match로 기록한다.
