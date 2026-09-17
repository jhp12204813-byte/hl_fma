# C920 구현 / 장치 검증 (2026-09-11)

대상: 사용자 확인에 따라 `/home/idp2/fma_autonomous_vehicle`. 요청에 적힌 `/home/sy`는 현재 호스트에 없음.

## 사전 조사

- `git status --short` 확인 및 `/tmp/c920_before_status.txt` 저장. 기존 dirty 파일 유지.
- C920: USB `046d:08e5`, HD Pro Webcam C920, `/dev/video0` Video Capture. `/dev/video1`은 Metadata Capture이며 사용하지 않음.
- YUYV / MJPG 지원. 640×480@30 두 포맷 지원. MJPG는1280×720@30 및1920×1080@30도 열거됨. 고해상도 실제 녹화는 미시험.
- 설치 드라이버: usb_cam 확인, v4l2_camera 미검출. usb_cam의 설치 params_1.yaml에 있는 `mjpeg2rgb` 사용.
- 테스트 전 카메라 프로세스/장치 점유 없음. 기존 프로세스 kill 없음.
- D435i 실행 구조는 별도 sh가 아니라 `d435i_recorder.runner` console entry point. 기존 루트 tools 없음. 요청대로 `tools/record_c920.sh` 신규 생성, 내부는 같은 guarded runner 방식.

## 코드 결과

새 c920_recorder만 생성. stop_line.candidates의 RGB 함수는 D435i 원본을 복사해 독립 사용. Depth/IMU/RealSense import 없음. 원본 D435i 파일 수정 없음. 새 launcher는 기존 publisher 또는 점유 장치가 있으면 중복 시작을 거부하고 `--existing-camera` 사용을 안내함. 녹화 종료는 자신이 생성한 subprocess에만 신호를 보냄.

8 unit tests PASS: RGB-only 정상 저장, MP4 전체 decode/원본 무오버레이, 노란색 제외·흰색 후보, timestamp duplicate, 손상 MP4, count mismatch, >=3s gap, stop schema, writer exception drain, overflow, startup readiness/monotonic watchdog. `bash -n tools/record_c920.sh` PASS. `rqt --force-discover --list-plugins`에서 C920 plugin 확인.

## 실제 장치 결과

ROS: `/c920/image_raw`, sensor_msgs/msg/Image, encoding `rgb8`. Publisher `/c920/c920_camera`, usb_cam_node_exe. topic hz 독립 측정29.696~30.001Hz. 아래 평균은 저장 ROS timestamp 전체 구간으로 계산하므로 짧은 topic hz 측정과 다를 수 있음.

|세션|시험|저장/전체decode|평균FPS|최대ROS gap(s)|queue peak/cap|overflow|결과|
|---|---|---:|---:|---:|---|---:|---|
|20260911_201124|15초 headless|439/439|29.2703|0.100006|1/128|0|PASS|
|20260911_201410|25초 camera+recorder|743/743|29.7214|0.068044|1/128|0|PASS|
|20260911_201412|10초 rqt UI|297/297|29.6057|0.068044|1/128|0|PASS|

PASS는 파일 무결성 기준이며 무손실 수신 보증이 아니다. 1.5 frame period 초과 간격은 warnings와 estimated_missing_periods에 보존했다. 첫15초 시험은11 missing periods 추정,10초 UI는4개 추정. 장치 frame-number 정보가 없어 실제 device drop과 DDS/수신 지연을 확정 구분하지 않는다. 최대 receive gap은 각각102.23ms/69.93ms/68.48ms. MP4는 고정30FPS 컨테이너이므로 수신 gap이 있으면 재생길이가 녹화 wall-time과 다르다. 실제 timestamp는 frames.jsonl 기준으로 보존.

rqt는 offscreen Qt + 실제 C920 topic으로 미리보기 렌더링,Start/Stop 클릭,10초 녹화,자동 검증 PASS,play/folder 활성 확인. QDesktopServices.openUrl을 테스트에서 intercept하여 PASS autoplay/수동play/folder 정확한 경로 호출을 확인함. 외부 플레이어가 실제로 화면에 뜨는지/영상 재생은 미시험. preview screenshot은 UI 세션의 preview_ui.png.

첫 UI 시험은20초 시험 카메라 종료가 UI10초 녹화보다 먼저 도달해 RGB watchdog FAIL. 카메라 종료 후3.09초에 AUTO STOP 이유가 기록됨. 카메라 준비 직후 UI를 시작하는 순서로 재시험하여 PASS. 이 실패 세션도 삭제하지 않음.

## 남은 사항

- CameraInfo calibration 파일 미설정 경고: RGB 녹화 무결성에는 요구하지 않음. geometry 사용 전 별도 calibration 필요.
- usb_cam의 white_balance_temperature_auto/exposure_auto/focus_auto 이름이 현재 UVC control 이름과 달라 unknown control 경고. 디폴트 노출/초점의 목표값 적용 여부는 보증하지 않음. 드라이버 소스 변경하지 않음.
- 간헐 frame gap 존재. queue overflow는0이며 gap 원인 세분화는 이번 시험 범위 밖.
- stop-line은 RGB 후보이며 crosswalk 등 의미 판정 정확도 미검증. 거리 생성하지 않음.
- 720p/1080p 실제 녹화, 장시간 시험 미실시.
- 기존 source2140개 해시 변화 없음. git add/commit/push, 차량 구동/STM32 명령 없음.
