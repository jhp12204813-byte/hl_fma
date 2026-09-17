# D435i RGB 차선 검출 / C920 신호 영상

## 노트북 D435i 640×480 RGB 검증

전면 serial은 `142122070689`이다. 실측 차선 폭과 metric calibration은 아직
없으므로 `lane_width_m`, `meters_per_pixel_y`, `single_lane_width_px`를 모두
`0.0`으로 유지한다. 아래 실행은 perception만 시작한다.
각 터미널에서 먼저 환경을 설정한다.

```bash
cd ~/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
```

1. RealSense RGB only (설치된 Humble driver launch 인자 기준):

```bash
ros2 launch realsense2_camera rs_launch.py \
  serial_no:="'142122070689'" camera_namespace:=camera camera_name:=camera \
  enable_color:=true rgb_camera.color_profile:=640,480,30 \
  rgb_camera.color_format:=RGB8 enable_depth:=false \
  enable_infra:=false enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false enable_motion:=false \
  pointcloud.enable:=false align_depth.enable:=false
```

2. 차선 검출 입력 remap:

```bash
ros2 run fma_perception lane_detector --ros-args \
  -r /front/color/image_raw:=/camera/camera/color/image_raw \
  -p lane_width_m:=0.0 -p meters_per_pixel_y:=0.0 -p single_lane_width_px:=0.0
```

3. Debug overlay:

```bash
ros2 run rqt_image_view rqt_image_view /perception/lane/debug_image
```

4. Debug mask (별도 터미널):

```bash
ros2 run rqt_image_view rqt_image_view /perception/lane/debug_mask
```

640×480 기본 ROI는 x=0..639, y=288..479, lookahead y=312,
영상 중심 x=320.0이다. 양쪽 fit과 중심선은 보정 없이 표시되며 한쪽만
검출되거나 유효 폭 검사를 통과하지 못하면 중심을 추정하지 않는다.
`debug_image`에는 왼쪽 fit(노랑), 오른쪽 fit(자홍), 차선 중심선(초록),
영상 중심선(청록), lookahead 점(빨강)을 표시한다.
`lane_center_px`는 lookahead y에서 좌우 fit x의 평균이고,
`image_center_px`는 영상 폭/2이다. `pixel_error = image_center_px - lane_center_px`
이므로 차선 중심이 영상 중심 왼쪽이면 양수, 오른쪽이면 음수이다.
중심이 유효하지 않으면 중심 좌표와 pixel error 모두 `N/A`로 표시한다.
이 숫자는 debug overlay 전용이며 ROS Lane 메시지나 파라미터를 추가하지 않는다.
미보정 상태에서는 항상 metric `detected=false`, 물리 오차·confidence=0이다.
RGB8 입력은 cv_bridge가 BGR8로 변환하고 두 debug 출력은 모두 640×480 BGR8이다.

오프라인 테스트는 합성 640×480 RGB8 프레임의 좌우 이동, 한쪽 소실, 빈 화면,
두 debug publisher 호출 및 입력 header 보존을 검증한다. 실제 카메라 영상에서의
검출 품질과 debug topic 수신률은 노트북에 카메라를 연결하여 별도로 확인해야 한다.

센서 역할:

- 전방 D435i RGB: 차선, 향후 정지선 검출
- 전방 D435i Depth: 향후 정지선 거리·장애물 거리 (이번 구현 미구독)
- C920: 향후 신호등·신호차

RealSense ROS driver가 color 영상을 canonical /front/color/image_raw로
발행하도록 driver topic을 remap한다. lane_detector는 ROS Image만 구독하며
D435i를 VideoCapture로 직접 열지 않는다. 실제 driver 설치/launch는 별도이다.
C920 노드는 기존 /dev/fma_c920, MJPG 요청, 1280×720/30 FPS,
/camera/front/image_raw(bgr8, front_camera)를 유지하며 차선 입력으로 사용하지 않는다.

## 실행과 출력

ROS Humble와 workspace를 source한 뒤:

~~~bash
ros2 run fma_perception lane_detector
ros2 run rqt_image_view rqt_image_view
~~~

입력: /front/color/image_raw (sensor-data QoS, cv_bridge로 bgr8 변환).
출력: /perception/lane (Lane, reliable depth 1),
/perception/lane/debug_image와 /perception/lane/debug_mask (Image, sensor-data QoS).
촬영 stamp/frame_id는 입력에서 보존한다. GUI는 노드 내부에서 열지 않는다.
rqt에서 debug topic을 선택한다. Perception 자체는 command를 발행하지 않는다.

## 검출과 ROI

HSV 흰색 (S<=65, V>=170)과 노란색 (H=15..40, S>=75, V>=90) 마스크를
결합하고 morphology opening → Canny → Hough 선분 → 좌우 직선 fit을 수행한다.
영상 중심 기준으로 좌우를 나누고 수평 선분/작은 노이즈를 제거한다.
잔차 필터 및 세로 지지 범위, 차선 폭을 확인하고 중심선을 계산한다.

ROI, lookahead, HSV, morphology/필터는 runtime 변경 가능하다.
잘못된 범위·타입·min/max 순서·ROI/lookahead 조합은 거부하고 기존 설정을 유지한다.
물리 calibration(lane_width_m, meters_per_pixel_y, single_lane_width_px)은 물리 단위
해석을 운용 중 바꾸지 않도록 startup-only로 유지한다.
초기값은 --ros-args -p 이름:=값으로 지정한다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| roi_top_ratio | 0.6 | ROI 상단 |
| roi_bottom_ratio | 1.0 | ROI 하단 |
| roi_left_ratio | 0.0 | ROI 왼쪽 |
| roi_right_ratio | 1.0 | ROI 오른쪽 |
| lookahead_ratio | 0.65 | 영상 높이 기준 참조 행; ROI 내부/하단 이전 |
| lane_width_m | 0.0 | 실측 차선 폭(m), 0은 미보정 |
| meters_per_pixel_y | 0.0 | ROI 전방 거리 보정(m/pixel), 0은 미보정 |
| single_lane_width_px | 0.0 | 한쪽 검출 시 사용할 실측 lookahead 차선 폭(px), 0은 사용 안 함 |

ROI 계산은 순수 함수 roi_bounds로 분리했다. 현재 직사각형이며 trapezoid는 없다.
Overlay는 색상 마스크 후보, ROI, 왼쪽 노란색/오른쪽 자홍색 fit, 초록 중심선,
청록 영상 중심, 빨간 lookahead 점, 오차·confidence·detected를 표시한다.

## 물리 보정과 부호

양쪽 차선 검출 시 lookahead에서
scale_x = lane_width_m / observed_lane_width_px로 변환한다.
0보다 크고 0.05m 미만인 폭, 음수/NaN/Inf calibration은 startup 오류로 거부한다.
관측 폭이 영상 폭의 10% 이하 또는 95% 이상이거나 선이 교차하면 검출 실패이다.
미보정 기본값에서는 시각적 후보만 그리고 detected=false, 물리 오차/confidence=0이다.

- lateral_error_m = (영상 중심 x - lookahead 차선 중심 x) × scale_x
- heading_error_rad = atan2((하단 중심 x - lookahead 중심 x) × scale_x,
  (하단 y - lookahead y) × meters_per_pixel_y)

둘 다 LEFT 양수, RIGHT 음수다. 차량이 lane center 오른쪽에 있으면 lateral은
양수이고 양수 gain의 controller가 왼쪽으로 조향한다. 영상 중심을 차량 전방 중심으로
가정하므로 카메라 정렬을 확인해야 한다. 전방 scale은 실측 도로 평면에서 별도로
보정해야 하며 lane width만으로 대체할 수 없다. 이 모델은 ROI의 국소 일정 scale
근사이며 원근 보정/homography를 자동으로 수행하지 않는다. 보정과 실제 영상 검증 없이
정확한 물리 거리/heading이라고 해석하면 안 된다.

한쪽만 검출되면 기본적으로 detected=false/confidence=0이다. 실측 lane_width_m,
meters_per_pixel_y, single_lane_width_px가 모두 유효할 때만 측정된 선에서
반폭을 이동한 제한적 중심을 추정한다. 없는 반대편 선은 그리지 않는다.
이 경우 confidence=0.35이며 기본 controller threshold 0.6으로는 STOP이다.
양쪽 confidence는 선분 세로 지지 범위 기반 휴리스틱(확률 아님)이다.
Curvature=0은 미구현/unavailable이며 도로가 직선임을 보장하지 않는다.

## 오프라인 검증

의존성: rclpy, rcl_interfaces, sensor_msgs, fma_interfaces, cv_bridge,
python3-opencv, python3-numpy; tests: python3-pytest.
사용자 NumPy 2와 ROS binary cv_bridge/OpenCV 충돌 시 시스템 환경을 사용한다.

~~~bash
PYTHONNOUSERSITE=1 python3 -m pytest ros2_ws/src/fma_perception/test
~~~

Synthetic 영상과 mock VideoCapture만 사용하며 실제 장치 검증은 수행하지 않았다.

## 야간 영상 runtime 튜닝

~~~bash
ros2 param set /lane_detector white_v_min 150
ros2 param set /lane_detector yellow_s_min 100
ros2 param set /lane_detector morph_kernel_size 5
ros2 param set /lane_detector min_candidate_area_px 30
~~~

ROI를 바꿀 때 lookahead_ratio가 ROI 내부/하단 이전이어야 한다.
여러 값이 함께 바뀌어야 유효한 조합이면 set_parameters_atomically 서비스를 사용하거나
lookahead를 먼저 옮기는 등 각 중간 설정이 유효하도록 변경한다.
승인된 설정은 다음 frame부터 반영한다. 파라미터 검증 callback은 설정을 직접 바꾸지
않으므로 다른 callback이 변경을 거부하더라도 일부 값만 검출에 반영되지 않는다.

| HSV parameter | 기본값 |
|---|---:|
| white_h_min / white_h_max | 0 / 179 |
| white_s_min / white_s_max | 0 / 65 |
| white_v_min / white_v_max | 170 / 255 |
| yellow_h_min / yellow_h_max | 15 / 40 |
| yellow_s_min / yellow_s_max | 75 / 255 |
| yellow_v_min / yellow_v_max | 90 / 255 |

HSV는 정수이고 H는 0..179, S/V는 0..255, min <= max여야 한다.

| Filtering parameter | 기본값 | 의미 |
|---|---:|---|
| morph_kernel_size | 3 | Opening 정사각 kernel; 홀수 1..31, 1은 변화 없음 |
| min_candidate_area_px | 0 | 연결 성분 최소 면적; 0은 추가 면적 필터 해제 |
| min_line_length_px | 0 | Hough 최소 길이; 0이면 기존 max(10, ROI높이//5) 자동값 |
| max_line_gap_px | 0 | Hough 최대 gap; 0이면 기존 max(5, 영상높이//40) 자동값 |
| min_abs_slope | 1.0 | abs(dy)/abs(dx) 최소값; 수직 허용, 최소 dy=5px 유지 |

면적/길이/gap은 음수가 아닌 정수, slope는 음수가 아닌 유한값이다.
면적 필터는 morphology 이후 결합 마스크의 연결 성분에 적용한다.
기본 면적 필터 0과 기존 HSV/필터 기본값으로 ROI 외 동작 변화를 최소화했다.

/perception/lane/debug_mask는 ROI와 morphology/면적 필터를 통과한 색상 후보를
bgr8로 발행한다. WHITE는 흰색, YELLOW는 노란색이며 둘 다 해당하면 노란색이
우선 표시된다. Lane fitting/Hough 이전 결과이므로 threshold 조절을 직접 확인할 수 있다.
입력 header를 보존하며 rqt_image_view로 확인한다. calibration이 없어도 두 debug
영상과 선 후보는 계속 발행한다. detected=false 및 물리 오차/confidence=0 원칙은 유지한다.

## 저장 영상/사진 replay (노트북)

`lane_media_replay`는 로컬 MP4/AVI/MOV/MKV 및 JPG/JPEG/PNG를 읽어
`/front/color/image_raw`에 `sensor_msgs/msg/Image`, `bgr8`로 발행한다.
현재 ROS time을 stamp에 넣고 `frame_id=front_camera`를 사용한다.
원본 파일을 쓰거나 resize/crop하지 않는다. OpenCV가 디코딩한 해상도와
source FPS, publish FPS를 시작 로그로 출력한다.

| 시작 파라미터 | 기본값 | 의미 |
|---|---|---|
| media_path | 빈 문자열(필수 지정) | 로컬 영상/사진 경로 |
| loop | true | EOF에서 영상 반복; 사진은 동일 이미지 반복 |
| fps_override | 0.0 | 0은 원본 FPS, 읽을 수 없으면 30; 양수는 지정 FPS |

사진의 source FPS는 0으로 표시하며 기본 30 FPS로 반복한다.
`loop=false` 사진은 한 번 발행한다. 영상 EOF에서는 timer를 취소하고
파일을 닫는다. 노드는 Ctrl+C까지 남는다. 파일 경로/디코딩 오류는 실패로 종료한다.
파라미터는 시작 시 고정되며 topic은 ROS remap으로 변경할 수 있다.

각 터미널의 공통 준비:

```bash
cd ~/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
```

Terminal 1, 영상1 (카메라 또는 다른 replay publisher를 종료한 뒤 실행):

```bash
ros2 run fma_perception lane_media_replay --ros-args \
  -p media_path:="/home/idp2/차선영상1.mp4" -p loop:=true -p fps_override:=0.0
```

영상2로 바꿀 때 Terminal 1에서 Ctrl+C 후:

```bash
ros2 run fma_perception lane_media_replay --ros-args \
  -p media_path:="/home/idp2/차선영상2.mp4" -p loop:=true -p fps_override:=0.0
```

Terminal 2:

```bash
ros2 run fma_perception lane_detector --ros-args \
  -p lane_width_m:=0.0 -p meters_per_pixel_y:=0.0 -p single_lane_width_px:=0.0
```

Terminal 3:

```bash
ros2 run rqt_image_view rqt_image_view /perception/lane/debug_image
```

Terminal 4:

```bash
ros2 run rqt_image_view rqt_image_view /perception/lane/debug_mask
```

Terminal 5:

```bash
ros2 param dump /lane_detector
```

이 노트북에서는 사용자 site-packages NumPy 2와 시스템 OpenCV가 충돌하므로
`PYTHONNOUSERSITE=1`이 필요하다. 원본 약 2MP 영상 검출 속도는 30 FPS보다
낮을 수 있다. 상세 검토 시 replay에 `-p fps_override:=5.0`을 사용할 수 있다.
이는 재생 속도만 바꾸며 원본/검출 해상도는 유지한다.

두 대회장 영상 baseline/A/B 결과와 runtime 실험 명령은
[MEDIA_ANALYSIS_KO.md](MEDIA_ANALYSIS_KO.md)를 참고한다.
오프라인 샘플링 재실행 (입력은 원본 그대로, preview만 축소):

```bash
PYTHONPATH="$PWD/src/fma_perception:$PYTHONPATH" python3 \
  src/fma_perception/tools/analyze_lane_media.py \
  --output /tmp/lane_media_analysis \
  /home/idp2/차선영상1.mp4 /home/idp2/차선영상2.mp4
```

JSON의 left/right/visual은 검출기 출력이며 정답률이 아니다.
white/yellow는 morphology 전 ROI coverage, mask는 morphology 후 결합 coverage다.
속도는 디코딩/ROS/파일 저장/계측용 두 번째 호출을 제외한 detect_lane 한 번의 시간이다.
