# 대회용 metric lane tracking

새 `competition_lane` entry / `competition_lane.launch.py`는 metric 후보 선택과
장시간 single boundary 추종을 사용한다. 기존 `lane_follow`, `lane_stop_test`의
tracker 선택은 그대로다. 모든 launch의 enable_drive 기본값은 false다.

## 추적 상태와 제어 허가

- PAIR_TRACK: 실제 두 boundary의 midpoint center를 추종한다. 높은 신뢰도이며
  경쟁 후보와 구분되는 PAIR에서만 실제 normal offset과 폭을 EMA로 갱신한다.
- SINGLE_LEFT_TRACK / SINGLE_RIGHT_TRACK: 현재 boundary의 heading/curvature를
  매 프레임 갱신하고, 학습한 해당 side의 offset으로 virtual center를 만든다.
  offset/폭은 single 프레임에서 갱신하지 않으며 고정 시간 만료가 없다.
- INVALID: boundary 소실, 낮은 신뢰도, identity jump, 모호한 경쟁 후보,
  비현실적 pair, normal curve의 fold/큰 근사오차 등. 지난 boundary만 재생해
  주행을 허가하지 않는다.

Perception 상태와 motion state는 별개다. 기존 motion-safe 관측의 5 NEW frame
획득, 카메라 freshness <0.25초, stop snapshot freshness <0.5초, control window와
관측 margin을 그대로 적용한다. single offset의 무기한 보존은 stale camera 주행을
허용한다는 뜻이 아니다. 유효 single↔pair 전환은 정상 획득 상태를 유지한다.

PAIR high의 drive_pwm 기본 120, SINGLE high의 single_drive_pwm 기본 64,
DEGRADED지만 valid인 결과의 degraded_pwm 기본 40이다. SINGLE/PWM 요청은 항상
설정 drive_pwm으로도 제한한다. PAIR degraded도 degraded_pwm을 사용하고 EMA를
갱신하지 않는다. 이는 CCR PWM 수치이며 실제 주행 속도 측정값이 아니다.

STOP_LINE, system/camera fault, emergency, obstacle stop은 lane 추종보다 우선한다.
STOP_APPROACH 진입/timeout/held steering/STOP_LINE 전환은 기존 정책을 따른다.
`/cmd/emergency`의 명시적 SET/CLEAR와 arbiter 우선순위는 유지한다.
`obstacle_stop_topic` 기본 `/safety/obstacle_stop`에 std_msgs/Bool 입력을 준비했다.
true는 STOP, 명시적인 false는 해제이며, 해제 후 일반 획득 조건을 다시 만족해야 한다.
기존 obstacle publisher와 자동 연결하지 않았으므로 실제 배선/토픽 연결은 별도 확인 대상이다.

## 후보와 pair 평가

HSV mask와 BEV calibration은 변경하지 않는다. Connected component를 미터로
변환하여 길이, row 방향 두께, aspect ratio를 먼저 검사한다. 짧고 굵은 block은
그룹화 전에 제외한다. 얇고 방향이 비슷한 component만 연결해 dashed lane도 평가한다.
row 중심의 metric RANSAC으로 polynomial을 fit하며 길이/두께/점수/잔차/inlier ratio,
observed y, heading/curvature, gaps/coverage, 이전 boundary 연속성을 기록한다.

Pair는 normal 방향 폭, y에 따른 폭 변화, heading/curvature 차이, 공통 overlap,
이전 boundary의 lateral continuity, 이전 학습 폭과의 차이로 점수화한다.
전역 plausibility 밖의 폭과 비현실적 형상은 제외한다. 학습 폭과의 차이는 주로
score penalty이며 특정 도로 폭을 hard gate로 삼지 않는다. 1·2위 pair score가
pair_score_gap보다 가까우면 INVALID다. 양쪽이 각각 신뢰 가능하지만 pair geometry가
불일치하면 임의로 single로 강등하여 진행하지 않는다.

주요 시작값(모두 competition parameter, legacy detector 설정과 별개):

| Parameter | 기본값 | 의미 |
|---|---:|---|
| nominal_lane_width_m | 3.5 | 미학습 cold single의 초기 폭 prior |
| min_lane_width_m / max_lane_width_m | 2.0 / 6.0 | 전역 물리적 plausibility |
| ema_alpha | 0.15 | 높은 신뢰도의 PAIR에서만 학습 |
| min_component_length_m | 0.25 | 작은 paint fragment 제외 |
| max_thickness_m / min_aspect_ratio | 0.30 / 3.0 | chunky block 제외 |
| min_length_m / max_gap_m | 0.9 / 1.2 | boundary 관측 길이와 간격 |
| residual_limit_m | 0.06 | metric fitting residual |
| lateral_jump_m | 0.65 | 이전 boundary 대비 급격한 이동 차단 |
| valid_confidence / high_confidence | 0.55 / 0.80 | INVALID/degraded/high 구분 |
| pair_score_gap | 0.06 | competing pair ambiguity |
| pair_return_blend_sec | 0.3 | single→pair steering blend |

같은 기본 config의 2.7/3.05/3.2/4.1/4.6m synthetic pair를 테스트한다.
nominal은 학습 전만 사용하며 정상 PAIR 이후 학습 offset을 우선한다. 학습 reset은
tracker.reset()/노드 재시작 시에만 한다. 현재 관측 side의 curve 변화로 보이지 않는
side의 continuity reference만 이동시켜 장시간 편측 추종 후 재획득을 지원한다.
이는 학습 폭/offset의 갱신이 아니다.

## Local normal과 fusion interface

x=f(y), s=f'(y), 왼쪽 boundary offset=d>0일 때:

```
x_center = f(y) + d / sqrt(1+s*s)
y_center = y    - d*s / sqrt(1+s*s)
```

오른쪽 boundary에는 -d를 적용한다. y까지 이동한 샘플을 다시 x(y)로 fit한다.
normal curve가 접히거나 근사오차가 residual_limit_m보다 크면 INVALID다. 실제
observed center 구간만 기존 control window와 교차하므로 관측 범위를 늘리지 않는다.
PAIR는 기존 pair_control_max_m(기본 field 3.5, 4.0 미만), SINGLE은 기존
single_side_control_max_m(최대 3.30)을 사용한다. 따라서 충분한 near observation이
없는 single은 perception이 valid여도 motion이 허용되지 않을 수 있다.

Tracker result의 source, virtual_center, boundary_heading(rad), boundary_curvature(1/m),
lateral_offset_target(m), confidence, offset_source, frame_timestamp를 fusion에 제공한다.
Adapter result에도 virtual_center와 boundary metadata를 유지한다. ROS의 기존
DriveCommand 형식/steering calibration은 바꾸지 않는다. source는 별도 tracking_source로
로그하며 control_source/motion_control_source는 실제 제어 선택인 PAIR_TRACK 또는
SINGLE_LEFT_TRACK/SINGLE_RIGHT_TRACK를 사용한다. 기존 controller 호출에만
PAIR/TEMPORARY_CENTER adapter를 사용한다.

`CompetitionLaneTracker(..., consistency_check=callback)`으로 보조 검사를 연결할 수 있다.
`RelativeHeadingConsistency.update(relative_heading_rad, timestamp, valid=True)` helper는
외부에서 quality/freshness 확인한 path heading−vehicle heading을 BEV의 오른쪽 양수
방향으로 변환해 받는다. 실제 F9P 토픽을 구독하지 않으며 GPS 없이도 작동한다.
optional data가 없거나 오래되면 해당 검사만 생략하고, fresh heading mismatch는 veto한다.

PAIR 상태는 재획득 즉시 복귀한다. 조향은 마지막 valid single command에서 현재 pair
command로 짧게 blend한다. invalid가 끼면 STOP/재획득 경로로 돌아가며 과거 조향을
계속 유지하지 않는다. STOP_APPROACH에는 기존 자체 steering 정책이 우선한다.

## Diagnostic / offline 검증

각 candidate에 L/R ID, length, thickness, heading, residual, score를 표시한다.
선택된 boundary는 굵은 초록색, virtual center는 자홍색, rejected component는 빨간색이다.
하단에 measured/EMA width와 1·2위 pair score를 표시한다. 로그에는 candidate_count,
selected_pair_score, second_pair_score, selected_lane_width_m, expected_lane_width_m,
width_delta_m, heading_diff_deg, pair_overlap_m, reject_reason과 source/confidence가 포함된다.

카메라/차량을 열지 않는 synthetic offline 명령:

```bash
python3 ros2_ws/src/fma_perception/tools/replay_competition_lane.py \
  --synthetic --max-frames 120 --output-dir /tmp/fma_competition_offline
```

실제 저장 영상은 --synthetic 대신 --video /path/to/recorded.mp4를 사용한다.
CLI는 존재하는 파일만 허용하며 camera device를 입력받지 않는다.
Live diagnostic tool에는 --competition-tracking 옵션을 추가했지만 자동 실행하지 않는다.

빌드 후 대회용 DRY-RUN 명령(사용자가 실행하며 카메라는 사용, 제어 publisher는 없음):

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch fma_control competition_lane.launch.py enable_drive:=false \
  device:=/dev/video14 drive_pwm:=120 single_drive_pwm:=64 degraded_pwm:=40 \
  nominal_lane_width_m:=3.5
```

Synthetic 테스트는 geometry/state/safety 정책의 동작을 검증한다. 실제 도로에서의
후보 선택 정확도와 confidence 수치는 현재 카메라 녹화 영상으로 추가 검증해야 한다.

## Control-window fallback과 출력 로그

선택된 PAIR가 outside_control_window이면 그 PAIR의 현재 신뢰 가능한 좌우 boundary를
각각 local-normal center로 변환하여 기존 single 제어창에서 평가한다. 평가 가능한
후보 중 confidence가 높은 쪽을 실제 motion source로 선택한다. 모호하거나 invalid인
perception 결과, secondary consistency veto는 이 fallback으로 우회하지 않는다.
학습 폭/offset과 tracker identity는 fallback 평가 중 변경하지 않는다.
STOP detector에는 원래 실제 boundary geometry를 전달한다.

tracking_source는 perception 선택, motion_control_source는 제어 선택이다.
motion_observed_min/motion_used_lookahead/motion_valid는 선택된 제어 geometry의
관측 범위와 평가 결과이며, requested_pwm은 lane 요청이다. published_pwm은 획득,
STOP, emergency 등의 정책을 적용하여 이 노드가 실제 publish한 PWM이다(DRY-RUN은 0).
하위 arbiter/MCU가 최종 적용한 값을 측정한 피드백은 아니다.
drive_enabled/configured_pair_pwm/configured_single_pwm은 현재 safety 출력과 분리한다.
