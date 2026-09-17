# BEV 차선 검출 단계적 구현 보고

## 기존 구조와 호환성

작업 시작 시 기존 tracked/untracked 파일 해시와 perception 사본을
`/tmp/fma_bev_before`에 보관했다. 기존 미커밋 변경을 되돌리지 않았다.

| 확인 항목 | 기존 | 추가 BEV 경로 |
|---|---|---|
| 입력 | `/front/color/image_raw` | 동일 + `/front/color/camera_info`; launch에서 D435i 토픽으로 remap |
| 출력 | `/perception/lane`, `debug_image`, `debug_mask` | 동일 + `debug_bev`, `diagnostics` |
| 왜곡 보정 | 없음 | CameraInfo K/D 기반 remap 캐시; 크기/모델/frame_id 확인 |
| ROI | 하단 40%, 가로 전체 | 이미지 ROI와 설정된 BEV source polygon의 교집합; 예시 ROI 상단 .42 |
| HSV | WHITE H0..179 S0..65 V170..255, YELLOW H15..40 S75..255 V90..255 | 기존 설정 재사용; 색 선택 별도, D435i 예시는 yellow |
| morphology | 3×3 opening, 선택적 최소 component 면적 | opening 유지; component 전체 형상 veto 없음 |
| Hough | Canny→HoughP, 기울기 필터 | 새 경로에서는 사용하지 않음 |
| 좌우 분류 | 선분 midpoint가 화면 중앙 좌/우인지 판단 | histogram 다중 seed→window 추적→곡선 순서/폭/차량 기준점 관계 |
| fitting | x=a·y+b, residual 제거 후 재적합 | x=a·t²+b·t+c, t=y/(BEV높이−1); RANSAC+재적합 |
| center/pixel_error | 좌우 직선 평균, lookahead에서 화면중앙−차선중앙 | 좌우 곡선 평균, BEV 차량 기준점−중심곡선 |
| lookahead | 이미지 y=.65h | 별도 BEV lookahead 비율; 실제 전방거리는 보정 후 계산 |
| 한쪽 차선 | 실측 폭이 있으면 제한적 추정 | 후보만 표시; 반대편/중심/metric 생성하지 않음 |
| 차선 소실 | 현재 프레임 false | 즉시 false/0, 이전 곡선·확인 횟수 초기화 |
| confidence | endpoint span 기반, 보정 전 0 | row coverage×inlier ratio×residual 품질; 시각 점수와 제어용 점수 분리 |
| curvature | 항상 0 | 보정된 중심곡선의 부호 있는 곡률(1/m) |
| 폭 sanity | ROI 상하 2곳 10..95% 폭 | 25개 위치 폭 범위/변동, 접선 차이, 후보 쌍 ambiguity |
| temporal | 없음 | 연속 검출 확인, 좌우 계수 EMA, jump veto, timestamp 역전/gap 초기화 |

기존 `Lane.msg`에 lateral_error_m, heading_error_rad, curvature, confidence가
모두 있으므로 interface를 수정하지 않았다. 헤더는 입력 그대로 보존한다.
기본 `pipeline=hough`와 기존 `detect_lane` 구현은 바꾸지 않았다.
`pipeline=bev`는 명시적 선택이며 실패 시 Hough를 자동으로 제어 출력에 사용하지 않는다.
Hough는 기존 소비자 호환 및 비교/debug 경로로 유지한다.

## 새 파일과 변경 이유

- `fma_perception/bev_lane.py`: 독립적인 순수 영상/상태 추적 모듈.
- `fma_perception/lane_detector_node.py`: pipeline 선택, CameraInfo와 BEV/debug JSON 출력 연결.
- `config/d435i_bev.yaml`: 실측되지 않은 값은 비워 둔 설정 템플릿.
- `launch/d435i_bev.launch.py`: perception만 실행하고 입력을 remap. 카메라/제어 노드 실행 없음.
- `setup.py`, `package.xml`: config/launch 설치 및 직접 의존성 선언.
- `test/test_bev_lane.py`, `test/test_lane_perception.py`: 수식·곡선·손실·노드 계약 검증.
- `tools/regress_bev_phone.py`: 기존 휴대폰 false-positive 재현과 비교 이미지/JSON 생성.

## 실제 픽셀 수집과 방어 로직

BEV binary mask에서 각 행의 연속 도색 run을 수집하고, run 폭이 너무 넓은 행만
제외한다. 정상 차선과 정지선이 연결돼도 다른 행의 정상 차선 픽셀은 유지한다.
각 window에서 예측 위치에 가장 가까운 run을 행마다 선택한다. fitting 입력은
각 행의 run 중심으로, 두꺼운 도색이 픽셀 수만으로 적합을 지배하지 않게 한다.
현재 diagnostics의 `pixels`는 실제 foreground 총 픽셀 수가 아니라 수집한 행 중심 수다.
최소 도색 중앙폭으로 산발적인 2px 잡음 추적을 제한한다. 이는 tuning parameter이며
멀리 있는 얇은 차선/점선의 recall을 떨어뜨릴 수 있다.

연속 지지 행 비율, 최대 내부 gap, 최종 inlier 비율/RMS, 좌우 폭과 폭 변동,
접선 차이, 여러 유사 후보 쌍의 모호성, 시간상 위치 급변을 검사한다.
이는 도색 의미 분류기가 아니므로 화살표/횡단보도 오검출을 완전히 보장하지 않는다.
급곡선·점선·가림과 차선 변경에서도 미검출 가능성이 있다.

## metric 정의와 활성화 조건

좌표: BEV x는 오른쪽, y는 아래쪽. metric X는 왼쪽, Y는 전방.
중심 곡선 x=f(y), 보정 배율 sx/sy, 차량 기준점 (xv,yv), 평가 위치 yl에서:

```
e_y       = (xv - f(yl)) * sx
e_heading = atan(f'(yl) * sx / sy)
curvature = -f''(yl) * sx / sy² / (1 + (f'(yl)*sx/sy)²)^(3/2)
lookahead = (yv - yl) * sy
```

기존 controller의 +left 의미를 유지하며 e_y는 기존처럼 lookahead에서 평가한다.
차량 기준점은 카메라 중심으로 임의 대체하지 않는다. geometry_calibrated=true,
source/destination, x/y 미터 배율, 차량 기준점이 모두 있어야 metric_ready가 된다.
추가로 연속 프레임 확인 후에만 detected=true가 된다. 기본 확인은 3프레임이다.
보정 전에는 visual_detected/visual_confidence/pixel_error만 debug JSON에 제공하고
Lane 메시지는 detected=false, metric/confidence=0으로 유지한다.
confidence는 확률이 아니며 현 controller의 threshold에 맞게 억지로 올리지 않았다.

## 검증과 남은 한계

휴대폰 영상은 BEV 기하 추정에 사용하지 않았다. 기존 하단 ROI를 직사각형으로
재배치한 **미보정 pixel-space negative fixture**로만 시험했다. 결과 경로:
`~/d435i_vision_reviews/bev_phone_regression_v2/summary.json`.

- 36개 표본: 기존 visual center 14, 새 visual center 0. 검출 정확도/recall 수치가 아니다.
- 핵심 f603/f643 화살표와 f884 횡단보도는 중심 생성 없음.
- f840의 정지선 연결 차선은 단일 곡선 후보 1개 보존.
- 초기 구현의 f780 노면 잡음 후보는 최소 도색폭 검사 후 제거됨.
- 흰색+노란색 모드로 검사했으므로 흰색을 단순 제외해서 통과시킨 시험은 아니다.
- 정상 실영상 양쪽 차선 positive 검증은 아직 부족하다. 합성 양쪽 곡선,
  연결 정지선, outlier, 폭 불일치, metric 수식/부호, temporal loss/jump,
  CameraInfo/해상도 불일치, 미보정 fail-closed는 unit test로 검증했다.

### 재현 명령

```bash
cd ~/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src/fma_perception:$PYTHONPATH"
python3 -m pytest src/fma_perception/test -q
python3 src/fma_perception/tools/regress_bev_phone.py --output /tmp/new_phone_regression
colcon build --packages-select fma_perception --symlink-install
ros2 launch fma_perception d435i_bev.launch.py --show-args
```

실행용 launch는 보정값이 비어 있으므로 현재 기본 설정으로는 제어용 검출을 내지 않는다.
실제 차량/STM32 명령, PD/PID/controller 변경은 이번 작업에 포함하지 않았다.

## D435i 데이터와 실제 차량 시험 전 확인할 사항

640×480 실제 CameraInfo, 장착 자세, 왜곡 보정 후 지면 대응점, BEV 목적 좌표와
실제 x/y 축 거리, 차량 제어 기준점 위치를 측정해야 한다. normalized points도
다른 해상도/화각으로 자동 호환된다고 가정하지 않으며 입력 크기 불일치를 거부한다.
ROI/HSV/window 폭/도색폭/fit·gap·폭 sanity/EMA/lookahead는 여러 직선·곡선·그늘·
횡단보도·방지턱·가림 장면에 걸쳐 조정해야 한다. phone 결과로 최종 고정하지 않는다.
한쪽 차선 추정은 아직 비활성이다. 실제 30Hz 처리 지연과 callback backlog도
장치 데이터에서 측정해야 하며 이번 작업은 30FPS 실시간 달성을 주장하지 않는다.
구동 전 기록 재생으로 metric 부호·단위·조향 응답 방향, confidence/손실 시
기존 controller 동작과 타임아웃을 확인해야 한다.

최종 검증: perception 테스트 **105 passed**, fma_perception colcon build 성공,
launch --show-args 정상, git diff --check 및 신규 파일 whitespace 검사 통과.
