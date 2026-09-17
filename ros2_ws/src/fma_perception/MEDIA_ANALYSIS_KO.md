# 대회장 영상 pixel 검출 검증 — 2026-09-09

두 파일 모두 OpenCV VideoCapture open 및 첫 프레임 decode 성공.
검출 알고리즘/기본값은 변경하지 않았고 metric 파라미터 3개는 0.0으로 유지했다.

## MEDIA

| path | type | resolution (W×H) | FPS | metadata frames | duration |
|---|---|---|---:|---:|---:|
| /home/idp2/차선영상1.mp4 | MP4 | 1080×1920 | 30.0128179743 | 192 | 6.397267 s |
| /home/idp2/차선영상2.mp4 | MP4 | 1920×1080 | 29.9937965261 | 967 | 32.240000 s |

duration은 frame count / FPS다. 영상1 마지막 index 191의 seek/read는 실패했고
190은 성공했다. 영상2 index 966은 성공했다. 메타데이터 frame count가 모든
프레임의 디코딩 성공을 보장하지는 않는다. 원본 파일은 변경하지 않았다.

## 방법과 baseline

각 영상에 대해 처음부터 끝 직전까지 균등 25지점과 30프레임 간격의 합집합을
샘플링했다. 영상1 30개, 영상2 56개다. 시간은 index/source FPS 근사치다.
세 설정에 동일 프레임을 적용했다. 25지점씩 원본/overlay를 육안 검토했다.
라벨링된 정답 좌표가 없으므로 precision/recall 또는 정확한 miss 비율은 산출하지 않았다.

WHITE/YELLOW coverage는 ROI 면적 대비 morphology 전 HSV mask 비율이다.
최종 mask는 morphology 이후 결합 비율이다. 픽셀 중심은 visual=true인 경우만
집계했으며 false positive도 포함한다. `detected=false`는 미보정 정책에 의한
정상 동작이며 여기서 시각적 miss 지표로 사용하지 않았다.

| baseline 지표 | 영상1 | 영상2 |
|---|---:|---:|
| WHITE 평균 (최소~최대) | 16.376% (5.209~24.376) | 42.792% (11.371~52.565) |
| YELLOW 평균 (최소~최대) | 0.018% (0.00048~0.0535) | 0.659% (0~12.020) |
| 최종 결합 mask 평균 | 5.732% | 26.045% |
| left fit 있음 | 17/30 | 28/56 |
| right fit 있음 | 27/30 | 12/56 |
| visual=true, 중심 생성 | 12/30 | 8/56 |
| image center x | 540 px | 960 px |
| lookahead y | 1248 px | 702 px |
| 생성된 lane center x 범위 | 319.8~578.0 px | 600.9~967.7 px |
| pixel error (image−lane) 범위 | −38.0~+220.2 px | −7.7~+359.1 px |
| offline 처리 FPS | 17.8 | 11.5 |
| 처리시간 중앙값 / p95 | 56.3 / 72.3 ms | 88.9 / 115.6 ms |

속도는 노트북에서 원본 해상도 `detect_lane` 한 번을 재는 단일 실행 측정이다.
디코딩, ROS 전달, 저장, 내부 값 수집용 두 번째 호출은 제외한다. 하드 실시간
보장이나 ROS topic 수신률이 아니며 시스템 부하에 따라 달라진다.

## False positives / misses / 조명

- 영상1: 진행 노란 차선이 화면 위쪽에 있고 기본 ROI y=1152..1919 밖에 있다.
  f55(1.83 s), f102(3.40 s), f158(5.26 s) 등의 overlay는 차선 없는 하단
  아스팔트/그림자 부근에 좌우 fit과 중심을 만든다. 중심 생성 12/30을 성공률로
  해석하면 안 된다. 보이는 상단 차선은 놓친다.
- 영상2 초반: ROI y=648..1079는 진행 차선의 짧은 끝부분과 밝은 아스팔트가
  대부분이다. 실제 차선의 얕은 기울기, ROI 밖 위치, fit의 세로 지지 범위 조건
  때문에 경계가 빠진다. f201(6.70 s)의 중심은 아스팔트 위 가짜 fit이다.
- 영상2 f603(20.10 s), f643(21.44 s), f723(24.10 s): 오른쪽 fit이 실제 오른쪽
  경계 대신 좌회전 화살표 도색 가장자리를 따른다. 예를 들어 f603의 center는
  828.2 px로 산출되지만 올바른 좌우 차선 중심으로 볼 수 없다.
- 영상2 f884(29.47 s): 횡단보도 흰색 블록 측면으로 좌우 fit/center=967.7 px를
  만든다. 정지선의 긴 수평 가장자리는 slope 조건이 제거하지만, 횡단보도 블록의
  세로/대각 측면과 화살표는 통과한다. 횡방향 도색 전체를 의미적으로 배제하는
  기능은 현재 없다. 정지선 자체를 오검출한 별도 사례는 이 표본에서 확정하지 않았다.
- 영상1의 마름모 도로 문양은 기본 ROI 밖이며 A에서는 ROI 안으로 들어온다.
  A의 증가한 fit은 대부분 도로 텍스처 위에 있어 개선으로 인정하지 않았다.
- HSV 기준상 낮은 S, 높은 V의 밝은 아스팔트도 WHITE가 된다. 희미한 노란 차선은
  S>=75를 만족하지 않아 WHITE로 잡힐 수 있다. WHITE/YELLOW는 의미 라벨이 아니다.

첫 프레임에서 육안으로 고른 도로 패치의 HSV p10/median/p90:

| 영상/패치 (정규화 x1,y1,x2,y2) | H | S | V | WHITE 비율 |
|---|---|---|---|---:|
| 1 햇빛 (.65,.65,.85,.80) | 7/20/130 | 8/17/34 | 92/138/203 | 27.68% |
| 1 그늘 (.03,.65,.15,.80) | 115/120/120 | 21/34/57 | 36/60/96 | 0% |
| 2 햇빛 (.65,.65,.75,.75) | 20/20/20 | 7/9/10 | 145/174/203 | 57.48% |

`white_v_min=200`이면 위 햇빛 도로 오염이 각각 11.24%, 13.98%로 줄어든다.
다만 그늘 속 도색에 대한 충분한 표본은 없어서 그늘 차선 보존을 검증할 수 없다.
낮은 채도의 H는 불안정하므로 도로 H 통계만으로 WHITE H 범위를 좁히지 않는다.

## Runtime 후보 비교 — A/B 모두 채택 보류

| 실제 parameter 이름 | 현재 default | A | B | 실험 이유 |
|---|---:|---:|---:|---|
| roi_top_ratio | 0.6 | 0.25 | 0.25 | 위쪽 진행 차선 포함 |
| roi_bottom_ratio | 1.0 | 0.6 | 0.6 | 하단 아스팔트/그림자 배제 |
| lookahead_ratio | 0.65 | 0.4 | 0.4 | 변경 ROI 내부 참조 행 |
| min_abs_slope | 1.0 | 0.3 | 0.3 | 원근에 따른 얕은 대각 차선 허용 |
| white_v_min | 170 | 170 | 200 | 밝은 아스팔트 mask 감소 |

| 설정 | 영상1 L/R/중심 생성 | 영상2 L/R/중심 생성 | WHITE 평균 영상1/2 | 처리 FPS 영상1/2 |
|---|---|---|---|---|
| baseline | 17/27/12 (30개) | 28/12/8 (56개) | 16.38 / 42.79% | 17.8 / 11.5 |
| A | 27/24/21 | 8/18/0 | 35.69 / 44.22% | 15.6 / 14.4 |
| B | 2/0/0 | 10/8/1 | 9.32 / 14.15% | 28.3 / 23.2 |

A/B의 YELLOW 평균은 영상1 2.352%, 영상2 5.381%다. ROI가 달라 coverage를
같은 공간의 개선량으로 직접 비교할 수 없다. A는 영상1 fit 수가 늘어도 텍스처
오검출이 남고 영상2에서 중심을 만들지 못한다. B는 아스팔트 mask를 줄이지만
진행 차선의 안정적 좌우 검출은 복구하지 못한다. 영상2 B의 유일한 중심
f844(28.14 s, x=958.3)는 횡단보도 부근 가짜 경계다. 둘 다 공통 개선안으로 추천하지 않는다.
참조 y가 바뀌므로 baseline과 후보의 pixel error 차이도 정확도 개선량이 아니다.

다른 검토 파라미터는 유지한다:

| 이름 | 현재값/후보값 | 유지 이유 |
|---|---|---|
| roi_left_ratio / roi_right_ratio | 0.0 / 1.0 | 공통 수평 crop의 근거 부족 |
| white_h_min/max, white_s_min/max, white_v_max | 0/179, 0/65, 255 | 밝기 외 변경 근거 부족 |
| yellow_h_min/max, yellow_s_min/max, yellow_v_min/max | 15/40, 75/255, 90/255 | 실제 도색 색상 라벨/그늘 표본 부족 |
| morph_kernel_size | 3 | 큰 도색/연결된 노이즈는 opening만으로 분리 불가 |
| min_candidate_area_px | 0 | 큰 화살표/횡단보도는 면적 필터로 제거하기 어려움 |
| min_line_length_px | 0 (자동) | 큰 도색도 긴 선분을 생성; 두 해상도에 공통 고정값 근거 부족 |
| max_line_gap_px | 0 (자동) | 증가 시 다른 도색 연결 가능; 개선 근거 부족 |
| lane_width_m / meters_per_pixel_y / single_lane_width_px | 모두 0.0 | metric calibration 미실시 |

baseline 자동 Hough min length는 영상1 153px, 영상2 86px이며 gap은 48px/27px다.
최소 slope만 높이면 횡단보도 세로 측면은 남고 실제 대각 차선을 더 잃을 수 있다.
현 시점 추천은 default를 보존하고 A/B는 실패 사례 재현용으로만 사용하는 것이다.

후보 A 재현 (기본값으로 시작한 lane_detector에 순서대로 실행):

```bash
ros2 param set /lane_detector roi_top_ratio 0.25
ros2 param set /lane_detector lookahead_ratio 0.4
ros2 param set /lane_detector roi_bottom_ratio 0.6
ros2 param set /lane_detector min_abs_slope 0.3
```

후보 B는 A 적용 후:

```bash
ros2 param set /lane_detector white_v_min 200
```

baseline 복귀 (ROI/lookahead 유효성 검사를 만족하는 순서):

```bash
ros2 param set /lane_detector roi_bottom_ratio 1.0
ros2 param set /lane_detector lookahead_ratio 0.65
ros2 param set /lane_detector roi_top_ratio 0.6
ros2 param set /lane_detector min_abs_slope 1.0
ros2 param set /lane_detector white_v_min 170
ros2 param dump /lane_detector
```

## REPLAY / 검증

`lane_media_replay`: `/front/color/image_raw`, Image/bgr8, loop=true,
원본 FPS(fallback 30), 원본 해상도, 현재 ROS stamp, frame_id=front_camera.
MP4/AVI/MOV/MKV 및 JPG/JPEG/PNG 경로 지원. 실제 codec 지원은 OpenCV backend에 따른다.

- 관련 pytest 77개 통과: 기존 lane/snapshot 회귀, 실제 AVI codec replay/loop/EOF,
  이미지 반복, 잘못된 경로/파일/FPS, FPS fallback, publish stamp/해상도/encoding,
  파일 hash 불변 확인. ROS publisher는 단위 테스트에서 mock 사용.
- 별도 실제 ROS smoke: 두 원본 MP4 각각 Image 3개를 구독하여 해상도,
  bgr8, frame_id 및 stamp 갱신 확인. 차량 제어 노드는 실행하지 않았다.
- `colcon build --packages-select fma_perception` 성공.
- `git diff --check` 통과. add/commit/push 미실행.
- 원래 있던 control/interfaces/firmware 등의 미커밋 변경은 이번 작업의 변경이 아니다.

분석 산출물은 `/tmp/lane_media_analysis/results.json`,
`baseline_v1_sheet.jpg`, `baseline_v2_sheet.jpg`, A/B sheet 및 개별 mask PNG다.
원본 해상도로 검출하고 review preview만 축소했다. `/tmp`는 임시 저장소이며
재생/분석 실행 명령은 README_KO.md에 있다.

## 후속 단계별 추적

f603/f643/f884의 binary mask, 연결 성분, Hough 선분 및 endpoint residual 추적은
[LANE_FALSE_POSITIVE_ANALYSIS_KO.md](LANE_FALSE_POSITIVE_ANALYSIS_KO.md)에 기록했다.
후속 작업도 offline 분석만 수행했으며 default와 Candidate A/B 채택 상태는 그대로다.
