# Depth ground BEV 추적 — 오프라인 구현, 미승인 프로파일

## 현재 상태

원본 640×480을 crop하지 않고 y<168(상단35%) 후보를 마스킹한다. CameraInfo K/D와 RGB/depth timestamp는 변경하지 않는다. 기존 ROS lane node / Hough 기본값 / control interface는 유지했다. 새 경로는 `ground_tracker.py`와 `tools/replay_ground_tracker.py`의 오프라인 검증 경로다. 기존 임의 perspective BEV 결과를 새 지면 BEV 성능으로 간주하지 않는다.

## 처리 순서와 변경 이유

1. 매 프레임 aligned uint16 depth와 CameraInfo로 deproject. 전체 가로폭의 하단55%부터 RANSAC/SVD로 평면 추정. 이 경로에서는 optical Z 4m 이내 depth만 사용한다. 원래 plane API의 기본8m는 기존 도구 호환성을 위해 유지한다.
2. 색상 후보 상단35% 마스크. yellow HSV `(5,25,60)..(40,255,255)`, white `(0,0,170)..(179,65,255)`는 기존 값 유지. 흰색은 도로 표시 진단이며 노란 차선 tracker에는 넣지 않는다.
3. 유효 depth의 point-to-plane residual이 8cm를 넘으면 제외. depth가 없으면 ray-plane 후보를 유지하되 `unknown_depth_bev_pixels`로 기록한다. 이 값은 지면 검증 완료 픽셀이 아니다. `ground_rejected_pixels`는 ROI 내 전체 불일치 픽셀 수이며 차선 오검출률이 아니다.
4. ground cell → 원영상으로 역투영하여 nearest-neighbor remap. 왜곡은 CameraInfo D로 반영하며 별도 cropped K를 만들지 않는다. dilation/closing 없이 sampling hole을 줄인다. 실제 차선 단절, 원영상 해상도 한계, 시야 밖 영역을 복원하는 기능은 아니다.
5. 새 metric raster를 기존 `sliding_curves` / robust quadratic fit에 직접 연결한다. 원영상의 임의 사다리꼴 transform은 사용하지 않는다. 하단 차선이 시야 밖일 수 있어 여러 높이에서 seed를 탐색하되 최소 support는 55%로 유지한다. 부분 곡선을 억지로 좌우 쌍으로 승격하지 않는다.
6. run width, 길이/연속성, RANSAC inlier/residual, 좌우 폭/폭 변화/접선, heading/curvature 관계로 검사. 중심점 midpoint만으로 좌우를 분류하지 않는다. 이전 중심 이동/heading/curvature 점프를 거부하고 통과한 중심에만 EMA를 적용한다. 300ms 이상 간격/역순 timestamp/소실 시 상태를 초기화한다.
7. confidence는 fit score × ground inlier ratio × RMS 감점. 정확도 확률로 calibration된 값이 아니다. 평면 잔차가 작아도 노란 보도 경계를 진짜 차선으로 보증하지 않는다.

## 검증용 설정 — 실측 calibration이 아님

`GroundConfig` 및 replay `--config JSON`으로 설정한다. 실행 디렉터리 `config.json`에 저장된다.

- raster: 좌우±3m, 전방0.7..4m,1cm/cell. 표시/탐색 범위이며 실측 lane width가 아니다.
- 11 windows, margin30cm, marking width2.5..20cm, fit residual6cm, 최소support55%, 최대gap15%.
- pair width 허용범위0.8..4.5m, 폭 변화20%, heading 차이0.2rad, curvature 차이0.3/m. 모두 **검증되지 않은 rejection envelope**이며 누락된 차선을 추정하는 실측값으로 사용하지 않는다.
- temporal center20cm, heading0.15rad, curvature0.2/m. 차량 장착 데이터의 정답 annotation으로 다시 검증해야 한다.

`camera_ground_offset_m`, `camera_ground_heading_rad`, `camera_ground_curvature_inv_m`는 카메라 지면 좌표 진단이다. 차량 rear axle extrinsics가 없으므로 차량 기준 e_y라고 부르지 않는다. +횡방향/heading은 좌측 convention. `detected`, `steering_valid`, `metric_control_enabled`는 항상 false이며 실제 명령을 publish하지 않는다. 한쪽 차선만으로 임의 폭을 보충하지 않는다.

## 재현

```bash
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/ros2_ws/src/fma_perception:$PYTHONPATH"
/usr/bin/python3 ros2_ws/src/fma_perception/tools/replay_ground_tracker.py \
  --root ~/d435i_recordings --output ~/d435i_vision_reviews/ground_tracker_new
```

각 영상25/50/75% 지점에서 연속10프레임씩 평가한다. 출력 MP4는 세 짧은 구간을 이어 붙인 진단 영상이며 전체 영상도 실제 시간 연속 영상도 아니다. 입력 timestamp와 depth 매칭 오차는 results.json에 보존한다. 20ms 초과 매칭은 제외/명시하고 temporal state를 초기화한다. MP4 decode frame count도 확인한다.

BEV 진단 색: 흰색=yellow candidate, 회색=white marking, 어두운 회색=시야 밖, 주황색=통과 곡선, 녹색=중심. 원영상 파란 박스=검출 ROI.

## 성능 해석 / 남은 검증

합성 직선의 전방2.5..4m 150행에서 forward scatter42행 → inverse150행으로 개선됐다. 이는 rasterization 검증이고 실제 차선 인식률이 아니다.

실제 데이터는 12개 영상360개 요청 프레임, RGB/depth 매칭354개. 지면 추정354개 모두 accepted, 좌우 pair/center 생성0개다. 현재 추적은 실패 상태다. 지면만 정상이라는 이유로 추적을 완료 처리하거나 camera 각도를 바꿔야 한다고 판단할 수 없다. RGB에서 가까운 양쪽 차선이 화각 밖으로 나가는 장면, 관측 길이가 짧은 장면, 노란 curb/방지턱이 섞인 장면을 별도로 검증해야 한다. ROI/HSV를 더 완화하지 않았다.

실제 좌/우 차선 보존률, arrow/crosswalk/curb/stop-line별 false-positive율은 정답 라벨이 없어 미측정이다. 모든 center가 거부되므로 center jitter / e_y / heading 연속성은 N/A이며 0이라고 보고하지 않는다. 전체 연속영상 검증도 아직 아니다. 좌우 실제 차선 annotation을 이용한 보존률/평행성 검증 후에만 추적 파라미터를 승인해야 한다. 차량 기준 metric 출력에는 장착 extrinsic 확인이 추가로 필요하다.

Phone regression은 기존 pixel-space 추적기의 화살표/횡단보도 fixture 보존 확인에만 사용했다. depth가 없어 새 ground pipeline의 정확도 검증이 아니다. 카메라 각도 변경, 차량 구동, STM32 명령, git add/commit/push는 수행하지 않았다.
