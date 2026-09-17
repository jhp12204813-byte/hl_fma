# 저장 depth 기반 도로 평면 및 top-down 검증

임의 사다리꼴/직사각형 BEV의 추가 튜닝을 중단했다. 기존 추적 threshold를
변경하지 않고 별도 offline 지면 모듈을 작성했다. 실제 차량 제어/STM32 연결 없음.

## 방법

- 각 session.json의 color/depth CameraInfo K, distortion, 동일 optical frame을 확인한다.
- RGB timestamp와 depth timestamp 최근접 20ms 이내 매칭. 미매칭은 명시적으로 제외한다.
- 하단 y>=.55h, x=.12w.. .88w 영역에서 4px 간격으로 uint16 depth를 추출한다.
- scale은 depth_frames.jsonl 값(.001m)을 사용한다. undistortPoints로 만든 ray에
  optical Z depth를 곱해 3D 점을 구한다. RGB 자체를 다시 undistort하지 않아 이중 보정하지 않는다.
- 400회 RANSAC, 25mm 거리 한계, optical-down 방향 normal 제약으로 수직 벽/차량을
  제외하고 SVD로 평면을 재적합한다. 지지율>=.5와 좌/중/우 공간 지지를 검사한다.
- plane n·P+d=0에 각 lane pixel ray r를 대입해 P=(-d/(n·r))r로 교차한다.
  개별 lane depth는 이 계산에 전혀 필요하지 않다. 평행/뒤쪽/과도한 거리 교점은 제외한다.
- 원점은 카메라의 지면 수직 투영점, forward는 optical Z의 지면 투영,
  right는 normal×forward이다. 차량 rear axle/base_link 좌표가 아니다.
- 표시 범위 좌우±3m, 전방0..6m, 원본 top-down grid 1cm/pixel이다.
  이 배율은 임의 camera calibration이 아니라 SDK depth 단위에서 얻은 metric 시각화 배율이다.
- yellow HSV(5..40,25..255,60..255), white(0..179,0..65,170..255)는 기존 값을 유지했다.
  binary mask 직접 forward splat라 먼 거리에서 행 사이 빈칸이 생긴다. 실제 frame drop이 아니다.

## 결과

12개 세션에서 6개씩 총72표본. 68개 timestamp 매칭 및 평면 수용, 4개는
33ms가량 차이로 제외했다. 모든 표본이 성공했다고 표시하지 않았다.

- 카메라–추정 평면 거리: 중앙 .4579m, 전체 .3912.. .5388m.
- inlier 비율: 중앙 .9777, 최소 .7644.
- 평면 RMS: 중앙 .00550m, 최대 .01204m.
- inlier 절대오차 p95: 중앙 .01232m.
- 평면 지원 depth Z의 5..95백분위 범위 중앙값은 약1.16..5.56m.

이 잔차는 fit에 쓴 depth 기준이므로 독립적인 지면 정확도 측정이 아니다.
높이 변화는 경사·충격·지면 모델 편향 등일 수 있으며 장착 높이 실측으로 확정하지 않는다.
낮은 curb/방지턱/비평면 구간은 단일 평면 모델에 혼입될 수 있다.
RANSAC이 장애물을 제외하더라도, 그 물체의 HSV pixel을 ray-plane 투영하면
지면 위 가짜 도색이 될 수 있다. 투영 성공은 차선 의미 판별 성공이 아니다.

## BEV 방향 검증

직선에 가까운 184328 f954: 2..5m 구간의 좌우 노란 도색 행 중앙값을
직선 적합했을 때 세로축 대비 -0.41° / +4.23°, 방향 차이4.64°,
각 RMS .0106/.0153m. 184157 f548에서는 -3.96°/+3.15°, 차이7.10°.
183215 f0은 차이10.59°. 방지턱 노란 도색을 포함한 장면은44.7..69.8° 차이로
실패한다. 도색의 의미 라벨 없이 좌/우 X로 분리한 진단이며 실제 lane pair
정답 검증은 아니다. 비스듬히 주행하면 진짜 직선들도 화면 세로에 정확히
평행할 필요가 없고, 서로의 평행성과 잔차를 함께 봐야 한다.

초기 지면 기반 투영은 확인됐으나 전체 장면의 BEV 검증 완료로 선언하지 않는다.
따라서 Sliding Window/polyfit/HSV 완화는 하지 않았다. 기존 metric controller
출력도 활성화하지 않았다. 정답 도로 구간 지정, depth 오차와 지면 경사·곡률,
장애물 HSV 투영 제외, inverse rasterization 및 ground support 범위 검증이 남았다.

## 산출물과 재현

~/d435i_vision_reviews/ground_depth_v2/index.html : 12개 세션 결과
results.json : plane/잔차/매칭/공간 지지와 지원 거리
parallel_diagnostic.json : 좌우 도색 방향 비교
 temporal.json : 세 세션의 연속30 depth frames별 평면/normal/높이 변동

```
export PYTHONNOUSERSITE=1
export PYTHONPATH=~/fma_autonomous_vehicle/ros2_ws/src/fma_perception:$PYTHONPATH
OPENBLAS_NUM_THREADS=1 python3 tools/analyze_ground_depth.py --output /tmp/new_ground_review
```

코드: ground_plane.py, tools/analyze_ground_depth.py, tools/check_ground_parallel.py,
test/test_ground_plane.py. 합성 outlier/결측 depth, ray-plane 수식, 빈 depth 거부를
검증했다. 신규 실패 검출 후 빈 ray 처리를 수정하고 전체 테스트를 재실행했다.
