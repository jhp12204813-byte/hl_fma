# 다중 높이 ROI 추적

ROI 상단 .35, 전체 가로폭의 시각화 후보 설정을 추가했다.
config/d435i_roi_multilevel_preview.yaml은 실제 지면 보정이 아니라 직사각형
재배치이며 geometry_calibrated=false, metric 배율/차량 기준점은 미설정이다.
운영 기본 d435i_bev.yaml과 Hough 기본 경로는 변경하지 않았다.

bev_multi_height_seeds=true이면 매 window 높이에서 histogram seed를 찾고
각 seed부터 위쪽으로 추적한다. 기존 bottom-only도 선택 가능하다.
관측 높이 15% 이상 부분 곡선을 허용하되 partial 표시하고 중심/metric
출력에는 사용하지 않는다. 부분 후보는 실제 관측 y_min..y_max 범위만
원영상과 BEV에 그린다. 겹치는 관측 구간에서 중복 후보를 제거한다.

동일한 전체폭 ROI/HSV/window 설정에서 bottom-only와 multilevel을 비교했다.
D435i 12개 × 8표본=96장: 곡선 후보 발생 0→31장, 10개 세션에서 후보 복구.
양쪽 중심 생성은 둘 다 0이다. 정확도 향상 수치가 아니며 정답 라벨/시간 추적
검증은 아직 없다. 짧은 잡음/보도/도로 표시가 부분 곡선에 들어올 수 있다.
휴대폰 36표본의 core f603/f643/f884 중심 생성 없음, f840 단일 후보 보존.
f884에서도 부분 후보는 2개 남으므로 횡단보도 후보 자체를 제거했다고 주장하지 않는다.

결과: ~/d435i_vision_reviews/multilevel_roi_v2/index.html
휴대폰: ~/d435i_vision_reviews/multilevel_phone_regression/summary.json

재현:
```
export PYTHONNOUSERSITE=1
export PYTHONPATH=~/fma_autonomous_vehicle/ros2_ws/src/fma_perception:$PYTHONPATH
python3 tools/tune_d435i_offline.py --multilevel --output /tmp/new_multilevel_review
python3 tools/regress_bev_phone.py --multilevel --output /tmp/new_phone_multilevel
```

perception 테스트 107개 통과, colcon build 성공, git diff --check 통과.
차량 제어/STM32/기록 원본 변경과 git add/commit/push 없음.
