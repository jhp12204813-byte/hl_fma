# 방향 기반 window 추적 실험

`bev_directional_tracking=true` 선택 시 최근 두 window의 관측 행 중심에서
국소 1차 방향을 추정한다. 다음 window는 고정 x 중심 대신 각 y의 예측 x
주변(margin의 40%)을 탐색한다. 최종 2차 fitting, residual/coverage 검사,
partial 후보의 중심/metric 배제는 유지한다. 기본값 False이며 기존 설정은 그대로다.

## 동일 설정 비교

D435i 12개 세션, 각 8표본, 총96표본에서 전체폭 ROI/HSV 등은 고정했다.
후보 발생 프레임 31→33, 중심 생성 0→0. 이 수치는 정확도가 아니다.
183820 세션 1→5, 184328 3→4로 늘었으나 183629 5→4,
185112 4→2, 185416 3→2로 줄었다.
184328의 f681에서 노란 왼쪽 차선 구간이 복구됨을 시각 확인했다.
f1022/f1362에서는 차선을 따라가다 배경 잡음으로 이어지는 오류가 남아 있다.
성공한 장면만으로 운영 기본값을 바꾸지 않았다.

결과: ~/d435i_vision_reviews/directional_tracking_v1/index.html
수치/설정: 같은 폴더 results.json

휴대폰 36표본 회귀: core f603/f643/f884 중심 생성 없음.
부분 곡선 후보는 배경·횡단보도에도 생기므로 오검출 완전 해결을 뜻하지 않는다.
미터 배율/차량 기준점/BEV 지면 보정값을 추정해서 채우지 않았다.

## 검증

perception 테스트 109개 통과, fma_perception build 성공, git diff --check 통과.
표본 처리 p95는 기존129.2ms / 방향추적135.5ms(매 표본 detector·remap 초기화 포함).
실시간 steady-state 성능 측정이 아니며 30Hz 달성을 주장하지 않는다.

재현:
```
export PYTHONNOUSERSITE=1
export PYTHONPATH=~/fma_autonomous_vehicle/ros2_ws/src/fma_perception:$PYTHONPATH
python3 tools/tune_d435i_offline.py --directional --output /tmp/new_directional
python3 tools/regress_bev_phone.py --multilevel --directional --output /tmp/new_directional_phone
```

다음 단계는 실제 지면 BEV 검증 및 연속 관측의 도색폭/방향 일관성 강화다.
단순 min_support 완화나 결과 합집합은 잡음도 늘리므로 적용하지 않았다.
차량 제어/firmware 수정, STM32 전송, git add/commit/push 없음.
