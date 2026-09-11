# 신호 인식 팀 인계

저장소 루트의 `autonomous_vision/`이 현재 사용하는 Ultralytics 기반 구현입니다.
DEBUG_REPORT의 앞부분은 이전 Keras 실험 기록입니다. 그 안의 로컬 산출물 경로는
이 브랜치에 전부 포함된 파일 목록을 뜻하지 않습니다.

## 실행

README의 설치 안내에 따라 환경을 준비한 후 저장소 루트에서 실행합니다.
모델 두 개는 이 브랜치에 포함되어 있습니다. 처음 실행할 때 별도 학습은 필요 없습니다.

```bash
python autonomous_vision/main.py --source path/to/video.mp4 \
  --detector-weights autonomous_vision/models/detector_best.pt \
  --classifier-weights autonomous_vision/models/sign_classifier_best.pt \
  --device 0
```

카메라는 `--source 0`, CPU는 `--device cpu`로 지정합니다.
현재 차로를 확실히 알 때만 `--current-lane LEFT`, `CENTER`, `RIGHT`를 지정합니다.
기본값 UNKNOWN이면 차로변경 목표는 결정하지 않습니다. Jetson 설치는 JetPack에
맞는 PyTorch 환경이 필요하며 개발 PC용 wheel 명령을 그대로 적용하지 않습니다.

## 차선 인식과 연결할 위치

현재 `main.py`의 `run()`은 카메라를 직접 읽는 단독 실행 루프입니다.
통합 시 차선 인식 쪽에서 받은 동일한 OpenCV BGR 프레임을 그 루프의 검출 및
분류 단계에 전달하도록 구성하세요. 모델·추적기·상태 필터는 시작할 때 한 번 만들고
프레임 간 유지합니다. `draw_*` 호출 전에 인식 및 차선 계산을 끝내야 화면에 그린
글자와 박스가 다음 알고리즘 입력에 섞이지 않습니다.

| 결과 객체 | 사용할 값 | 의미 |
|---|---|---|
| `traffic` | `stable_state` | RED/YELLOW/GREEN/GREEN_LEFT/GREEN_AND_LEFT/UNKNOWN |
| `slot_states` | 각 항목의 `slot`, `lane`, `raw_state`, `stable_state`, `observed` | 상단 패널별 현재/안정화 판정 |
| `decision` | `board_valid`, `lane_change_required`, `target_lane`, `reason` | 현재 차로와 패널 상태로 계산한 인식 판단 |
| `message` | `detected`, `active` | 하단 전광판 검출 및 켜짐 여부; 문구 OCR 없음 |

슬롯 LEFT/CENTER/RIGHT는 화면상 좌→우 패널 대응이며 실제 주행 차로 번호를
자동 추정한 결과가 아닙니다. 차선 인식/미션 담당 쪽에서 대응을 검증해야 합니다.
UNKNOWN이나 검출 소실을 통행 허가로 간주하지 마세요.
현재 ROS2 토픽 발행과 조향/제동 연결은 구현되어 있지 않습니다.

## 장애물 후보 모델

신호 detector와 별개인 2-class YOLO11n 후보 모델도 포함합니다.

```text
models/obstacle_detector_candidate.pt
0 child_dummy
1 vehicle_obstacle
SHA256 77de8602865021736366e658f8c9dfd07a57e39ea859c34b7d6e3a077bd4169c
```

두 클래스는 제어에서 모두 `OBSTACLE_AVOIDANCE` 후보로 취급합니다. 클래스가 서로
바뀌어도 공통 장애물 검출에는 성공한 것으로 판단합니다. FN을 줄이기 위해 단일
confidence 0.25로 제거하지 말고 0.01~0.03의 저신뢰 후보를 유지한 뒤, 정렬된 D435i
Depth, 주행 corridor, 프레임 연속성으로 배경 FP를 제거하는 구조가 권장됩니다.
confidence 0.01 자체를 곧바로 회피 명령으로 연결하면 안 됩니다.

현재 clean val 28장 기준 공통 장애물 recall은 confidence 0.01에서 0.964
(TP 27/FP 762/FN 1), 0.03에서 0.750(TP 21/FP 115/FN 7)입니다. 데이터가 작고
vehicle val bbox가 2개뿐이므로 실차 배포 모델이 아니라 **통합 시험 후보**입니다.
상세 학습/비교 결과는 `OBSTACLE_MODEL_REPORT.md`를 확인하세요.

학습 데이터, 라벨, 영상, run 전체는 용량과 촬영 데이터 보호를 위해 Git에 넣지
않았습니다. `train_obstacles.py`, `evaluate_obstacles.py`,
`evaluate_avoidance_priority.py`가 재학습 및 FN 우선 평가 코드입니다. 기존
`main.py`는 신호용 3-class weight를 사용하므로 이 2-class weight로 바꾸지 마세요.
장애물 detector는 별도 인스턴스로 통합해야 합니다. D435i Depth와 ROS2 제어 연결은
아직 구현하지 않았습니다.

## 제공 모델과 검증 범위

- detector: YOLO11n, traffic_light/sign_panel/message_board, 50 epoch 학습.
- classifier: YOLO11n-cls, GREEN_ARROW/RED_X/LANE_CHANGE, 기본 confidence 0.70.
- detector validation 22장: mAP50 0.725. 별도 교차로 test 24장에서는
  640 입력 mAP50 0.0666, 1280 입력 0.622로 작은 신호등에서 성능 차이가 큽니다.
- classifier test 307장: top-1 약 0.857. 꺼진 패널은 별도 학습 클래스가 없어서
  낮은 confidence 처리만으로 항상 UNKNOWN이 보장되지는 않습니다.
- 새 test에서 임계값을 바꾸며 진단했으므로 최종 설정 평가에는 추가 미사용 세션이 필요합니다.
- 실차 제어 준비 완료 모델이 아닙니다. 작은 신호등 누락, 패널/전광판 오검출,
  프레임 건너뛰기와 박스 평활화에 따른 상태 지연을 실제 주행 환경에서 검증해야 합니다.

촬영 데이터·라벨·결과 영상과 과거 Keras 체크포인트는 로컬에 보존되어 있습니다.
재학습 재현에는 해당 데이터를 별도로 공유받아야 합니다.
