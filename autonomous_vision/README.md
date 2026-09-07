# Autonomous Vision

이 브랜치는 신호 인식 소스, 학습/라벨링 도구와 테스트를 제공합니다.
학습한 `detector_best.pt`와 `sign_classifier_best.pt`는
`autonomous_vision/models/`에 포함되어 바로 추론에 사용할 수 있습니다.
촬영 원본, 데이터셋과 생성된 결과 영상은 포함하지 않습니다.
팀원의 실행 및 차선 인식 연동 방법은 [TEAM_HANDOFF.md](TEAM_HANDOFF.md),
학습·평가 이력은 [DEBUG_REPORT.md](DEBUG_REPORT.md)를 참고하세요.
현재 모델은 시험 단계이며, 일반 도로의 작은 신호등과 패널 오검출에 대한
추가 검증이 필요합니다. ROS2 및 차량 제어 연결은 포함하지 않습니다.

ROS2와 차량 제어에 의존하지 않는 standalone perception 구현입니다. 프레임마다
하나의 YOLO detection 모델을 한 번만 실행합니다.

```text
0 traffic_light   가로형 4구 일반 차량 신호등 전체
1 sign_panel      신호차 상단 LED 패널 한 개(프레임당 최대 3개)
2 message_board   신호차 하단 중앙 문자 전광판
```

신호차 차체 전체는 detection class가 아닙니다. `sign_panel`은 각각 개별 bbox로
라벨링하며, 검출된 패널을 좌→우로 SLOT 1/2/3에 연결합니다.

## 구현 구조

```text
autonomous_vision/
├── main.py
├── config.py
├── yolo_detector.py
├── bbox_tracker.py
├── stability_filter.py
├── traffic_light_classifier.py
├── sign_classifier.py
├── sign_board_interpreter.py
├── message_board.py
├── prepare_sign_dataset.py
├── train_detector.py
├── train_sign_classifier.py
├── models/
│   ├── detector_best.pt
│   └── sign_classifier_best.pt
├── dataset_detection/
└── dataset_sign_cls/
```

## 처리 방식

`traffic_light` bbox는 Dynamic ROI가 됩니다. 내부의 상대좌표 Lamp ROI 네 개를
HSV, morphology, 색 픽셀 비율과 contour 면적으로 분석하여 다음 상태를 냅니다.

```text
RED YELLOW GREEN GREEN_LEFT GREEN_AND_LEFT UNKNOWN
```

`sign_panel`은 최대 3개를 각각 검출합니다. 세 개가 처음 함께 보이면 center_x로
정렬하여 SLOT 1/2/3을 확정합니다. 이후 1~2개만 검출되면 이전 bbox 중심과
association하여 번호가 뒤바뀌지 않게 합니다. 최초부터 일부 패널만 보이면 안전을
위해 슬롯을 임의 추정하지 않습니다.

세 crop은 동일한 classifier에 batch 한 번으로 입력합니다. 학습 class는 다음
세 개이고, confidence가 낮으면 코드에서 `UNKNOWN`으로 처리합니다.

```text
GREEN_ARROW
RED_X
LANE_CHANGE
```

OCR은 사용하지 않습니다. 각 슬롯은 raw/stable 상태를 따로 유지하고, 차로변경
판단에는 stable 상태만 사용합니다.

`SignBoardInterpreter`는 현재 차선에 `LANE_CHANGE`가 있고 GREEN_ARROW가 정확히
하나일 때만 `lane_change_required`, `target_lane`, `target_slot`을 출력합니다.
GREEN_ARROW가 없거나 여러 개면 임의 차선을 고르지 않고 `board_valid=False`로
처리합니다. 조향·trajectory 명령은 생성하지 않습니다.

`message_board`는 bbox 내부 밝기, saturation, 활성 픽셀 비율로 ON/OFF까지만
판정합니다. OCR/문구 classification은 실제 문구 목록 확정 후 추가합니다.

## Detection 라벨

[data.yaml](dataset_detection/data.yaml)의 클래스 순서를 반드시 지킵니다.
YOLO 라벨 형식은 다음과 같습니다.

```text
class_id center_x center_y width height
```

좌표는 0~1 정규화 값입니다.

- `traffic_light`: 4구 신호등 전체 하우징 하나
- `sign_panel`: 상단 LED 패널 각각 하나, 따라서 한 이미지에 최대 3개
- `message_board`: 하단 중앙 큰 전광판 하나

같은 영상의 유사 프레임을 train/val/test에 섞지 말고 촬영 영상 또는 주행 세션을
통째로 분리해야 합니다. 이미지와 같은 stem의 `.txt`가 필요하며 실제 negative
이미지만 빈 label 파일을 사용합니다.

현재 선별된 전체 세션을 새 라벨 작업공간으로 복사하려면:

```bash
python autonomous_vision/prepare_detection_dataset.py
python autonomous_vision/label_detector.py --port 8766
```

브라우저에서 `http://127.0.0.1:8766`을 열고 단축키 `0/1/2`로 세 클래스를
라벨링합니다. 기존 라벨은 새 기준에서 불완전하므로 자동 복사하지 않습니다.
기존 7/9/10 패널 박스는 `draft_labels/`에서 미저장 초안으로 표시됩니다. 초안이
보여도 완료 상태가 아니며, 꺼진 상단 패널과 하단 message_board를 모두 추가한 뒤
직접 저장해야 합니다. 신호차 차체는 박스로 만들지 않습니다.

## Sign classification 데이터

```text
dataset_sign_cls/
├── train/{GREEN_ARROW,RED_X,LANE_CHANGE}
├── val/{GREEN_ARROW,RED_X,LANE_CHANGE}
└── test/{GREEN_ARROW,RED_X,LANE_CHANGE}
```

`UNKNOWN` 폴더는 만들지 않습니다. classification 데이터도 동일 영상의 인접 crop이
train/val에 섞이지 않도록 세션 단위로 분리해야 합니다.

## 설치

현재 기본 `.venv`에는 Ultralytics와 PyTorch가 없습니다. 기존 TensorFlow 환경을
바꾸지 않고 별도 환경 사용을 권장합니다.

```bash
cd ~/hl_fma
python3 -m venv .venv-yolo
source .venv-yolo/bin/activate
python -m pip install -U pip
# 이 학습 PC(RTX 3050, driver CUDA 12.8)에서는 먼저 호환 PyTorch를 설치:
python -m pip install --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.11.0+cu128 torchvision==0.26.0+cu128
python -m pip install -r autonomous_vision/requirements-yolo.txt
```

Jetson은 JetPack과 맞는 NVIDIA PyTorch를 먼저 설치해야 합니다. JetPack 버전 확인
없이 일반 데스크톱 torch wheel을 강제로 설치하면 안 됩니다.

## 학습

Detection:

```bash
python autonomous_vision/train_detector.py \
  --data autonomous_vision/dataset_detection/data.yaml \
  --model yolo11n.pt --epochs 200 --imgsz 640 --batch 4 --lr0 0.001

cp autonomous_vision/runs/detector/weights/best.pt \
   autonomous_vision/models/detector_best.pt
```

Sign classification:

```bash
python autonomous_vision/train_sign_classifier.py \
  --data autonomous_vision/dataset_sign_cls \
  --model yolo11n-cls.pt --epochs 100 --imgsz 224 --batch 16

cp autonomous_vision/runs/sign_classifier/weights/best.pt \
   autonomous_vision/models/sign_classifier_best.pt
```

학습 데이터가 비어 있으면 두 스크립트 모두 모델 다운로드 전에 오류로 종료합니다.

## 실행

Detector와 classifier를 모두 사용하는 카메라 실행:

```bash
python autonomous_vision/main.py \
  --source 0 \
  --detector-weights autonomous_vision/models/detector_best.pt \
  --classifier-weights autonomous_vision/models/sign_classifier_best.pt \
  --current-lane CENTER \
  --debug
```

영상 파일:

```bash
python autonomous_vision/main.py \
  --source path/to/test.mp4 \
  --detector-weights autonomous_vision/models/detector_best.pt \
  --classifier-weights autonomous_vision/models/sign_classifier_best.pt \
  --current-lane CENTER \
  --debug --output models/perception_debug.mp4
```

classifier가 아직 없을 때는 `--classifier-weights`를 생략하면 detection, HSV,
패널 crop, message board ON/OFF만 확인할 수 있습니다. 이 경우 모든 sign 상태와
board 해석은 안전하게 UNKNOWN/invalid가 됩니다.

상단 패널의 1순위 분류가 맞지만 confidence가 기본값 0.70보다 낮으면 UNKNOWN으로
표시됩니다. 현장 확인에 한해 `--classifier-confidence 0.60`처럼 실행 시 기준을
조절할 수 있습니다. 기준을 낮추면 UNKNOWN은 줄지만 잘못된 차로 상태를 승인할
가능성도 커지므로, 제어에 연결하기 전 독립 영상으로 다시 검증해야 합니다. 화면의
`raw=`는 현재 프레임 판정이고 `stable=`은 연속 3프레임 확인을 통과한 판정입니다.

분류 전 패널 crop 저장:

```bash
python autonomous_vision/prepare_sign_dataset.py \
  --source path/to/sign_car.mp4 \
  --detector-weights autonomous_vision/models/detector_best.pt \
  --output-dir autonomous_vision/dataset_sign_cls/unclassified \
  --save-every 10
```

저장된 이미지를 영상 세션 단위로 train/val/test에 나누고 실제 내용에 맞는 세
class 폴더로 이동합니다.

기존 mission detector의 검수된 패널 라벨(7/9/10)을 재사용하려면 기존 세션
split을 그대로 보존하는 다음 변환기를 사용합니다.

```bash
python autonomous_vision/import_legacy_sign_panels.py
```

이 변환은 classification crop만 만듭니다. 기존 신호등 라벨은 켜진 램프만 감싸고
message_board 라벨은 없으므로 새 3-class detection 데이터로 자동 변환하지 않습니다.

현재 제공된 테스트 영상:

```bash
python autonomous_vision/main.py \
  --source 'picturesandvideos/testvideo/V20260905_154831000_E01F3BE1-E311-4C6E-B72F-F84BD0E3CFF5 (1).mp4' \
  --detector-weights autonomous_vision/models/detector_best.pt --debug

python autonomous_vision/main.py \
  --source 'picturesandvideos/testvideo/V20260905_155655000_6C0E07B5-6737-4151-9285-D4FD43C88269 (1).mp4' \
  --detector-weights autonomous_vision/models/detector_best.pt \
  --classifier-weights autonomous_vision/models/sign_classifier_best.pt \
  --current-lane CENTER --debug
```

두 파일은 기존 train 추출 세션과 촬영 시각/장면이 같은 154831 및 155655
영상입니다(일부는 frame rate/인코딩만 달라 해시가 다름). 따라서 기능 동작과
화면 표시 확인에는 사용할 수 있지만 독립 test 정확도 계산에는 사용하지 않습니다.

## 현장 설정

[config.py](config.py)에서 다음을 보정합니다.

- 신호등: `TRAFFIC_RED_ROI`, `TRAFFIC_YELLOW_ROI`,
  `TRAFFIC_GREEN_LEFT_ROI`, `TRAFFIC_GREEN_ROI`, HSV 범위
- 검출/추적: confidence, image size, EMA alpha, 누락 허용 프레임
- 패널: classification threshold, 확인/소실 프레임
- 차선 대응: `SLOT_LANE_MAPPING`
- 전광판 ON/OFF: brightness, saturation, 활성 픽셀 비율

`--debug`는 신호등 crop과 mask, 각 sign_panel crop, message_board crop과 활성
mask를 표시합니다. `--device 0 --frame-skip 2 --imgsz 512`로 Jetson 부하를
낮출 수 있습니다.

## 아직 제외된 기능

- message_board OCR 또는 문구 classification
- 실제 차선변경 trajectory와 steering
- 차선/정지선, waypoint, driving state machine
- TensorRT export와 Jetson 실측 benchmark
- ROS2 Humble wrapper
