# Mission detector 진단 기록 — 2026-09-07

## 현재 판정

기존 `mission_detector_20ep_best.keras`는 배포 불가다. 낮아진 loss는 실제 검출
성공을 의미하지 않았다. 클래스 이름 재매핑이나 NMS 표시 버그는 재현되지 않았고,
작은 GT의 양성 타깃 배정이 끊기는 경우는 원래 모델에서 직접 재현했다.
아래 실험은 **본학습 재개가 아니며** 원본 모델/데이터/분할을 보존한다.

## 원본 모델의 실제 검출

[전체 검증 결과](../../models/debug/validation_20ep.json): val 144장, GT 135개.
IoU 0.5, class-aware 일대일 매칭이며 COCO mAP가 아니다.

| confidence | TP | FP | FN | recall |
|---|---:|---:|---:|---:|
| 0.4 | 0 | 45 | 135 | 0 |
| 0.05 | 0 | 75 | 135 | 0 |

yellow는 val GT 자체가 없어 recall 평가 불가다. train의 yellow 이미지는 암기
진단에만 사용했고 val로 옮기지 않았다.

전체 split을 다시 확인한 결과 yellow는 train 14장/25 boxes, val 0, test 0이다.
따라서 새 독립 촬영 세션의 class 3 annotation을 val/test에 각각 확보하기 전에는
yellow 일반화 recall을 보고하지 않는다. 기존 daySequence1/2를 나누지 않는다.

위 내용은 원본 20 epoch 모델을 진단한 당시의 데이터 상태다. 이후 2026-09-07에
대회장 직접 촬영 yellow 7장/7 boxes를 하나의 독립 val 세션으로 추가했다. 사용자가
제공한 인터넷 참고 이미지도 중복 1장을 제외하고 test 16장/29 boxes로 격리했다.
현재 수량은 train 14장/25, val 7장/7, test 16장/29다. 기존 모델의 과거 수치를 새
151장 validation 결과로 간주하지 않으며, 재학습 모델에서 다시 평가해야 한다.

## 코드/데이터 점검

- GT: 00620은 ID 2/11, 00900은 ID 4 세 개, 05114는 ID 3 두 개와 ID 2 두 개.
  YOLO normalized center-xywh를 원본 pixel xyxy로 변환해 시각적으로 위치를 확인했다.
- `dataset.py` → detector/label encoder/loss → decode/NMS는 pixel xyxy다.
  학습/추론은 RGB float32 0–255, 중앙 letterbox 114, backbone 내부 rescaling 1/255다.
- `classes.py`, 모델 metadata, head의 12개 출력, infer의 표시/신호등 필터는 같은 ID다.
  모델 원시 출력 자체가 잘못된 클래스/낮은 점수를 내므로 표시 단계의 문제가 아니다.
- 00900/05114 원시 클래스별 최대 점수는 모두 0.018 미만이었다. confidence 0.05도
  복구하지 못한다. 00620의 높은 ID 8/9 예측은 GT 신호등 위치가 아닌 하늘 쪽이다.
- TensorFlow와 OpenCV 전처리의 평균 픽셀 차이는 약 0.40/255였으며 TF 입력에서도
  동일한 오탐/미검출이 재현됐다. augmentation 후 GT 박스/ID도 시각 확인했다.
  좌우 반전은 없고, 실제 암기 실험에는 augmentation을 사용하지 않았다.
- 학습 snapshot의 이미지/라벨 916개 파일 해시가 현재 파일과 모두 일치했다.
  ROS2/STM32/제어 코드, source split, 클래스 순서, 패키지 버전은 변경하지 않았다.

GT/예측/증강/원시 tensor:
[baseline artifacts](../../models/debug/diagnosis_final_baseline/).

## 왜 loss가 감소했나

`model_factory.py`는 BCE/CIoU로 compile하고, 설치된 KerasCV 0.9의
`YOLOV8Detector.compute_loss()`는 GT를 label encoder에 전달한 뒤 quality-weighted
box/class loss를 계산한다. 출력과 loss가 끊긴 것은 아니다.

[원본 loss/배정 분해](../../models/debug/assignment_baseline/assignment.json):

- 세 이미지의 합계 anchor 16,128개 중 양성 quality가 있는 anchor는 17개.
- box loss 약 2.964, class loss 약 4.290, 합 약 7.254.
- class loss 중 배경 anchor 약 4.070, 양성 anchor 약 0.220.

따라서 배경 확률 억제로 loss가 크게 줄어도 GT의 올바른 class/confidence/IoU를
보장하지 않는다. `val_loss=6.36`은 정확도나 검출 성공률이 아니며 "매우 낮아서
학습 성공"이라고 판정할 근거가 없다. 초기 loss가 큰 데는 임의 초기화된 detector
head와 다수 배경 anchor의 합산도 관여한다. 전체 학습 곡선의 매 단계 기여도를
당시 저장하지 않았으므로 감소분 전부를 특정 원인으로 단정하지는 않는다.

### 확인된 작은 객체 배정 실패

512 입력에서 일부 신호등은 6–10픽셀에 불과하고 내부 anchor가 한 개뿐이다.
원본 encoder는 CIoU를 alignment에 사용한 다음 overlap > 0 조건으로 양성을
선택한다. IoU가 양수라도 CIoU가 음수일 수 있어 그 GT의 학습 신호가 사라진다.

- 00900의 가장 왼쪽 초록불: 내부 anchor 1개, assigned target score sum = 0.
- 05114의 작은 오른쪽 노란불: 내부 anchor 1개, assigned target score sum = 0.
- 기존 encoder로 300회 암기한 상태에서도 해당 후보 CIoU가 각각 약 -0.006,
  -0.003으로 배정 0이 지속됐다.

또한 alignment에 overlap**6을 사용한 후 절대 epsilon을 더해 정규화하는 방식은
매우 작은 overlap의 quality를 과도하게 줄일 수 있다. 이 수치 문제는 합성
fixture에서 재현했다. 모든 실제 오탐이 이 한 가지 문제 때문이라고 단정하지 않는다.

## 최소 변경과 비교 실험

`stable_label_encoder.py`는 프로젝트 내부의 **opt-in 실험용** encoder다.
배정 overlap만 nonnegative IoU로 바꾸고, 상대 quality를 log 공간에서 안정적으로
정규화하며 실제 배정된 anchor만 foreground로 표시한다. 박스 loss는 기존 CIoU,
분류 loss는 BCE다. 내부 anchor가 없는 GT에 임의의 양성 anchor를 만들지 않는다.
IoU를 이용하는 배정 원리는 [TOOD 원본 구현](https://raw.githubusercontent.com/fcjian/TOOD/master/mmdet/core/bbox/assigners/task_aligned_assigner.py)을 참고했으며,
이 프로젝트의 log 정규화는 별도의 실험적 보완이다.

설치된 KerasCV 코드는 수정하지 않았다. `model_factory.py`에는 새 encoder
archive 역직렬화 등록과 기본값 0인 선택적 DFL 보조 loss를 추가했다.
**기존 모델 생성/학습 기본값은 그대로**다.
encoder만 바꾸는 것은 이미 저장된 모델의 잘못된 추론을 고치는 일이 아니다.
`assignment_stable` 진단에서도 학습 전 예측은 원본과 동일하다.

### 암기 테스트 (일반화 평가 아님)

세 이미지를 임시 디렉터리에 복사했다. val 이미지까지 학습했으므로 생성된
모델을 production으로 승격하거나 validation 점수로 보고하면 안 된다.
같은 원본 20 epoch 모델, seed 42, 512, batch 3, augmentation off,
Adam learning rate 0.001로 비교한다. 100회마다 출력/체크포인트를 남긴다.

- 기존 encoder 600회: confidence 0.4, IoU 0.5에서 GT 9개 중 6개 TP, FP 0.
  [예측](../../models/debug/tiny_retry_600/tiny_predictions.json),
  [학습 곡선](../../models/debug/tiny_retry_600/tiny.csv).
- 변경 encoder 600회: TP 7 / FP 0 / FN 2. 저장/재로드 후 원시 출력 차이 0.
  [예측](../../models/debug/tiny_stable_600/tiny_predictions.json).
  배정 개선은 확인했지만 가장 작은 초록불/노란불을 여전히 놓쳐 해결 완료로
  판정하지 않았다.
- 앞선 LR 0.0001/200회 실험은 마지막 GPU 출력 단계에서 CUDA 오류가 발생하여
  모델 저장/복원 검증이 완료되지 않았다. 성공한 실험으로 계산하지 않는다.

현재 재시도 비교 실험들은 CUDA 초기화 실패로 CPU에서 수행됐다. 드라이버/시스템
CUDA는 변경하지 않았다. GPU 재인식 문제는 detector 문제와 별도 확인이 필요하다.

### 선택적 DFL 보조 loss 실험

기존 600회 암기 모델의 첫 번째 노란불 anchor에서는 아래쪽 거리의 bin 1 확률이
0.999558, 기대 거리가 약 1.0017 stride였다. GT의 해당 거리는 약 0.55 stride다.
잘못된 bin에 몰린 분포를 CIoU(기대 거리)만으로 교정하는 것이 어려운지 추가 비교한다.
큰 거리 분포를 유지한 더 작은 두 GT의 회귀 신호도 확인 대상이다.

`dfl_loss.py`는 GT 거리 양옆 두 bin을 logit cross-entropy로 직접 지도한다.
[DFL 원 논문](https://arxiv.org/abs/2006.04388)의 방식이며 기존 64-channel 회귀
head에 보조 supervision만 추가한다. 추론 구조/클래스/좌표 형식은 바뀌지 않는다.
`dfl_weight=0`이면 기존 BCE+CIoU 경로 그대로이며, 실험 CLI에서만 1.5를 선택한다.
보조 loss와 설정은 `.keras`에 직렬화하고 저장/복원 테스트로 확인한다.

```bash
python ai/mission_detector/debug_detector.py --output models/debug/new_dfl_check \
  --tiny-epochs 600 --tiny-learning-rate 0.001 --stable-assignment --dfl-weight 1.5
```

quality-weighted DFL의 600회 결과는 TP 7 / FP 0 / FN 2이며 저장/복원 차이는 0이다.
박스 정확도는 개선됐으나 가장 작은 두 GT를 여전히 놓쳤다.

추가로 quality 가중치 자체가 매우 작은 GT의 DFL 신호를 억제하는지 분리하기 위해
`--dfl-weighting foreground` 옵션을 비교한다. 이 옵션은 **양성으로 실제 배정된
anchor만** 균등 가중치로 거리 분포를 지도한다. GT마다 균등한 가중치는 아니며,
배경과 padded box에는 가중치 0이다. 정확한 foreground mask를 위해 수정 encoder가
필수다. BCE/CIoU의 quality weighting은 건드리지 않는다. 기본값 `quality`는 유지한다.

```bash
python ai/mission_detector/debug_detector.py --output models/debug/new_foreground_check \
  --tiny-epochs 300 --tiny-learning-rate 0.001 --stable-assignment \
  --dfl-weight 1.5 --dfl-weighting foreground
```

이는 독립적인 진단 가설을 확인하는 실험적 weighting 정책이며, 원래 YOLOv8의
quality-weighted DFL과 같다고 주장하지 않는다. 설정은 `.keras`와 실험 JSON에
기록된다. 결과 경로는 `models/debug/tiny_foreground_dfl_300/`이다.

### 최종 대조 결과

모두 동일한 20 epoch checkpoint에서 시작한 3장 암기 실험이다. confidence 0.4,
IoU 0.5 기준이며 GT는 9개다. foreground 실험은 최대 300회로 실행했지만
200회에 9개 전체 검출/오탐 0 기준을 충족해 조기 종료했다.

| 방법 | 실제 반복 | TP | FP | FN |
|---|---:|---:|---:|---:|
| 원본 checkpoint, 암기 전 | 0 | 0 | 4 | 9 |
| 기존 encoder + BCE/CIoU | 600 | 6 | 0 | 3 |
| stable IoU encoder + BCE/CIoU | 600 | 7 | 0 | 2 |
| 위 설정 + quality-weighted DFL | 600 | 7 | 0 | 2 |
| 위 설정 + foreground-weighted DFL | 200 | **9** | **0** | **0** |

최종 [예측/매칭 JSON](../../models/debug/tiny_foreground_dfl_300/tiny_predictions.json):
00620 2/2, 00900 3/3, 05114 4/4 정답. IoU 범위 약 0.604–0.972.
confidence 범위 약 0.429–0.963.
[학습 후 저장/복원](../../models/debug/tiny_foreground_dfl_300/tiny_roundtrip.json)의
원시 boxes/classes 최대 차이는 모두 0이다. 실제 `infer.py`의 이미지 실행 결과는
`models/debug/cli_before/`와 `models/debug/cli_after/`에 별도로 저장했다.
실제 CLI에서도 각각 2/3/4개의 올바른 신호등이 `traffic_lights`에 들어가는 것을
확인했다. CLI before는 confidence 0.05, after는 더 엄격한 0.4로 실행했으며,
위 표의 동일-threshold 비교는 debug JSON의 0.4 매칭 결과다.

이 결과는 **작은 객체의 타깃 배정/회귀 supervision 실패를 보완하면 실제 동일
이미지 검출이 가능하다**는 근거다. 3장·단일 seed의 비교이며 본래 20 epoch 모델의
모든 실패 원인이 한 가지라고 입증한 것은 아니다. 수정된 암기 모델은 validation
이미지를 학습했으므로 독립 검증 또는 배포에 사용할 수 없다.

## 저장/복원과 회귀 테스트

- 원본 모델 roundtrip: 모든 가중치/원시 출력 최대 차이 0.
- 기존 encoder 600회 암기 모델: fit 직후와 저장/재로드 후 원시 출력 차이 0.
- 소형 offline 모델의 실제 fit 후 가중치·출력·Adam 상태 복원도 자동 테스트한다.
- 기존 29개 테스트에 검출 매칭, 작은 GT 배정 실패, 빈/padded GT, 충돌 시 class
  ownership, 기존/실험 encoder 모델 저장/복원 테스트를 추가했다.
- 실행 결과: 43 passed. 새 패키지 설치 없음.
- 최종 재실행도 43 passed (20.45초), `compileall` 성공. 이미지/라벨 916개와
  원본 20 epoch final/best 모델 SHA256 보존을 다시 확인했다.

## 파일 변경 범위

- 추가: `debug_detector.py`(재현/시각화/격리 암기), `evaluate_detections.py`(검출 평가),
  `detection_metrics.py`(일대일 매칭), `stable_label_encoder.py`(배정 수정 실험),
  `dfl_loss.py`(분포 지도), `DEBUG_REPORT.md`(증거/한계).
- 추가 테스트: `test_detection_metrics.py`, `test_model_roundtrip.py`,
  `test_stable_label_encoder.py`, `test_dfl_loss.py`.
- 수정: `model_factory.py`(기본 비활성 진단 옵션/직렬화), AI 폴더의 `README.md`.
- 수정: `train.py`에 목적함수 CLI를 연결하고 매 epoch class-aware validation
  precision/recall/F1 기록 및 F1 기준 checkpoint/early stopping을 추가했다.
  `evaluate_detections.py`의 같은 decoder/matcher를 재사용하며 model decoder는
  변경하지 않는다. 기존 evaluator 결과와 TP/FP/FN이 정확히 일치하는 회귀 검사와
  512 build-only 검사를 통과했다.
- 결과물: `models/debug/` 하위 새 디렉터리. 실패한 실험 기록도 보존했다.
- 수정하지 않음: `dataset.py`, `infer.py`, 클래스 순서, 기존 원본 데이터,
  production 모델, ROS2/STM32/제어 코드, 시스템 CUDA/드라이버 및 패키지 버전.
  Git에 원래 존재하던 루트 README/ROS 파일 변경은 이번 작업 변경이 아니다.

```bash
cd ~/hl_fma
source .venv/bin/activate
PYTHONPATH="$PWD/ai/mission_detector:$PWD" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest ai/mission_detector -q
python ai/mission_detector/debug_detector.py --output models/debug/new_stable_check \
  --tiny-epochs 600 --tiny-learning-rate 0.001 --stable-assignment
```

## 남은 판정

동일 이미지 검출/저장복원 진단은 통과했다. 본학습/독립 validation에서 이 수정안의
효과는 아직 평가하지 않았다. `train.py`의 기본 설정은 원래대로이며, 수정안을
본학습에 채택하려면 명시적인 옵션·설정 기록과 실제 검출 metric 검증을 함께
진행해야 한다. 이번 작업에서는 본학습 50 epoch를 실행하지 않았다.
yellow 독립 validation은 7장 확보했지만 단일 연속 촬영 세션이라 다양성은 부족하다.
vehicle_obstacle 등 적은 클래스도 이후 실제 영상/독립 데이터 검증이 필요하다.
ROS2 통합은 진행하지 않는다.

## 2026-09-07 전체 데이터 10 epoch 시험 학습

yellow 보강 후 수정안을 실제 train 314장에 처음 적용했다. 512×512, batch 2,
Adam 1e-4, stable assignment, foreground-weighted DFL 1.5를 사용했다. 인터넷 참고
이미지는 학습에 넣지 않고 test 평가에만 사용했다. 최초 F1 기준 trial은 F1=0이
계속되어 loss가 감소 중인데도 patience 3으로 4 epoch에서 멈췄다. 저장된 optimizer
상태를 이어 총 10 epoch까지 `val_loss` 기준으로 연장했다.

| epoch | train loss | val loss | val TP / FP / FN @0.4 | F1 |
|---:|---:|---:|---:|---:|
| 1 | 2079.74 | 1509.07 | 0 / 15100 / 149 | 0 |
| 4 | 770.26 | 766.41 | 0 / 9102 / 149 | 0 |
| 5 | 584.79 | 521.97 | 0 / 886 / 149 | 0 |
| 8 | 160.05 | 176.22 | 0 / 101 / 149 | 0 |
| 10 | 69.99 | 82.86 | 0 / 21 / 149 | 0 |

10 epoch final의 confidence 0.05 val은 TP 2 / FP 14546 / FN 147,
F1 0.000272다. 두 TP는 `traffic_light_green_straight`이며 yellow는 val 7개,
train 25개, 인터넷 test 29개 모두 TP 0이다. train 자체도 confidence 0.4에서
TP 0 / FP 105 / FN 516이고 0.05에서 TP 17 / FP 30724 / FN 499이므로, 현재 실패를
validation 사진 부족이나 domain shift만으로 설명할 수 없다.

[NMS 전 정렬 진단](../../models/runs/fixed_trial_10ep/alignment_diagnostic.json)에서는
val GT 149개 중 102개에 IoU 0.5 이상인 후보 박스가 존재했고 최고 IoU 중앙값은
0.650이었다. 그러나 최고-IoU 후보에서 정답 클래스가 top-1인 GT는 8개뿐이었다.
yellow 7개는 모두 최고 후보 IoU가 0.665–0.778이지만 정답 클래스 top-1은 0개이며,
yellow 점수가 가장 높은 anchor의 IoU는 모두 0이었다. 따라서 10 epoch 시점의 직접
병목은 박스 회귀보다 **정답 위치와 정답 클래스 점수의 결합 실패**다.

같은 코드/목적함수가 3장 반복 암기에서는 9/9를 검출했으므로 좌표 변환이나 출력
decoder가 완전히 끊긴 상태는 아니다. 전체 데이터의 각 이미지는 10회만 관측했고
detector head 학습률은 1e-4인 반면, 성공한 암기 비교는 같은 3장을 200회 반복하고
학습률 1e-3을 사용했다. 현재 근거의 우선순위는 (1) 낮은 학습률과 부족한 update,
(2) 희소 클래스/작은 객체 불균형, (3) quality-weighted BCE에서 공간별 클래스 신호가
약한 문제다. 다음에는 무작정 50 epoch를 돌리기 전에 학습률 또는 분류 loss만 분리한
짧은 대조 trial을 수행하고 train TP가 먼저 증가하는지 확인해야 한다.

산출물은 `models/mission_detector_fixed_trial_10ep.keras`와
`models/mission_detector_fixed_trial_10ep_best.keras`, 상세 로그/val/train/test 평가는
`models/runs/fixed_trial_10ep/`에 있다. 둘 다 시험 모델이며 production으로 승격하지
않는다.

### 학습률 통제 대조

분류 loss 자체 변경 전에 학습률만 분리했다. 동일한 10-epoch checkpoint에서 시작해
동일 3장, augmentation off, stable assignment, foreground DFL 1.5, 200 update를
사용했다. confidence 0.4, IoU 0.5 결과다.

| 학습률 | TP | FP | FN | precision | recall | F1 |
|---:|---:|---:|---:|---:|---:|---:|
| 1e-4 | 0 | 0 | 9 | 0 | 0 | 0 |
| 1e-3 | 7 | 1 | 2 | 0.875 | 0.778 | 0.824 |

1e-3은 100 update에도 TP 5 / FP 1 / FN 4였던 반면, 1e-4는 200 update 후에도
confidence 0.4 TP가 없었다. 1e-3의 200-update 결과는 00620 2/2, 00900 2/3,
05114 3/4 TP다. 두 결과 모두 저장 전후 raw boxes/classes 최대 차이는 0이었다.
산출물은 `models/debug/lr_compare_10ep_1e4/`와
`models/debug/lr_compare_10ep_1e3/`에 격리했다.

따라서 bbox/class mapping/decoder/save-load가 끊긴 pipeline bug라는 가설은
기각한다. 원래 encoder의 작은 GT 배정 실패는 확인된 training bug였고 stable
encoder+DFL이 이를 보완했다. 그 이후 전체 데이터 TP 0의 직접 병목은 1e-4에서
분류 confidence가 너무 느리게 상승한 것이다. 다만 tiny 암기 결과만으로 1e-3의
전체 validation 일반화를 보장하지 않으므로, 다음은 새 5–10 epoch 전체-data 대조
trial로 확인하고 50 epoch production 학습은 아직 실행하지 않는다.

## 2026-09-07 LR 1e-3 전체 데이터 10 epoch 대조

위 학습률 가설을 확인하기 위해 이전 모델을 resume하지 않고 COCO preset XS
backbone에서 새로 시작했다. 데이터, seed 42, 512x512, batch 2, augmentation on,
stable assignment, foreground DFL 1.5는 유지하고 학습률만 1e-3으로 변경했다.
인터넷 이미지는 계속 test reference에만 두었으며 학습에는 사용하지 않았다.

| epoch | train loss | val loss | val TP / FP / FN @0.4 | F1 |
|---:|---:|---:|---:|---:|
| 1 | 815.15 | 95.19 | 0 / 0 / 149 | 0 |
| 2 | 25.87 | 16.14 | 0 / 0 / 149 | 0 |
| 3 | 8.10 | 7.97 | 0 / 18 / 149 | 0 |
| 5 | 6.94 | 8.18 | 0 / 241 / 149 | 0 |
| 8 (best loss) | 6.34 | **5.63** | 0 / 0 / 149 | 0 |
| 10 | 6.08 | 5.79 | 0 / 0 / 149 | 0 |

1e-4 대조보다 loss 수렴은 훨씬 빨랐지만 실제 class-aware 검출은 개선되지
않았다. best-loss 모델을 별도로 평가한 결과는 다음과 같다.

| split | confidence | TP | FP | FN | F1 |
|---|---:|---:|---:|---:|---:|
| train | 0.05 / 0.4 | 0 | 0 | 516 | 0 |
| val | 0.05 / 0.4 | 0 | 0 | 149 | 0 |
| test reference | 0.05 / 0.4 | 0 | 0 | 336 | 0 |

Yellow도 train 25, val 7, test reference 29 GT 모두 TP 0이다. best 모델의
40개 train 이미지 raw 출력을 표본 계측하면 최대 class score는 0.0271,
foreground anchor 평균 최대 score는 0.00289였다. 215,040 anchor 중 foreground는
516개(0.240%)였다. 이미 score가 낮아진 시점의 BCE 합은 foreground 99.53,
background 8.66으로, 단순히 background loss가 계속 수치상 지배한다고만 설명할
수는 없다.

따라서 tiny 3장/200-repeat의 LR 1e-3 성공은 전체 데이터 10-pass 일반화를
보장하지 않았다. 좌표/decoder/save-load 오류는 앞선 진단으로 기각됐고, 이번에는
train에서도 검출이 0이므로 validation 자료 부족이 직접 원인도 아니다. 남은 주요
가설은 (1) 각 이미지 10회 관측으로는 부족한 update 수, (2) augmentation이 작은
객체의 학습 신호를 어렵게 만드는 효과, (3) 동적 quality target과 BCE의
classification 최적화 불안정성이다. 다음 시험은 50 epoch 본학습 전에 동일한
소규모·다양한 subset에서 augmentation on/off와 positive/classification weighting을
한 변수씩 분리해야 한다.

산출물은 `models/mission_detector_lr1e3_trial_10ep.keras`,
`models/mission_detector_lr1e3_trial_10ep_best.keras`와
`models/runs/lr1e3_trial_10ep/`에 보존했다. best는 epoch 8의 val-loss 모델이며
둘 다 배포용이 아니다. 최초 평가 프로세스의 XLA libdevice 탐색 실패는
`XLA_FLAGS=--xla_gpu_cuda_data_dir=.../nvidia/cuda_nvcc`를 평가 프로세스에만
지정해 해결했으며 시스템 CUDA나 패키지는 변경하지 않았다.

## 2026-09-07 총 50 epoch 연장 결과

사용자 요청에 따라 LR 1e-3 10-epoch final archive의 optimizer step 1,570을
복원하고 같은 데이터/목적함수/augmentation으로 40 epoch를 추가해 총 50 epoch까지
학습했다. `train.py --resume`의 `--epochs`는 추가 횟수이므로 40을 지정했으며 로그가
`Epoch 11/50`부터 `Epoch 50/50`까지인 것을 확인했다. 인터넷 이미지는 계속 test
reference에만 사용했다.

| 모델 | val loss | val TP / FP / FN @0.4 | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|
| epoch 41 best-loss | **5.0918** | 0 / 2 / 149 | 0 | 0 | 0 |
| epoch 50 final | 5.7415 | **5 / 0 / 144** | 1.0 | 0.0336 | **0.0649** |

Loss 기준 checkpoint가 실제 detection 기준 최선은 아니었다. epoch 50 final을
confidence 0.05로 낮추면 val TP 19 / FP 72 / FN 130, F1 0.1583이다. confidence
0.4의 5 TP는 모두 `road_left_arrow`이고, 다른 11개 클래스의 validation TP는 0이다.

| split (epoch 50 final) | confidence | TP | FP | FN | F1 |
|---|---:|---:|---:|---:|---:|
| train | 0.4 | 90 | 25 | 426 | 0.2853 |
| train | 0.05 | 229 | 290 | 287 | 0.4425 |
| val | 0.4 | 5 | 0 | 144 | 0.0649 |
| val | 0.05 | 19 | 72 | 130 | 0.1583 |
| test reference | 0.4 | 0 | 1 | 336 | 0 |
| test reference | 0.05 | 6 | 112 | 330 | 0.0264 |

Yellow는 epoch 50에서도 train 25, val 7, test reference 29 GT 모두 TP 0이다.
test의 0.05 TP 6개는 모두 `signal_car_red_x`다. 즉 update 수를 늘리면 train 검출과
일부 validation 검출은 생기지만, 독립 데이터 일반화와 클래스 균형은 여전히 매우
부족하다. 이 결과는 classification/희소 클래스 개선이 필요하다는 기존 결론을
강화하며 production 또는 ROS2 제어에 사용하기에는 부족하다.

산출물은 `models/mission_detector_lr1e3_total50.keras`,
`models/mission_detector_lr1e3_total50_best.keras`, 상세 로그와 고정-threshold 평가는
`models/runs/lr1e3_total50/`에 있다. 시작 시 `--epochs 50`이 추가 50회를 뜻하는 것을
발견해 첫 재개를 저장 전에 중단했으며, 그 빈/중단 기록은
`models/runs/lr1e3_trial_50ep/`에 보존했다. 원본 10-epoch archive는 변경되지 않았다.

## 2026-09-07 Ultralytics sign panel classifier 시험학습

새 perception schema는 detector class를 `traffic_light`, `sign_panel`,
`message_board`로 분리한다. 신호차 차체 전체는 class로 사용하지 않으며, 상단 LED
패널 3개를 각각 검출한 뒤 center_x 기준 SLOT 1/2/3으로 연결한다. 기존 mission
라벨 중 7/9/10은 실제 패널 전체를 감싸므로 각각 LANE_CHANGE/RED_X/GREEN_ARROW
classification crop으로 변환했다. 기존 train/val/test 주행 세션 구분을 그대로
보존했고 무작위 이미지 split은 하지 않았다.

| split | GREEN_ARROW | RED_X | LANE_CHANGE | total |
|---|---:|---:|---:|---:|
| train | 17 | 26 | 9 | 52 |
| val | 8 | 13 | 3 | 24 |
| test | 103 | 153 | 51 | 307 |

별도 `.venv-yolo`에 Ultralytics 8.4.142와 CUDA 12.8용 PyTorch 2.11.0을 설치했다.
RTX 3050 4GB에서 GPU smoke test를 통과했다. YOLO11n-cls를 최대 50 epoch,
early-stopping patience 20으로 시험했으며 epoch 32에 종료되고 epoch 12 모델이
best로 선택됐다. 한글 문구를 뒤집거나 작은 LED 심벌을 지우지 않도록 좌우/상하
flip과 random erasing은 비활성화했다.

| 평가 | 결과 |
|---|---:|
| validation top-1 | 0.917 |
| 독립 test top-1 | 0.857 |
| test confidence 0.70 coverage | 0.860 |
| test confidence 0.70 accepted accuracy | 0.894 |

confidence 0.70에서 test 혼동은 GREEN_ARROW 103장 중 정답 68, RED_X 오분류 14,
UNKNOWN 20으로 GREEN_ARROW 보강이 가장 시급하다. RED_X는 153장 중 128장,
LANE_CHANGE는 51장 중 40장을 맞혔다. 이 모델은 시험용이며 새 영상/거리/조명에서
추가 검증하기 전 제어 판단에 사용하면 안 된다. checkpoint는
`autonomous_vision/models/sign_classifier_best.pt`, 학습 기록은
`autonomous_vision/runs/sign_classifier_50ep/`에 있다.
RTX 3050에서 세 panel을 한 batch로 처리한 warm-up 이후 지연시간은 20회 기준
median 5.47 ms, mean 5.43 ms, 범위 4.69~6.16 ms였다. 첫 호출은 모델/CUDA
warm-up을 포함해 약 551 ms였으므로 런타임 시작 시 warm-up이 필요하다. 이 수치는
개발 PC 측정값이며 Jetson 성능을 의미하지 않는다.

사용자가 추가한 `picturesandvideos/testvideo`의 두 영상을 검사했다. 신호등 영상은
가로형 4구 하우징의 위치/크기 변화가 있고, 신호차 영상은 상단 개별 패널 3개와
하단 message_board가 명확하다. 다만 기존 신호등 annotation은 켜진 lamp만 감싸며
message_board annotation은 없으므로 3-class detector 데이터로 자동 변환하지
않았다. 다음 필수 단계는 이 두 영상에서 traffic_light 전체 하우징, 각 sign_panel,
message_board를 새 기준으로 완전하게 라벨링한 뒤 detector를 학습하는 것이다.
두 testvideo는 기존 train 추출 세션과 동일한 촬영 시각과 장면이며 일부는 30 FPS로
재인코딩된 복사본이다. 기능 시연에는 쓸 수 있지만 독립 test metric에는 사용하지
않는다.

## 2026-09-07 3-class detector 50 epoch 시험학습

새 기준으로 수동 저장한 105장(train 83, val 22)을 검사했다. 대응 라벨 누락과
YOLO 좌표/class 오류는 0건이었다. 세션별 하위 디렉터리를 유지하기 때문에
`train_detector.py`의 사전 검사도 이미지를 재귀 탐색하고 같은 상대 경로의 라벨을
찾도록 수정했다. test split은 독립 촬영 세션이 아직 없어 0장이며 학습에는 사용하지
않았다.

YOLO11n pretrained detector를 RTX 3050에서 640 px, batch 4로 50 epoch 학습했다.
Ultralytics `optimizer=auto`가 선택한 optimizer는 AdamW(lr 0.001429)였다.

| class | val instances | precision | recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|
| 전체 | 63 | 0.750 | 0.686 | 0.725 | 0.339 |
| traffic_light | 7 | 0.562 | 0.429 | 0.417 | 0.237 |
| sign_panel | 42 | 0.815 | 0.629 | 0.763 | 0.254 |
| message_board | 14 | 0.874 | 1.000 | 0.995 | 0.525 |

best checkpoint는 `autonomous_vision/models/detector_best.pt`에 배치했고 원본 run은
`autonomous_vision/runs/detector_50ep/`에 보존했다. 두 testvideo를 1초 간격으로
샘플링한 기능 점검에서 신호등 영상은 45 frame 중 traffic_light 42 frame을 검출했고
message_board 오검출이 2 frame 있었다. 신호차 영상은 36 frame 중 sign_panel 32
frame(총 101개), message_board 34 frame에서 검출했다. 이 영상들은 학습 원본과 같은
세션이라 이 수치는 독립 일반화 성능이 아니다.

주석 결과 영상은 `autonomous_vision/models/test_traffic_annotated.mp4`와
`autonomous_vision/models/test_sign_vehicle_annotated.mp4`에 저장했다. 현재 가장
약한 detector class는 traffic_light이며, 다음 촬영에서는 다른 거리·조명·배경의
일반 신호등 세션과 신호차가 아닌 물체를 포함한 hard-negative 세션을 별도 test로
확보해야 한다.

신호차 결과에서 LANE_CHANGE가 UNKNOWN으로 보이는 사례를 조사했다. 15-frame 간격
표본에서 classifier top-1이 LANE_CHANGE인 crop은 62개였고 이 중 12개가 기본
confidence 0.70 미만이었다. 독립 classifier test 307장에서 threshold를 0.70에서
0.60으로 낮추면 coverage는 0.860에서 0.938로 증가하지만, 승인된 예측 정확도는
0.894에서 0.872로 감소했다. 따라서 기본값 0.70은 유지하고 `main.py`에
`--classifier-confidence` 실행 옵션을 추가했다. raw/stable 상태를 화면에 함께
표시하며, threshold 0.60 비교 영상은
`autonomous_vision/models/test_sign_vehicle_threshold060.mp4`에 저장했다.

## 2026-09-07 실제 교차로 독립 test

새로 촬영한 163256(4.24초)과 165023(2.60초) MOV를 어느 학습 split에도 섞지 않고
두 개의 독립 test 세션으로 고정했다. 약 3.33 fps로 각각 15장과 9장을 추출했고,
수동 라벨 24장/traffic_light 63개를 검사한 결과 누락 및 형식 오류는 0건이었다.

| 입력 크기 | test precision | test recall | mAP50 | mAP50-95 |
|---:|---:|---:|---:|---:|
| 640 | 0.0095 | 0.365 | 0.0666 | 0.0248 |
| 1280 | 1.0000 | 0.504 | 0.6220 | 0.2371 |

640의 precision/recall은 Ultralytics PR curve 요약값이다. 실제 기본 confidence 0.25
고정 평가에서는 TP 0 / FP 6 / FN 63이었다. test bbox 높이 중앙값은 원본에서
26 px이며 640 letterbox 입력에서는 약 9 px에 불과하다. 1280에서 크게 개선된 것은
주요 원인이 작은 객체 해상도임을 보여준다. 1280 고정 confidence 0.05에서는
traffic_light 예측 32개가 모두 IoU 0.5 TP였지만, 동시에 sign_panel/message_board
오검출 96개가 발생했다. 즉 입력 크기만 높이는 것으로 전체 3-class 시스템이
완성되지는 않는다. 이 test 세션은 이후에도 train/val로 이동하지 않고 보존해야
하며, 별도의 실제 교차로 train 세션과 hard-negative 세션을 추가해야 한다.
