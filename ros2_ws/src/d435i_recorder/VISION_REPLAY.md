# D435i 차선·정지선 영상 검토 도구

## 원본 저장소 조사

조사 대상: https://github.com/wonsukHeo1025/2025-HL-FMA-Driverless-Morai-Simulation

확인 commit: `9304267ec5c377ab3e142c057a37a8012e3fe4f0` (`main`).

`Perception/traffic_light_detection/scripts/stop_line_detector.py`는
Odometry/Path와 CSV GPS 좌표를 사용하여 정지선까지 경로 누적거리를 계산한다.
카메라 입력이나 영상 정지선 분할은 없다. 같은 폴더의
`traffic_light_detector.py`는 YOLO 신호등 검출이다. 전체 파일 트리에서
영상 차선 인식 구현은 찾지 못했다. 따라서 이식했다고 주장할 수 있는
차선/정지선 영상 알고리즘은 없으며, 아래 모듈은 별도 작성한 초기 기준 구현이다.
다른 브랜치/파일이 제공되면 비교·이식할 수 있다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source /home/idp2/fma_autonomous_vehicle/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 ros2 run d435i_recorder vision_replay \
  ~/d435i_recordings/20260909_185416_708387 \
  --output ~/d435i_vision_reviews/my_review
```

출력 디렉터리는 기존에 없어야 하고 원본 세션 밖이어야 한다.
`--config path.json`으로 `road_vision.VisionConfig` 필드를 재정의한다.
ROS publisher/subscriber, 카메라 실행, 전역 키 처리 없이 저장 파일만 읽는다.
실시간 recorder pipeline에는 연결하지 않는다.

## 동작과 제한

- 640×480 실외 D435i 영상 기준 ROI 상단 48%, 노랑/흰색 HSV 마스크,
  윤곽선 길이·두께·방향으로 세로 차선 및 가로 흰색 정지선 후보를 추출한다.
- 횡단보도 형태는 ambiguity로 별도 기록한다. 시각적 휴리스틱이므로
  차량/보도/깨진 도색도 오인할 수 있으며, 신뢰도나 정확도를 주장하지 않는다.
- 카메라 장착 높이/피치·지면 보정 없이 픽셀 차선을 미터 단위 경로나
  조향값으로 변환하지 않는다. 한 차선에서 반대편 차선을 임의 생성하지 않는다.
- RGB index와 원래 timestamp를 그대로 JSONL에 보존한다. depth는 index가 아닌
  timestamp 최근접, 최대 20 ms로 매칭한다. 실패는 null로 남긴다.
- aligned depth의 uint16·해상도를 확인하고 후보의 흰색 픽셀에서 유효 깊이
  중앙값을 계산한다. optical Z이며 지면상 거리나 범퍼까지 거리가 아니다.
- 원본 MP4/16-bit PNG/JSONL을 변경하지 않는다. overlay MP4는 검토용 재인코딩이며
  원본 컨테이너 FPS를 유지한다. 실제 촬영 시간 간격은 detections.jsonl을 참조한다.
- 모든 RGB 프레임을 처리하고 출력 전체를 다시 디코딩해 개수를 검사한다.
  RGB metadata/video 개수 불일치는 오류로 보고한다.

## 실제 녹화 재생 검증

세션 `20260909_185416_708387`, 결과
`~/d435i_vision_reviews/20260909_185416_baseline_v1/`:

- 입력/출력 전체 디코딩 4,500 / 4,500 프레임, timestamp rate 29.9799 Hz.
- 차선 후보가 있는 프레임 1,107, 정지선 후보 648, 횡단보도 의심 907.
  이는 후보 발생 빈도이며 정답 라벨 기반 검출 정확도가 아니다.
- depth timestamp 매칭 4,449 / 4,500; 51프레임은 20 ms 내 매칭 없음.
- 대표 프레임 시각 검토: 시작의 가로 흰색 띠, 90초 부근 노란 차선 후보를
  검출한다. 60초의 횡단보도 가장자리 오검출, 120초 노란 선 미검출,
  마지막 주차 차량 구간의 횡단보도 오인도 확인했다. 실차 제어용 검증은 미완료다.
- `overlay.mp4`, `contact_sheet.jpg`, `detections.jsonl`, `summary.json`을 함께 검토한다.

다음 튜닝은 장면별 정답 라벨과 카메라 장착 자세를 확보한 뒤 진행한다.
본 도구는 사용자 제어 코드와 독립적이며 git add/commit/push를 수행하지 않는다.

## 현재 작업 트리의 기존 차선 알고리즘 결합

```bash
PYTHONNOUSERSITE=1 ros2 run d435i_recorder vision_replay \
  ~/d435i_recordings/20260909_185416_708387 \
  --lane-backend fma \
  --output ~/d435i_vision_reviews/existing_lane_review
```

현재 branch는 `feature/d435i-lane-following`이며 로컬 `main` 트리에는
`ros2_ws/src/fma_perception` 파일이 없다. 사용자가 지칭한 기존 코드를
현재 작업 트리의 `fma_perception/lane_detection.py`로 해석했다.
`detect_lane` 순수 함수를 직접 호출하며 ROS node는 실행하지 않는다.
기존 알고리즘/기본 LaneConfig를 변경하지 않고 기존 debug 영상 위에 정지선
후보만 추가한다. 별도 baseline의 차선 후보는 결합 출력에서 제외한다.
`--lane-config path.json`으로 기존 LaneConfig 필드를 재정의할 수 있다.

`summary.json`에는 실제 import한 파일 경로, SHA256, 설정을 기록한다.
이번 설치 코드와 작업 트리 원본 SHA256은 동일하다:
`e23272e1ef10e40c2278bc902345d1ce02767962589cc40da4dd2478ec0197fb`.
JSONL의 `existing_lane`에 원래 알고리즘의 visual_detected/detected/confidence를
기록하며 유효하지 않은 미터 단위 출력은 null로 기록한다.

결과: `~/d435i_vision_reviews/20260909_185416_existing_lane_v1/`.
입력/출력 4,500프레임, 원래 timestamp 유지, visual_detected 27프레임,
metric detected 0프레임(보정 미설정), 정지선 후보 648프레임이다.
기존 알고리즘에서 visual_detected는 양쪽 차선 형상 조건을 뜻하므로,
기존 baseline의 단일 후보 발생 수 1,107과 직접 정확도 비교할 수 없다.
이번 작업은 기존 알고리즘을 영상에 결합한 결과이며 정확도 개선은 아니다.
테스트 41개 통과, d435i_recorder build와 git diff --check 통과.

## 색상 조건 수정: 노란 차선 / 흰 정지선

사용자 확인에 따라 현재 replay의 차선 후보는 노란색만 사용한다.
기존 차선 함수에 `yellow_only` 선택 인자를 추가하고 `ExistingLane`에서
이를 활성화한다. 흰색 마스크 자체를 제외하므로 흰 정지선/횡단보도가
차선 Hough 입력에 들어가지 않는다. 기본 baseline도 노란색만 차선으로 취급한다.
흰색 정지선 후보 추출·depth 매칭은 그대로 유지한다.
기존 ROS 차선 node는 변경하지 않았으며 이 선택 인자의 기본값은 False다.
실시간 적용은 별도로 연결해야 한다.

동일한 `--lane-backend fma` 실행 명령으로 색상 분리가 적용된다.
새 결과는 `~/d435i_vision_reviews/20260909_185416_yellow_lane_white_stop_v1/`이다.
이전 결과는 수정 전 비교 자료로 보존한다. 이전 SHA/검출 수는 이전 실행에만
해당하며 현재 소스 SHA와 결과 수는 새 `summary.json`을 참조한다.
