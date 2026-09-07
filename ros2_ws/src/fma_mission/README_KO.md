# 순차 waypoint mission manager

`ros2 run fma_mission mission_manager`는 설치된 `config/waypoints.yaml`을 사용한다.
다른 radius 설정 파일은 `--ros-args -p waypoints_file:=/path/waypoints.yaml`로
지정한다. 기본 radius는 각 3.0 m이며 실측 좌표는 변경하지 않는다.

입력: `/gps/fix` (NavSatFix), `/mission/status` (MissionState).
출력: `/mission/current` (MissionState), 상태 변경 즉시 및 2 Hz 반복.
START를 먼저 발행하고 첫 0.5초 timer에서 NORMAL_DRIVE로 전환한다.
현재 target 하나만 Haversine 거리로 검사하며 반경 이내일 때 활성화한다.
GPS status가 NO_FIX/알 수 없는 값이거나 위경도가 유효하지 않으면 무시한다.
활성 mission 동안 GPS는 진행에 사용하지 않는다.

현재 mission과 같은 current_mission, completed=true만 완료로 인정한다.
WP01~09는 완료 후 다음 target의 NORMAL_DRIVE가 된다. WP10 SIGNAL_CAR 완료는
GPS 없이 즉시 LANE_CHANGE를 활성화하며 그 완료 후 WP11로 진행한다.
WP11 도착은 terminal FINISH이고 이후 GPS/완료 입력은 진행시키지 않는다.
FINISH는 active=false, completed=true, 일반 mission은 active=true이다.

GPS는 trigger만 담당한다. 구동 명령·조향·주차·회전 maneuver는 구현하지 않는다.
MANUAL > EMERGENCY > MISSION > LANE 우선순위를 따른다. MissionState enum 숫자가
변경되므로 관련 패키지를 함께 재빌드하고 기존 기록/외부 소비자의 enum도 맞춰야 한다.
현재 메시지에는 execution ID가 없으므로 일치하는 mission의 오래된 완료 메시지와
새 완료를 구별하지 못한다. 재시작은 WP01부터 시작하며 자동 복구는 구현하지 않는다.

오프라인 검증: `python3 -m pytest ros2_ws/src/fma_mission/test` (ROS 환경 source 후).


## GPS 없는 단일 mission 연습

`mission_test_runner`는 competition mission_manager와 별개인 node이다.
**Do not run mission_manager at the same time.** 두 node는 `/mission/current`의
발행자가 겹치므로 함께 실행하면 안 된다. 단독 실차 연습에서는 사용하지 않는
lane controller도 실행하지 않는 것을 권장한다.

필수 startup parameter `test_mission`은 다음 이름만 허용한다:
RAMP, INTERSECTION_STRAIGHT_1, S_CURVE, INTERSECTION_STRAIGHT_2,
PERPENDICULAR_PARKING, INTERSECTION_LEFT, CHILD_DUMMY, PARALLEL_PARKING,
INTERSECTION_RIGHT, SIGNAL_CAR, LANE_CHANGE.
누락·오타·START/NORMAL_DRIVE/FINISH는 활성화 없이 오류로 종료한다.

```bash
ros2 run fma_mission mission_test_runner --ros-args -p test_mission:=S_CURVE
ros2 run fma_mission mission_test_runner --ros-args -p test_mission:=PERPENDICULAR_PARKING
ros2 run fma_mission mission_test_runner --ros-args -p test_mission:=CHILD_DUMMY
ros2 run fma_mission mission_test_runner --ros-args -p test_mission:=PARALLEL_PARKING
ros2 run fma_mission mission_test_runner --ros-args -p test_mission:=SIGNAL_CAR
```

실행 즉시 선택한 mission을 active=true, completed=false로 발행한다.
Competition node와 같은 reliable/transient-local depth 1 및 2 Hz steady timer를
사용한다. GPS 구독이나 waypoint 파일 의존성은 없다.
`/mission/status`에서 같은 mission의 completed=true만 인정하며, 완료 즉시
active=false, completed=true를 발행하고 TEST COMPLETE를 출력한다.
이후에도 같은 완료 상태를 반복 발행하여 늦게 연결된 consumer가 수신할 수 있게
유지한다. 다음 mission이나 WP10 chain을 시작하지 않으며 재실행 전까지 재활성화하지 않는다.

Runner는 MissionState만 발행한다. `/cmd/mission`, `/cmd/lane`, `/cmd/final`,
`/vehicle/command` 또는 serial에는 접근하지 않는다. 실제 motion은 향후 mission
maneuver node가 담당한다. Runner 종료 자체는 차량 STOP 명령이 아니며 기존
명령 timeout과 emergency 안전 경로를 유지해야 한다.
