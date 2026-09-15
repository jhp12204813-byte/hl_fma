# 최종 90 waypoint / mission 구조

기본 `config/waypoints.yaml`은 확정 번호 1~90, 이름, 좌표를 사용한다.
57번은 latitude/longitude 모두 null이다. 좌표 생성·보간은 하지 않는다.
MissionState enum 0~13은 유지하며 ControlMode는 독립 정책이다.
최종 코스에는 LANE_CHANGE와 별도 FINISH waypoint를 추가하지 않는다.
90번 SIGNAL_CAR 종료 조건 충족 후 FINISH로 고정된다.

## 완료 정책

YAML 최상위 `completion_policies`에서 미션별로 반드시 명시한다.

| 미션 | 구간 | 기본 completion_policy |
|---|---|---|
| RAMP | 4~5 | gps |
| INTERSECTION_STRAIGHT_1 | 22~24 | gps |
| S_CURVE | 30~33 | gps |
| INTERSECTION_STRAIGHT_2 | 35~37 | gps |
| PERPENDICULAR_PARKING | 37~40 | gps |
| INTERSECTION_LEFT | 51~55 | gps |
| CHILD_DUMMY | 67~76 | gps |
| PARALLEL_PARKING | 81~82 | gps |
| INTERSECTION_RIGHT | 83~86 | gps |
| SIGNAL_CAR | 89~90 | gps |

저장소 조사에서 실제 `/mission/status` completion publisher를 확인하지 못했다.
기본값은 **오프라인 GPS 진행 테스트용**이다. GPS 도착은 실제 주차/장애물 회피/
신호 준수 완료의 증거가 아니다. 이 설정은 실차 운행 승인이나 완성된 controller를
의미하지 않는다. 실제 controller 연결 후 해당 미션의 정책을 명시적으로 변경한다.

- `gps`: exit GPS 도착으로 종료. status가 없어도 진행한다.
- `status`: 해당 미션 completed=true로 종료. exit GPS 도착을 요구하지 않는다.
  단, 앞선 hard waypoint는 실제 처리되어 exit가 순차적으로 eligible해야 한다.
  먼저 받은 status는 저장하며 앞선 soft guide는 건너뛸 수 있다.
- `gps_and_status`: exit GPS 도착과 해당 미션 completed=true가 모두 필요하다.
  어느 순서로 받아도 처리한다. GPS만으로 강제 완료하지 않는다.

미션 entry에서 완료 수신 플래그를 초기화한다. 완료 메시지는 현재 미션과 일치하는
completed=true만 받으며 중복은 무시한다. 완료 대기 중 주차 COMPLETE는 STOP이다.
status를 기다리는 장애물 미션은 OBSTACLE 소유권을 유지한다. controller가 종료
대기 중 실제 정지를 판단해야 한다.
기존 MissionState 메시지에는 execution ID가 없어 재시작 이전의 동일 미션 status나
feedback과 새 실행 입력을 완전히 구별할 수 없다. 자동 재시작/재개 기능은 없다.

## Waypoint 진행

START → 첫 0.5초 timer에서 NORMAL_DRIVE. 현재 target부터 순차 처리한다.
유효한 NavSatFix(status 0/1/2, 유한·범위 내 좌표)만 거리 계산에 사용한다.

| waypoint_type | 의미 |
|---|---|
| route | 도착 시 진행, 미션 시작 없음 |
| mission_approach | 접근 정보, 미션 상태/현재 phase 변경 없음 |
| mission_entry | 미션과 실제 entry phase 활성화 |
| mission_guide | 미션 유지, 실제 도착 후 phase 갱신 |
| mission_exit | GPS 도착 기록 및 해당 완료 정책 적용 |
| mission_transition | 이전 종료와 다음 시작을 단일 상태 갱신으로 처리 |

37번은 INTERSECTION_STRAIGHT_2 → PERPENDICULAR_PARKING이다. 이전 정책의 완료
조건 충족 후 APPROACH로 진입하고 target은 38번이 된다. 중간 NORMAL_DRIVE 발행은
없다. 단조 증가하는 target과 처리 번호 기록으로 jitter/중복 status를 무시한다.

57번은 목록에 그대로 남는다. 56번 처리 후 target이 57번이 되면 거리 계산 없이
`WAYPOINT number=57 id=ROUTE_057 coordinate unavailable, skipped`를 기록하고
58번으로 진행한다. reached 이벤트를 기록하지 않는다. 다른 null 좌표는 오류다.

31/32, 68~75는 hard_point=false이다. 현재 soft guide부터 연속된 soft guide와
그 직후 첫 hard boundary까지만 도착 여부를 탐색한다. 뒤 guide/exit에 도착하면
앞 soft guide를 skipped로 기록한다. 전역 nearest 탐색은 하지 않으며 hard route,
entry, exit, transition을 임의로 건너뛰지 않는다.
GPS guide는 방향 참고용이다. D435i 검출과 OBSTACLE controller의 steering·감속·
정지·회피 결정을 덮어쓰지 않는다. 이 패키지는 회피 궤적이나 구동 명령을 만들지 않는다.

## MissionState / phase / ControlMode

phase는 다음 target이 아니라 실제 처리한 waypoint 또는 검증된 controller phase
보고를 기준으로 한다. target=39만으로 REVERSE가 되지 않는다.

| MissionState | phase / 조건 | ControlMode |
|---|---|---|
| START | 초기 | STOP |
| NORMAL_DRIVE | 일반 route | LANE |
| NORMAL_DRIVE | 주차 종료 직후 COMPLETE, 다음 route/entry 처리 전 | STOP |
| RAMP | ENTRY | LANE |
| INTERSECTION_STRAIGHT_1/2 | ENTRY/MID, 유효한 proceed 없음 / 있음 | STOP / LANE |
| S_CURVE | ENTRY/GUIDE/종료 대기 | OBSTACLE |
| PERPENDICULAR_PARKING | APPROACH/ALIGN | LANE |
| PERPENDICULAR_PARKING | REVERSE | REVERSE |
| PERPENDICULAR_PARKING | COMPLETE/PARKED | STOP |
| INTERSECTION_LEFT | ENTRY/STOP/TURN_1/TURN_2, proceed 없음 / 있음 | STOP / LANE |
| CHILD_DUMMY | ENTRY/GUIDE/종료 대기 | OBSTACLE |
| PARALLEL_PARKING | APPROACH | LANE |
| PARALLEL_PARKING | REVERSE | REVERSE |
| PARALLEL_PARKING | COMPLETE/PARKED | STOP |
| INTERSECTION_RIGHT | ENTRY/TURN_1/TURN_2, proceed 없음 / 있음 | STOP / LANE |
| SIGNAL_CAR | ENTRY, proceed 없음 / 있음 | STOP / LANE |
| 교차로/SIGNAL_CAR | 실제 EXIT 도착 후 외부 완료 대기 | LANE |
| FINISH | terminal | FINISH |

종료 정책 충족 후 일반 미션은 NORMAL_DRIVE로 복귀한다. 마지막 처리 phase는
EXIT 또는 COMPLETE로 유지하다 다음 route 도착 시 null로 바뀐다. 37번은 즉시
주차 APPROACH로 전환한다. 90번은 바로 FINISH이다. waypoint YAML의
recommended_control_mode는 참고 메타데이터이며 현재 상태 정책을 덮어쓰지 않는다.
GPS fallback 조건 함수는 기존대로 보존하며 자동 GPS 전환은 연결하지 않았다.

## 토픽 / 최소 추가 인터페이스

기존:

- `/gps/fix`: sensor_msgs/NavSatFix 입력.
- `/mission/status`: fma_interfaces/MissionState 완료 입력.
- `/mission/current`: MissionState 출력, reliable/transient-local depth 1, 2 Hz.
- `/mission/control_mode`: std_msgs/String 출력, 같은 QoS/주기.
- `/cmd/mission`: safety의 DriveCommand 정지 출력, 20 Hz.

새 입력은 기존 std_msgs/String을 사용한다. 새 ROS 메시지 패키지는 없다.

### `/mission/feedback` (String, reliable/volatile depth 10)

외부 perception/controller의 JSON 보고:

```json
{"mission":"INTERSECTION_STRAIGHT_1","phase":"MID","proceed":true}
```

- 현재 활성 미션과 일치해야 한다. phase 생략 시 실제 현재 phase를 사용한다.
- 교차로/신호차 phase는 실제 처리된 waypoint phase와 일치해야 한다.
- proceed는 bool만 허용한다. true 수신 후 1.5초 동안 유효하며 phase/미션 변경,
  false, 잘못된 입력, 만료 시 해제한다. 2 Hz timer에서도 만료를 반영하므로
  실제 모드 갱신에는 최대 한 timer 주기가 추가될 수 있다.
- 주차 phase는 APPROACH → ALIGN → REVERSE → PARKED → COMPLETE 순서로
  동일 단계 재보고 또는 다음 단계 보고만 허용한다. 평행주차만 APPROACH → REVERSE도
  허용한다. 이는 controller가 실제 단계 진입을 확인한 보고여야 한다.
- 이후 GPS guide 도착으로 주차 phase가 뒤로 돌아가지 않는다.
- feedback은 mission completion을 대신하지 않는다. 완료는 기존 `/mission/status`다.
- 수신자의 시각으로 신선도를 판단한다. 인증/실행 ID 인터페이스는 아니다.

### `/mission/controller_ready` (String, reliable/volatile depth 10)

현재 저장소에는 재사용할 readiness 토픽이 없어 최소 heartbeat를 추가했다.

- 값은 현재 ControlMode와 일치하는 `GPS`, `OBSTACLE`, `REVERSE` 중 하나다.
- controller는 현재 모드에서 센서/제어가 준비되고 유효한 명령 스트림을 유지할 때만
  10 Hz 이상 heartbeat를 발행한다. STOP은 readiness 철회용이다.
- 모드 변경 시 readiness를 지운다. 해당 모드 진입 후 새 heartbeat가 필요하다.
- 잘못된 값/다른 모드 heartbeat도 readiness를 철회한다.
- 마지막 heartbeat 후 0.3초 이상이면 safety가 정지를 발행한다. 확인 주기는 0.05초다.
- readiness는 controller의 계약이다. 이 패키지는 실제 명령 스트림을 검사하거나
  controller를 구현하지 않는다. 현재 publisher는 없으므로 기본적으로 정지한다.

## Safety / 소유권

- STOP/FINISH, 모드 미수신/오류/1.5초 만료: speed=0, steering=0,
  emergency_stop=true를 즉시 및 20 Hz 발행한다. PWM override는 사용하지 않는다.
- GPS/OBSTACLE/REVERSE에서 readiness 미수신/만료: 같은 정지를 유지한다.
  controller 부재 시 lane으로 자동 fall-through하지 않는다.
- LANE에서 유효한 모드 heartbeat가 유지되면 safety는 `/cmd/mission` 발행을
  중단한다. 기존 STOP 후보는 arbiter의 기존 timeout(기본 0.5초) 후 만료되어
  `/cmd/lane`이 선택될 수 있다. mode 통신 만료 시에는 다시 정지한다.
- controller 인계 시 기존 STOP 후보가 남아 있을 수 있다. controller는 자신의
  모드/readiness가 유효할 때만 명령을 발행하고 준비 상실/STOP/FINISH에서 발행을
  중단해야 한다. readiness 상실 때 움직이는 명령을 계속 발행하는 controller와
  동일 토픽에서 경쟁하는 상황은 이 계약으로 금지한다.
- FINISH는 safety에서 latch한다. 이후 LANE 등 늦은 모드 메시지로 해제되지 않는다.
  manager도 terminal 상태여서 자동 재출발하지 않는다.
- 기존 arbiter priority는 변경하지 않는다. 상위 수동/emergency 권한을 차단하는
  기능은 아니며 safety 프로세스 자체 종료를 감시하는 외부 watchdog은 없다.

진단은 target/미션/phase/mode 변화에 발행한다:

```text
WAYPOINT number=31 id=S_CURVE_GUIDE_1 type=mission_guide hard_point=false mission=S_CURVE
MISSION state=S_CURVE control_mode=OBSTACLE phase=GUIDE
```

각 skipped/reached 이벤트도 기록한다. active는 코스 미션 수행 여부이며 FINISH는
active=false, completed=true다. 일반 NORMAL_DRIVE는 completed=false다.

## 호환성과 검증

기존 WP01~WP11 YAML은 별도 엄격 parser로 읽고 기존 GPS entry → status 완료,
WP10 SIGNAL_CAR → LANE_CHANGE, WP11 FINISH 순서를 보존한다.
fixture는 `test/fixtures/waypoints_legacy.yaml`이다. 독립 ControlMode의 S_CURVE와
CHILD_DUMMY 매핑은 새 요구에 따라 OBSTACLE로 갱신된다.

`mission_test_runner_node.py`는 변경하지 않는다. GPS 없이 선택한 미션을 발행하고
일치하는 완료 후 completed 상태를 유지한다. manager와 동시에 실행하면 안 된다.
runner는 control_mode를 발행하지 않으며 이 manager/safety launch와 별개다.

오프라인 검증은 ROS Python 메시지 환경만 로드하고 pytest의 mock Node를 사용한다.
DDS, ROS node spin, 카메라, 차량, enable_drive=true를 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/ros2_ws/src/fma_mission:$PYTHONPATH" \
  python3 -m pytest -q -p no:cacheprovider ros2_ws/src/fma_mission/test
```

순수 진행 테스트는 정확한 90개 좌표/이름, null 금지, soft skip, hard 순서,
모든 미션별 세 가지 완료 정책, 37번 원자적 전환, phase, terminal FINISH를 검증한다.
mock 테스트는 manager 발행, feedback 만료, safety readiness/timeout/LANE 해제/
FINISH latch, legacy 11 waypoint와 기존 runner 동작을 검증한다.
