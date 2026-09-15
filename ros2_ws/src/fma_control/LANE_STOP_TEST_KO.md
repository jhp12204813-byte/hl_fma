# C920 첫 저속 정지선 테스트

`lane_stop_test.launch.py`는 이번 현장 테스트 전용이다. 기본 enable_drive=false이며
제어 publisher도, arbiter/controller/bridge도 시작하지 않는다. 명시적인 true에서만
기존 /cmd/lane → arbiter → /cmd/final → vehicle_controller → stm32_bridge를 사용한다.
유효 lane의 요청은 drive_pwm 파라미터의 CCR(기본 40, 약 5% duty)이며 새로운 속도 protocol은 없다.
실제 속도를 m/s로 보장하는 제어는 아니다.

카메라는 한 번만 열며 같은 frame의 make_mask 결과로 PaperLaneTracker와
StopLineDetector/StopLineTracker를 실행한다. 기존 calculate_command의 adaptive
lookahead와 gain, steering calibration을 그대로 사용한다.
정지선은 기존 stop_tracked=true(기본 confirm_frames=3)이며 현재 검출의 near edge가
stop_trigger_distance_m 이하일 때 latch한다. 기본값은 0.60 m다.
추가 confidence threshold나 감속은 없다. 후보 첫 프레임은 확정 검출이 아니다.
STOP CENTER/NEAR EDGE는 기존 detector의 선택된 candidate 거리/두께에서 계산한다.

- 시작은 ACQUIRING/STOP. 처음 유효 lane을 얻으면 DRIVING.
- 정지선 확정 + near edge 거리 조건 충족: STOP_LINE. 이후 검출 소실/차선 회복과 관계없이 STOP만 발행.
- Lane invalid/no fresh frame은 즉시 STOP 및 재획득. 실제 카메라 오류는 INVALID latch/STOP.
- 노드 재시작 외에 latch 해제 기능은 없다.
- Ctrl+C/종료는 기존 STOP 3회 cleanup과 arbiter/controller/firmware timeout을 사용한다.
  연결이 끊기면 소프트웨어가 STOP 전달을 보장할 수는 없다.
- 기존 manual/emergency 우선순위를 변경하지 않는다. Latch는 이 node의 lane 요청에
  적용되며 manual 등 다른 우선순위 입력을 막는 전역 emergency latch는 아니다.
  다른 주행/diagnostic 명령 writer 및 카메라 사용 도구를 동시에 실행하지 않는다.

일반 lane_follow의 DRIVE 3.50 m 제한은 유지한다. 이 전용 테스트 실행만
lane_min_width_m=3.0을 명시해 사용할 수 있으며 production tracker 기본값 3.50 m는
그대로다. Param은 startup-only이며 실행 중 변경할 수 없다.

```bash
cd /home/sohyun/fma_autonomous_vehicle/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to fma_control
source install/setup.bash
ros2 launch fma_control lane_stop_test.launch.py enable_drive:=false device:=/dev/video14 lane_min_width_m:=3.0
```

명시적 실제 구동:

```bash
ros2 launch fma_control lane_stop_test.launch.py enable_drive:=true device:=/dev/video14 lane_min_width_m:=3.0
```

기본 STM32 port는 검증된 ST-LINK by-id이며 port:=...로 지정 가능하다.
1초 로그의 STATE/lane_valid/오차/조향/stop_detected/stop_confirmed/거리/confidence/
drive_pwm/FPS로 확인한다. DRY-RUN의 DRIVING은 계산 상태이며 실제 PWM 발행은 0이다.
자동 테스트는 모의 camera/serial만 사용하며 실차 테스트는 자동 실행하지 않는다.

## Control window / independent stop worker

production calculate_command의 control window는 1.8..3.0 m다. field PAIR는
pair_control_max_m(기본 3.5 m)를 상한으로 사용한다. Desired는 기본 2.0 m이고
관측 구간 내부 margin과 이 window의 교집합에서만 target을 선택한다.
4..6 m만 관측되면 INVALID이며 publish 직전에도 used_lookahead 범위를 재검사한다.

Lane thread는 매 입력 frame의 mask/tracker/조향을 계산한다. Stop worker에는 같은
frame에서 얻은 white BEV와 lane 결과를 최신 1개 mailbox로 전달한다. 별도 camera는
없고 stop detect/update는 최대 5 Hz다. 기존 3회 confirm이므로 확인까지 걸리는
실시간 시간은 lane FPS와 독립적이며 대략 0.4..0.6초 이상 걸릴 수 있다.
검출 확정 및 near edge 거리 조건 충족 시 latch하며 그 다음 제어 tick부터 STOP 요청을 유지한다.

Stop snapshot에는 frame timestamp가 포함된다. 0.5초 이상 오래된 결과는 거리/판정에
사용하지 않으며, 처리가 늦게 끝난 결과도 폐기한다. Stop worker 오류는 STOP latch다. Stop snapshot stale은 즉시 STOP 후 재획득한다. Startup에서는 fresh stop snapshot을 기다린다.
5개의 연속 fresh valid lane frame 이후에만 PWM 40을 허용한다. 반복 timer tick은
프레임 수에 포함하지 않으며 invalid/stale이면 acquisition 횟수를 초기화한다.
Lane freshness 제한 0.25초, perception threshold, STOP latch, watchdog는 유지한다.
실제 FPS 개선은 실차 실행 없이 보장할 수 없으며 현장 재검증이 필요하다.

## Stop trigger distance

`stop_trigger_distance_m:=0.60`을 launch에서 지정할 수 있다. 유한한 양수만 허용하고
실행 중 변경은 금지한다. 먼 정지선의 검출/거리/confidence는 계속 표시하며, 다른
안전 조건을 모두 만족하면 PWM 40 접근을 계속한다. 오래된 결과나 near edge를
계산할 수 없는 결과는 거리 trigger에 사용하지 않는다. stop_triggered 로그는
latched 상태를 의미하므로 검출이 사라져도 true를 유지한다.
Lane invalid/stale은 즉시 STOP 후 재획득, camera failure는 영구 STOP이며 emergency 우선순위는 동일하다.
이 값은 STOP 요청 시점의 거리이며 실제 제동 후 정지 거리를 보장하지 않는다.

## PAIR 중심 제어 / tracker temporary center

유효한 PAIR_VALID + pair_quality.valid의 실제 center를 우선 추종한다.
제어에서 고정 lane width로 center를 생성하지 않는다.
`allow_single_side_test` 기본값은 false로 production의 pair-only 정책을 유지한다.
field-test에서 true로 지정하면 기존 tracker가 제공한 TEMPORARY_CENTER만 추가 허용한다.
현재 visible boundary와 일치하는 LEFT_ONLY_OFFSET/RIGHT_ONLY_OFFSET이 필요하다.
tracker의 이전 유효 pair 기반 폭, identity hysteresis, 프레임 만료 조건은 그대로다.

cold-start의 단독 boundary, 만료된 temporary center, PAIR_WEAK, control window 밖
PAIR에서는 fixed-width single-side 주행을 하지 않는다. PAIR/TEMPORARY_CENTER가
불가능하면 INVALID + PWM0이며 다시 5개의 NEW fresh safe frame을 획득해야 한다.
실험용 single-side offset memory와 짧은 dropout 주행 상태 및 해당 parameter/log는 제거했다.

PAIR control window는 production에서 1.8..3.0 m, field-test에서
1.8..pair_control_max_m(기본 3.5 m)다. 기존 TEMPORARY_CENTER의
single_side_control_max_m 기본값 및 hard maximum은 3.30 m이며 관측 margin은 유지한다.
발행 직전에도 source와 lookahead를 검사한다. 일반 lane invalid/stale은 즉시 STOP한다.
STOP_APPROACH 진입 후에는 아래 기존 접근 정책을 적용한다.

`drive_pwm`는 0..799 정수(기본 40)이며 field-test에만 적용된다. 120은 CCR 기준
15% duty 설정이며 실제 차량 속도를 보장하지 않는다. `stop_min_thickness_m` 기본값은
기존 0.04 m이고, detector/calibration은 변경하지 않는다. 현재 설정을 그대로 사용한다.

로그에는 control_source=PAIR/TEMPORARY_CENTER/NONE, reason, 관측 거리,
safe_streak, STATE, stop freshness/latch와 PWM을 기록한다.
Emergency의 명시적 SET/CLEAR와 arbiter 우선순위, system fault 및 STOP_LINE latch를 유지한다.

DRY-RUN에서 기존 temporary center를 허용하는 명령:

```bash
ros2 launch fma_control lane_stop_test.launch.py enable_drive:=false \
  device:=/dev/video14 allow_single_side_test:=true single_side_control_max_m:=3.30
```

## STOP_APPROACH (field only)

While already DRIVING with a safe lane, a fresh detected + confirmed stop within
2.0 m and confidence >=0.80 starts STOP_APPROACH. It uses stop_approach_pwm (default
40, integer 0..63), capped at the configured normal drive_pwm. Last valid steering
is held and clamped to +/-0.10 rad; fresh valid lane steering updates it immediately.
Lane dropout is tolerated only after entry. Normal driving still stops for invalid lane.

Camera frames must remain fresh (<0.25 s), stop snapshots fresh (<0.5 s), confirmation
and finite near-edge distance valid. Missing current distance stops conservatively;
held tracker identity alone does not authorize dead reckoning. Distance increase over
the unchanged tracker's max_distance_increase_m (0.35 m) aborts approach. Timeout
stop_approach_timeout_sec defaults to 3.0 s. Any approach abort stays stopped until
node restart. Actual worker faults and stop-line latch remain separate. Approach
near edge <= stop_trigger_distance_m latches STOP_LINE forever; the field command
uses 0.40 m. No detector/tracker thresholds change. Existing manual/emergency priority
is unchanged; no direct serial commands are added. Stop transmission still depends
on the existing ROS/bridge scheduling and hardware watchdog.

Logs include approach active/PWM, held steering, stop age and elapsed seconds.
No physical motion/camera is exercised by software tests. A commanded stop distance
is not a guarantee of final physical position.


## Field PAIR control maximum

`pair_control_max_m`은 startup-only parameter다. production은 3.0 m로 고정하고
field-test만 상한을 변경할 수 있다. lane_stop_test의 기본값은 3.5 m이며 유한한
1.8 이상 4.0 미만 값만 허용한다. launch와 node가 시작 전에 검증하고, 계산과 PWM
발행 직전에도 PAIR의 상한을 적용한다. observed_min > pair_control_max_m이면 차단한다.
TEMPORARY_CENTER의 기존 3.30 m 상한, lane width 정책, detector/calibration,
5 fresh frame acquisition 및 STOP_APPROACH/STOP_LINE은 유지한다.

기존 관측 0.10 m margin도 유지한다. 따라서 observed_min=3.42~3.43 m이면
필요한 target은 최소 3.52~3.53 m라서 기본 상한 3.5 m에서는 여전히
outside_control_window다. observed_min <= 상한은 필요조건이며 다른 유효성 조건을
대체하지 않는다. 상태 로그에 pair_control_max_m을 출력한다.

## Per-frame pair perception diagnostic

`PAIR_PERCEPTION` logs once per processed PAIR_VALID/PAIR_WEAK frame, including
frames rejected by lane control. This log bypasses the 1-second status throttle.
It reads the original yellow BEV mask before tracker horizontal-run suppression,
and the final selected left/right fits after any accepted joint refit.

For each side: observed_min_m/observed_max_m (clipped to BEV), y_max_row, points,
residual_px and inliers_1_3m. Common observed bounds are the intersection of the
side ranges. pair_near_limit_side is LEFT, RIGHT or BOTH according to the larger
side observed_min; NONE means the side ranges are unavailable. An empty common
intersection reports N/A for the common distances.

raw_mask_pixels_1_3m counts nonzero pixels across the entire raw BEV width, inclusive
of both distance endpoints (rows 300..500 with the current calibration).
LEFT/RIGHT_inliers_1_3m count actual final `_inlier_y` entries in that same interval;
selected_inliers_1_3m is their sum. It is not a count of pixels near an extrapolated
curve. Missing inlier telemetry is N/A, not zero. A positive raw count with zero
selected inliers indicates that those near pixels were not retained in the selected
fits; the raw pixels may also belong to other markings. No thresholds, calibration,
fit selection, control windows or source priorities are changed by this diagnostic.

### LEFT/RIGHT raw → search candidate → selected inlier

PAIR_PERCEPTION additionally reports LEFT/RIGHT_raw_mask_pixels_1_3m,
LEFT/RIGHT_candidate_points_1_3m, and LEFT/RIGHT_selected_inliers_1_3m.
Raw pixels split at the BEV vehicle origin (x<0 LEFT, x>=0 RIGHT); this is a spatial
split, not a claim that every raw marking belongs to that boundary.

Candidate points are the exact search output passed into RANSAC, after existing
horizontal-run suppression and previous/sliding-window search, before RANSAC/inlier
filtering. They follow the finally chosen boundary fit, not the union of all trial
fits. An accepted joint refit preserves its original RANSAC candidate lineage.
Unavailable candidate telemetry is N/A, never an inferred zero. Comparing raw with
candidate diagnoses combined suppression/search loss (B); comparing candidate with
selected diagnoses RANSAC/joint-fit loss (C). Very low raw counts suggest the earlier
mask/FOV/warp stages (A). Counts alone do not prove which particular sub-filter failed.

Candidate capture is enabled only for field-test and the live diagnostic tool;
production tracker defaults leave it off. The live tool colors RIGHT 1..3m raw pixels
blue, candidate halos cyan, rejected candidate cores red, and selected cores green.
This overlay changes only the displayed image; no mask, fit or control input is edited.
