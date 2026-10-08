# Encoder Odom 바닥 주행 보정 절차

이 절차는 정밀 `encoder_odom_scale` 보정이 필요할 때 사용한다.
각 장비에서 공중 방향·정지 검사를 먼저 통과하고, 바닥에서 독립적인 거리 기준으로
반복 측정한다. 각 trial마다 즉시 주 전원 차단 담당자와 새 승인이 필요하다.

`encoder_position_ticks_per_revolution:=4096.0`은 raw position register의 wrap modulus로 유지한다. 실제 이동거리 보정은 양의 유한 실수 `encoder_odom_scale`에만 적용한다.

translation 보정과 yaw 검증은 서로 다른 trial 집합으로 계산한다. raw position modulus는 바꾸지 않는다.

## 전제와 중지 조건

- 첫 저속 바닥 trial에는 공중 `(1.0, 652)` 단계의 여섯 방향·정지·토크 해제가 확인됐다. 상한을 더 높여 운용하려면 `(2.0, 1304)`, `(2.3009711818284617, 1500)` 단계를 순서대로 별도 검증한다.
- 현장 감독자, 비상 물리 전원 차단, 충분한 제동 거리가 준비됐다.
- 첫 바닥 시험은 `(1.0, 652)` override로 다시 낮춘다.
- 시작 전에 power가 disarmed인 `pre-arm /motor_ready false`와 zero `/cmd_vel`을 확인한다.
- 모든 non-zero 명령은 `lekiwi_safe_cmd_vel --yes-i-confirm-motion`으로만 보낸다.

첫 바닥 trial의 body 상한 `0.03m/s`와 `0.1rad/s`에서 가능한 wheel 각속도는
출고 장비의 바퀴 반지름, 유효 중심거리와 운동학 오프셋으로 계산한다.
계산값이 `1.0rad/s`·`652 ticks/s` 제한 안에 드는지 확인한 뒤 진행한다.
상한을 늘린 운용의 공중 검증과 첫 저속 바닥 안전 시험을 구분한다.

`Airborne power/readiness transition already accepted`가 floor 진입 전제다. `--arm-motors` service success만 readiness 관찰로 재해석하지 않는다. backend는 fresh encoder 전에는 zero만 허용하고, Airborne gate에서 그 `/motor_ready false -> true` 전환과 timeout/fault 동작을 이미 별도로 관찰했어야 한다.

예상 밖 방향, 미끄럼/충돌 위험, `/motor_ready false`, serial/encoder 오류, command timeout 뒤 움직임, zero/torque-off 실패가 보이면 즉시 `/motor_power false`와 물리 전원 차단을 수행하고 중지한다. 실패한 trial은 평균에 포함하지 않고 원인을 해결하기 전에는 다시 움직이지 않는다.

## Bringup과 동작 전 검사

실제 serial 경로는 셸 변수로만 사용하고 기록 파일이나 커밋에 복사하지 않는다.

```bash
cd "$HOME/lekiwi_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix lekiwi_node  # 이 workspace의 실행 파일인지 확인

MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_FEETECH_PORT"
ros2 launch lekiwi_bringup robot.launch.py \
  mode:=base \
  use_lidar:=false \
  use_imu:=false \
  motor_backend:=feetech \
  serial_port:="$MOTOR_PORT" \
  odom_source:=encoder \
  enable_motor_write:=true \
  torque_enable:=false \
  max_linear_x:=0.03 \
  max_linear_y:=0.03 \
  max_linear_speed:=0.03 \
  max_angular_z:=0.1 \
  max_wheel_speed:=1.0 \
  speed_tick_limit:=652 \
  encoder_position_ticks_per_revolution:=4096.0 \
  encoder_odom_scale:=1.0
```

non-zero 명령 전에는 다음 상태를 확인한다.

```bash
ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /odom --once
ros2 topic echo /wheel_joint_states --once
```

## Trial 구성

첫 바닥 동작은 거리 보정 표본으로 쓰지 않는 `safety` trial 한 번이다. 평평한 바닥에서
로봇을 전방 기준선에 놓고 모든 방향으로 최소 1m의 빈 공간을 확보한다. 사람은
로봇에서 손을 떼고 주 전원 스위치를 즉시 끌 수 있어야 한다. 전진 `0.03m/s`를
1초만 명령해 실물 이동 방향, 미끄럼·기울어짐, 명령 종료 뒤 정지, 토크 해제를
확인한다. 방향이 예상과 다르거나 종료 확인이 실패하면 보정 trial로 진행하지 않는다.

scale을 만드는 **calibration set: forward translation only**로 고정한다. 동일한 직선, 목표 거리, 바닥 조건에서 forward trial을 `n_forward >=3`회 먼저 수집한다. 이 단계에서는 backward/left/right/yaw 값을 scale 계산에 섞지 않는다. candidate 적용 뒤 validation에서 `translation trials per direction >=3`과 `yaw trials per direction >=3`을 별도로 수행한다. trial마다 같은 외부 기준으로 시작/종료를 측정하고 `/reset_odometry` 뒤 odom 변화량을 기록한다.

각 trial은 독립된 power/readiness/reset/motion/disable/physical-stop 경계를 가져야 한다. 이전 trial의 승인, power 상태, `/motor_ready true`, trap 또는 odom reset을 다음 trial로 넘기지 않는다. `lekiwi_safe_cmd_vel --arm-motors`는 ROS 연결과 `/motor_ready=false`를 토크 활성화 전에 확인하고, 토크 활성화 뒤 `/motor_ready=true`를 관찰해야만 명령을 발행한다. `--yes-i-confirm-motion` is a CLI acknowledgement, not authorization. 실제 시작 전에 `ros2 pkg prefix lekiwi_node`가 현재 workspace의 설치본을 가리키는지 확인한다.

먼저 `export FLOOR_PHASE=safety`로 1초 전진 한 번을 실행한다. 물리 정지와 토크 해제까지 확인한 후 `FLOOR_PHASE=calibration`으로 forward 5초 trial 3회를 각각 새로 승인받아 실행한다. 후보 scale 적용과 재launch 뒤 `FLOOR_PHASE=validation`으로 forward/backward/left/right/ccw/cw 5초 trial을 각각 3회 실행한다. 긴 구간에서는 1m 빈 공간과 외부 거리·각도 측정 기준을 다시 확인한다. 각 trial 직전에 주변, 제동 거리, 비상 물리 전원 차단 담당자, `(1.0, 652)`, disarmed `/motor_ready false`, zero `/cmd_vel`을 다시 확인한다. prompt가 요구하는 현재 phase/direction/trial 문자열을 정확히 새로 입력하는 것이 **STOP: fresh user confirmation required immediately before first non-zero command**이며, 미리 입력한 값이나 이전 trial 승인은 사용할 수 없다.

```bash
# FLOOR_TRIAL_MATRIX_START
set -Eeuo pipefail

: "${FLOOR_PHASE:?export FLOOR_PHASE as safety, calibration or validation}"
TRIALS_PER_DIRECTION=3
SAFETY_DIRECTIONS=("forward")
CALIBRATION_DIRECTIONS=("forward")
VALIDATION_DIRECTIONS=("forward" "backward" "left" "right" "ccw" "cw")

best_effort_disable_floor_trial() {
  (
    set +e
    timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}" || true
  )
}
interrupt_floor_trial() {
  exit 130
}

run_floor_trial() {
  # RUN_FLOOR_TRIAL_BEGIN
  local PHASE="$1"
  local DIRECTION="$2"
  local TRIAL_NUMBER="$3"
  local EXPECTED_CONFIRMATION FRESH_CONFIRMATION PHYSICAL_STOP_CONFIRMATION
  local PRE_READY_RESPONSE RESET_RESPONSE
  local DISABLE_RESPONSE DISARMED_RESPONSE
  local ODOM_RECORD MOTION_DURATION
  local -a COMMAND_ARGS

  case "$DIRECTION" in
    forward)  COMMAND_ARGS=(--linear-x 0.03) ;;
    backward) COMMAND_ARGS=(--linear-x -0.03) ;;
    left)     COMMAND_ARGS=(--linear-y 0.03) ;;
    right)    COMMAND_ARGS=(--linear-y -0.03) ;;
    ccw)      COMMAND_ARGS=(--angular-z 0.1) ;;
    cw)       COMMAND_ARGS=(--angular-z -0.1) ;;
    *) return 2 ;;
  esac

  PRE_READY_RESPONSE="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || exit 1
  printf '%s\n' "$PRE_READY_RESPONSE"
  grep -Eq '^data: false$' <<<"$PRE_READY_RESPONSE" || exit 1

  EXPECTED_CONFIRMATION="RUN ${PHASE} ${DIRECTION} TRIAL ${TRIAL_NUMBER}"
  read -r -p "Fresh motion confirmation: type exactly '${EXPECTED_CONFIRMATION}': " FRESH_CONFIRMATION
  test "$FRESH_CONFIRMATION" = "$EXPECTED_CONFIRMATION"

  trap best_effort_disable_floor_trial EXIT
  trap interrupt_floor_trial INT TERM

  RESET_RESPONSE="$(timeout 5 ros2 service call /reset_odometry std_srvs/srv/Trigger "{}")" || exit 1
  printf '%s\n' "$RESET_RESPONSE"
  grep -q "success: true" <<<"$RESET_RESPONSE" || exit 1

  if test "$PHASE" = safety; then MOTION_DURATION=1.0; else MOTION_DURATION=5.0; fi
  timeout --signal=INT --kill-after=2 30 ros2 run lekiwi_teleop lekiwi_safe_cmd_vel \
    "${COMMAND_ARGS[@]}" \
    --duration "$MOTION_DURATION" \
    --max-duration "$MOTION_DURATION" \
    --arm-motors \
    --yes-i-confirm-motion

  DISABLE_RESPONSE="$(timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}")" || exit 1
  printf '%s\n' "$DISABLE_RESPONSE"
  grep -q "success: true" <<<"$DISABLE_RESPONSE" || exit 1

  DISARMED_RESPONSE="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || exit 1
  printf '%s\n' "$DISARMED_RESPONSE"
  grep -Eq '^data: false$' <<<"$DISARMED_RESPONSE" || exit 1

  read -r -p "Physical stop confirmation: verify wheels stopped and torque is off, then type STOPPED: " PHYSICAL_STOP_CONFIRMATION
  test "$PHYSICAL_STOP_CONFIRMATION" = "STOPPED"

  ODOM_RECORD="/tmp/lekiwi_floor_${PHASE}_${DIRECTION}_${TRIAL_NUMBER}_odom.yaml"
  timeout 5 ros2 topic echo /odom --once >"$ODOM_RECORD"
  printf 'record only now: %s\n' "$ODOM_RECORD"
  trap - EXIT INT TERM
  # RUN_FLOOR_TRIAL_END
}

case "$FLOOR_PHASE" in
  safety) DIRECTIONS=("${SAFETY_DIRECTIONS[@]}"); TRIALS_PER_DIRECTION=1 ;;
  calibration) DIRECTIONS=("${CALIBRATION_DIRECTIONS[@]}") ;;
  validation) DIRECTIONS=("${VALIDATION_DIRECTIONS[@]}") ;;
  *) printf 'FLOOR_PHASE must be safety, calibration or validation\n' >&2; exit 2 ;;
esac

for DIRECTION in "${DIRECTIONS[@]}"; do
  for TRIAL_NUMBER in $(seq 1 "$TRIALS_PER_DIRECTION"); do
    run_floor_trial "$FLOOR_PHASE" "$DIRECTION" "$TRIAL_NUMBER"
  done
done
# FLOOR_TRIAL_MATRIX_END
```

power-on service success는 readiness가 아니다. 명령 도구는 각 trial에서 별도 `/motor_ready true`를 관찰해야 하며, reset은 성공 응답 뒤 단 하나의 bounded command 앞에 있어야 한다. disable service와 `/motor_ready false`도 각각 5초로 제한한다. trap의 disable response는 물리 정지 증거가 아니므로 `STOPPED`를 입력하기 전에 wheel 정지와 torque-off를 직접 확인하고, 확인할 수 없으면 물리 전원을 차단한다. 이 확인 전에는 odom/실측값을 기록하거나 다음 trial로 진행하지 않는다.

calibration set에서는 forward 결과만 같은 외부 목표 거리로 측정한다. candidate 적용 전에는 `--linear-y`, `--angular-z`, reverse 결과로 scale을 만들지 않는다. 직접 지속 publish 명령은 사용하지 않는다.

## 오차, 반복 편차와 scale 계산

각 forward calibration trial의 시작/종료 odom으로 거리와 ratio를 계산한다.

```text
d_odom = hypot(x_end - x_start, y_end - y_start)
translation_error_pct = 100 * abs(d_odom - d_measured) / d_measured
translation_ratio_i = d_measured_i / d_odom_i
forward_ratio_i = d_measured_i / d_odom_i
```

yaw trial은 quaternion에서 얻은 yaw의 최단 각도 차이를 사용한다. `yaw_measured`와 `yaw_odom`은 같은 방향의 signed 회전량이며 0이 아니어야 한다.

```text
yaw_delta_error = atan2(sin(yaw_odom - yaw_measured), cos(yaw_odom - yaw_measured))
yaw_error_pct = 100 * abs(yaw_delta_error) / abs(yaw_measured)
yaw_ratio_i = abs(yaw_measured_i) / abs(yaw_odom_i)
```

forward calibration set만 다음 순서로 집계한다.

```text
forward_ratio_mean = sum(forward_ratio_i) / n_forward
forward_repeat_variation_pct = 100 * (max(forward_ratio_i) - min(forward_ratio_i)) / forward_ratio_mean
recommended_scale = current_scale * forward_ratio_mean
```

forward trial이 3개 미만이거나, `d_measured <=0`, `d_odom <=0`, 명령/odom 부호 불일치, 미끄럼/충돌, `forward_repeat_variation_pct >3%` 중 하나라도 있으면 **abort without a recommended scale** 한다. 실패 trial을 조용히 빼고 남은 값으로 계산하지 않는다. 모두 통과할 때 위 식에서 **exactly one recommended scale**을 만들고 acceptance record에 calibration input 전체와 함께 기록한다.

## Scale 후보 재측정

계산된 단일 candidate를 `REPLACE_WITH_RECOMMENDED_SCALE`에 넣어 launch override로 적용한다. `encoder_position_ticks_per_revolution`은 `4096.0`에서 바꾸지 않는다. 재launch 뒤 첫 non-zero validation trial에도 위 STOP과 fresh confirmation을 다시 적용한다.

```bash
ros2 launch lekiwi_bringup robot.launch.py \
  mode:=base \
  use_lidar:=false \
  use_imu:=false \
  motor_backend:=feetech \
  serial_port:="$MOTOR_PORT" \
  odom_source:=encoder \
  enable_motor_write:=true \
  torque_enable:=false \
  max_linear_x:=0.03 \
  max_linear_y:=0.03 \
  max_linear_speed:=0.03 \
  max_angular_z:=0.1 \
  max_wheel_speed:=1.0 \
  speed_tick_limit:=652 \
  encoder_position_ticks_per_revolution:=4096.0 \
  encoder_odom_scale:=REPLACE_WITH_RECOMMENDED_SCALE
```

candidate 적용 뒤 forward/backward/left/right translation과 양방향 yaw를 각각 `>=3`회 검증한다. 각 방향에서 ratio 평균과 repeat variation은 validation 통계로만 기록하고 **do not compute a second scale from backward/left/right/yaw validation**. validation 방향 평균이 서로 3%보다 다르거나 아래 error/repeat 수락값을 하나라도 넘으면 candidate를 reject하고 기본 YAML을 바꾸지 않는다. 여러 방향 값을 다시 평균내거나 두 번째 scale을 만들지 말고 wheel radius, base radius, wheel별 편차와 미끄럼을 별도 설계한다.

candidate launch가 안정화된 뒤 `export FLOOR_PHASE=validation`을 설정하고 위 `FLOOR_TRIAL_MATRIX_START` block 전체를 다시 실행한다. wrapper 밖에서 candidate-validation motion을 추가하지 않는다.

```text
validation_direction_mean_d = sum(translation_ratio_i for direction d) / n_d
translation_direction_spread_pct = 100 * (max(validation_direction_mean_d) - min(validation_direction_mean_d)) / mean(validation_direction_mean_d)
```

`d`는 forward/backward/left/right이고 각 `n_d >=3`이다. `translation_direction_spread_pct <=3%`여야 하며, yaw ratio는 이 translation spread나 scale에 섞지 않는다.

## 수락 기준

- 각 translation 방향의 `translation_error_pct <=5%`
- 각 yaw 방향의 `yaw_error_pct <=5%`
- translation과 yaw 각 방향 집합의 `repeat_variation_pct <=3%`
- 부호가 명령 방향과 일치하고 정지 뒤 누적 drift가 계속 증가하지 않음
- scale 적용 뒤 raw wrap, `/motor_ready`, timeout/fault/recovery 계약이 유지됨

하나의 scale로 전후, 횡이동, 회전 기준을 함께 만족하지 못하면 그 candidate는 실패다. 임의로 trial을 제외하거나 wheel별 보정을 추가하지 않고 wheel radius, base radius, 바닥 미끄럼과 wheel별 scale이 필요한지 별도 설계한다.

## Acceptance record

repo 밖 기록에 commit, current/recommended scale, 각 방향별 odom/실측값, 오차, 반복 편차, 중지 사유를 남긴다. map, rosbag, `build/`, `install/`, `log/`, 실제 serial ID, 개인 절대 경로는 커밋하지 않는다. 실제 관찰 전에는 README나 기본 YAML의 보정값을 변경하지 않는다.
