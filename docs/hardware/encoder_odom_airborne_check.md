# Encoder Odom 공중 검증 절차

이 절차는 오프라인 회귀가 끝난 뒤 LeKiwi base를 안정적으로 지지해 모든 바퀴가
바닥에서 떨어진 상태에서만 수행한다. 단계별 속도 상한·방향·정지·토크 해제를
각 장비에서 확인하고 기록하기 전까지 아래 체크박스를 완료로 표시하지 않는다.

현재 소스의 Feetech backend에는 STS/SMS speed mode 설정, zero/torque packet, sync goal speed packet, present position/speed read와 `/motor_ready` fault latch가 있다. fake serial 기반 오프라인 테스트는 packet byte와 실패 정리를 검증하지만, 실제 bus에서의 현재 non-zero 동작을 증명하지 않는다.

## zero-speed 검증 게이트

zero-speed 수락값은 바퀴별 절댓값 `<=0.05 rad/s`다. 현재 설치본에서
무이동·토크 활성화 뒤 60초 drift를 장비마다 새로 확인한다.

## 시작 게이트

- [ ] 최신 오프라인 build/test가 `0 errors`, `0 failures`다.
- [ ] 로봇을 지지대에 고정했고 세 바퀴가 바닥과 장애물에서 떨어져 있다.
- [ ] 비상 물리 전원 차단 수단과 현장 감독자가 준비됐다.
- [ ] 출고 장비의 세 base wheel ID가 다른 장치 ID와 충돌하지 않는다.
- [ ] 실제 serial 경로는 화면에서만 확인하고 문서나 커밋에 기록하지 않는다.
- [ ] 첫 non-zero 명령 직전에 사용자가 다시 승인했다.

하나라도 만족하지 않으면 launch, torque enable, 공중 구동을 시작하지 않는다.

## 동작 없는 사전 점검

먼저 기본 안전 설정과 launch 인자를 소스에서 확인한다. 이 명령은 장치를 열거나 motor command를 보내지 않는다.

```bash
rg -n "motor_backend: mock|enable_motor_write: false|torque_enable: false" \
  lekiwi_bringup/param/base.yaml lekiwi_node/param/base_controller.yaml
ros2 launch lekiwi_bringup robot.launch.py --show-args
```

현장에서는 `torque_enable:=false`로 hardware bringup을 시작한 뒤, non-zero 명령 전에 아래 읽기 전용 검사를 수행한다. `$MOTOR_PORT`의 실제 값은 acceptance record에 복사하지 않는다.

```bash
ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 topic info /cmd_vel --verbose
```

`/motor_ready`가 `false`인 동안 non-zero motion을 시도하지 않는다. serial open/read 오류, 잘못된 wheel ID, 예상 밖 encoder 변화, zero가 아닌 `/cmd_vel`, 또는 service `success: false`가 보이면 즉시 `/motor_power false`와 물리 전원 차단을 수행하고 중지한다.

## Fresh-approved Airborne motor power-on

아래 전환은 안정적으로 지지된 Airborne gate에서 zero `/cmd_vel`, 비상 물리 전원 차단 담당자, 실제 base serial과 wheel ID를 다시 확인한 뒤에만 수행한다. 이 문서의 이어지는 shell block들은 같은 terminal에서 순서대로 실행한다.

**STOP: fresh user confirmation required immediately before Airborne motor power-on**

사용자의 새 승인을 받은 뒤 zero 상태에서만 다음 guarded block을 실행한다. power service timeout/실패, 참인 `success` 응답 부재, 별도로 관찰한 `/motor_ready true` 부재는 모두 disable 시도 후 즉시 abort다.

```bash
# AIRBORNE_POWER_ON_START
set -Eeuo pipefail

motor_power_succeeded() {
  local response
  local -a true_matches=()
  response="$(cat)"

  if grep -Eq '(^|[^[:alnum:]_])success(=False|: false)([^[:alnum:]_]|$)' <<<"$response"; then
    return 1
  fi
  if grep -Eq '(^|[^[:alnum:]_])success(=true|: True)([^[:alnum:]_]|$)' <<<"$response"; then
    return 1
  fi
  mapfile -t true_matches < <(grep -Eo 'success(=True|: true)' <<<"$response" || true)
  test "${#true_matches[@]}" -eq 1 || return 1
  grep -Eq '(^|[^[:alnum:]_])success(=True|: true)([^[:alnum:]_]|$)' <<<"$response"
}
disable_airborne_motors() {
  timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}" || true
}
interrupt_airborne_check() {
  exit 130
}
trap disable_airborne_motors EXIT
trap interrupt_airborne_check INT TERM

POWER_RESPONSE="$(timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}")" || exit 1
printf '%s\n' "$POWER_RESPONSE"
motor_power_succeeded <<<"$POWER_RESPONSE" || exit 1

READY_RESPONSE="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || exit 1
printf '%s\n' "$READY_RESPONSE"
grep -Eq '^data: true$' <<<"$READY_RESPONSE" || exit 1
```

`motor power service success does not imply /motor_ready true`; service response와 transient-local readiness sample은 서로 독립적으로 관찰하고 기록한다. 이 block이 abort하거나 wheel이 예상 밖으로 움직이면 software cleanup 응답을 기다리지 말고 물리 전원을 차단한다. installed EXIT trap은 이어지는 60초 drift 중 shell이 종료되더라도 bounded disable을 시도한다. 이 최초 power-on은 drift 측정만을 위한 것이며 뒤의 어느 motion trial도 승인하거나 arm하지 않는다.

## 60초 정지 drift

이 항목의 계약 이름은 `60-second drift check`다. 위 fresh-approved power-on과 별도 readiness 관찰이 성공한 뒤 `/cmd_vel`이 zero인 상태에서 같은 60초 구간의 `/odom`, `/joint_states`, `/motor_ready`, `/rosout`을 동시에 기록한다. `timeout 65`는 프로세스 정리 여유 5초를 포함하며 판정 구간은 처음 60초다.

base 시작 전에 관찰 도구를 준비하고, 실행 제한을 준비·60초 기록·토크 해제·종료 확인에 충분한 최소 180초로 설정한다. 관찰 중 `/odom` 또는 바퀴 joint state가 0.5초 넘게 끊기거나 base가 먼저 종료되면 즉시 기록을 중단하고 토크 해제를 확인한다. 벽시계 60초가 지났어도 ROS stamp 구간이 59초 미만이면 수락하지 않는다.

```bash
CAPTURE_DIR="$(mktemp -d)"
timeout 65 ros2 topic echo /odom >"$CAPTURE_DIR/odom.txt" &
ODOM_PID=$!
timeout 65 ros2 topic echo /joint_states >"$CAPTURE_DIR/joint_states.txt" &
JOINT_PID=$!
timeout 65 ros2 topic echo /motor_ready --qos-reliability reliable --qos-durability transient_local >"$CAPTURE_DIR/motor_ready.txt" &
READY_PID=$!
timeout 65 ros2 topic echo /rosout >"$CAPTURE_DIR/rosout.txt" &
ROSOUT_PID=$!
for PID in "$ODOM_PID" "$JOINT_PID" "$READY_PID" "$ROSOUT_PID"; do
  if wait "$PID"; then
    WAIT_STATUS=0
  else
    WAIT_STATUS=$?
  fi
  test "$WAIT_STATUS" -eq 124
done
```

다음 extraction block은 odom header stamp 기준 처음 60초 window를 고르고, 시작/끝 XY, quaternion의 wrapped yaw, 그 window의 최대 wheel velocity를 계산한다. `/motor_ready`는 header가 없으므로 동일한 bounded capture process에서 받은 모든 sample이 true인지 별도 항목으로 출력한다. `python3-yaml`이 없거나 YAML/message/window가 불완전하면 계산을 추정하지 않고 실패한다.

```bash
python3 - "$CAPTURE_DIR" <<'PY'
from pathlib import Path
import math
import sys

import yaml


capture_dir = Path(sys.argv[1])


def load_messages(name):
    with (capture_dir / name).open(encoding="utf-8") as stream:
        return [message for message in yaml.safe_load_all(stream) if message is not None]


def stamp_seconds(message):
    stamp = message["header"]["stamp"]
    return float(stamp["sec"]) + float(stamp["nanosec"]) * 1e-9


def yaw_from_quaternion(orientation):
    x = float(orientation["x"])
    y = float(orientation["y"])
    z = float(orientation["z"])
    w = float(orientation["w"])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


odom = load_messages("odom.txt")
joints = load_messages("joint_states.txt")
motor_ready = load_messages("motor_ready.txt")
rosout = load_messages("rosout.txt")
if not odom or not joints or not motor_ready:
    raise SystemExit("missing capture messages")

odom.sort(key=stamp_seconds)
window_start = stamp_seconds(odom[0])
window_end_limit = window_start + 60.0
odom_window = [message for message in odom if stamp_seconds(message) <= window_end_limit]
if len(odom_window) < 2 or stamp_seconds(odom_window[-1]) - window_start < 59.0:
    raise SystemExit("capture does not cover the 60-second window")

start_pose = odom_window[0]["pose"]["pose"]
end_pose = odom_window[-1]["pose"]["pose"]
x0 = float(start_pose["position"]["x"])
y0 = float(start_pose["position"]["y"])
x1 = float(end_pose["position"]["x"])
y1 = float(end_pose["position"]["y"])
translation_drift = math.hypot(x1 - x0, y1 - y0)
yaw0 = yaw_from_quaternion(start_pose["orientation"])
yaw1 = yaw_from_quaternion(end_pose["orientation"])
yaw_drift = abs(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)))

joint_window = [
    message
    for message in joints
    if window_start <= stamp_seconds(message) <= window_end_limit
]
joint_window.sort(key=stamp_seconds)
if (
    len(joint_window) < 2
    or stamp_seconds(joint_window[-1]) - stamp_seconds(joint_window[0]) < 59.0
):
    raise SystemExit("joint_states does not cover the 60-second window")
velocities = [float(value) for message in joint_window for value in message.get("velocity", [])]
if not velocities:
    raise SystemExit("missing wheel velocity samples")
max_wheel_velocity = max(abs(value) for value in velocities)
motor_ready_all_true = bool(motor_ready) and all(
    message.get("data") is True for message in motor_ready
)
rosout_error_count = sum(int(message.get("level", 0)) >= 40 for message in rosout)

print(f"translation_drift_m={translation_drift:.6f}")
print(f"yaw_drift_rad={yaw_drift:.6f}")
print(f"max_abs_wheel_velocity_rad_s={max_wheel_velocity:.6f}")
print(f"motor_ready_all_true={motor_ready_all_true}")
print(f"rosout_error_count={rosout_error_count}")

if not (
    translation_drift <= 0.01
    and yaw_drift <= 0.02
    and max_wheel_velocity <= 0.05
    and motor_ready_all_true
    and rosout_error_count == 0
):
    raise SystemExit("60-second drift acceptance failed")
PY
```

목표 수락값은 `translation drift <=0.01 m`, wrapped `yaw drift <=0.02 rad`, 세 wheel의 `max absolute wheel velocity <=0.05 rad/s`, `/joint_states` first/last stamp span `>=59 s`, 구간 전체 `/motor_ready true`, serial/encoder error 없음이다. 이 값은 아직 실물에서 관찰한 결과가 아니다. 하나라도 넘거나 pose/twist가 계속 증가하면 실패다. 성공한 drift 측정도 아래 block으로 disable하고 `/motor_ready false`와 물리 정지를 확인해야 한다. 확인되지 않으면 물리 전원을 차단하고 motion trial로 진행하지 않는다.

```bash
DRIFT_DISABLE_RESPONSE="$(timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}")" || exit 1
printf '%s\n' "$DRIFT_DISABLE_RESPONSE"
motor_power_succeeded <<<"$DRIFT_DISABLE_RESPONSE" || exit 1

DRIFT_DISARMED_RESPONSE="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || exit 1
printf '%s\n' "$DRIFT_DISARMED_RESPONSE"
grep -Eq '^data: false$' <<<"$DRIFT_DISARMED_RESPONSE" || exit 1

read -r -p "Physical stop confirmation after drift: type 'DRIFT STOPPED': " DRIFT_STOP_CONFIRMATION
test "$DRIFT_STOP_CONFIRMATION" = "DRIFT STOPPED"
trap - EXIT INT TERM
```

## 단계별 공중 구동

각 단계는 `(max_wheel_speed, speed_tick_limit)` 쌍을 launch override로 함께 적용하고 body test limit `max_linear_x/y/speed:=0.03`, `max_angular_z:=0.1`은 유지한다. 현행 `speed_tick_scale=651.8986469044033`과 실물 주행 상한 `2.3009711818284617 rad/s`를 따른다. 한 단계의 방향, timeout, readiness, stop/fault 검사가 모두 통과하기 전에는 다음 단계로 올리지 않는다.

1. `(1.0, 652)`
2. `(2.0, 1304)`
3. `(2.3009711818284617, 1500)`

최종 software wheel 상한은 현재 base 설정의 `1500 × 2π / 4096 = 2.3009711818284617 rad/s`다. 첫 두 단계의 tick 상한은 해당 wheel 상한에 `651.8986469044033 tick/(rad/s)`를 곱한 값을 올림했다. 각 단계에서 바퀴 joint state의 속도 부호가 명령과 일치하고 절대값이 해당 단계 상한을 넘지 않는지 확인한다.

실제 non-zero `/cmd_vel`은 직접 `ros2 topic pub`이나 일반 teleop으로 보내지 않는다. 아래 STOP에서 작업을 멈추고 로봇 고정, 비상 차단 담당자와 zero `/cmd_vel`을 다시 확인한다. 이어지는 wrapper는 18개 trial 각각에 대해 새 motion confirmation을 받은 뒤 disarmed 상태를 확인한다. `lekiwi_safe_cmd_vel --arm-motors`가 토크 활성화 전에 ROS 연결을 마치고 `/motor_ready=false → true`를 직접 관찰한 뒤에만 명령을 발행한다.

**STOP: fresh user confirmation required immediately before first non-zero command**

`--yes-i-confirm-motion` is a CLI acknowledgement, not authorization.

별도 terminal의 hardware bringup은 처음에 `(1.0, 652)`, `max_linear_x/y/speed:=0.03`, `max_angular_z:=0.1`, `torque_enable:=false`로 실행한다. 각 stage prompt에서 bringup을 종료하고 표시된 body test limit, `max_wheel_speed`, `speed_tick_limit`, `torque_enable:=false`로 다시 launch한 뒤 정확한 stage 문자열을 입력한다. 아래 authoritative block 전체를 operator terminal에서 한 번 실행한다. 각 trial은 confirmation, cleanup trap, disarmed 확인, 명령 도구 내부의 bounded power-on·독립 readiness 관찰·단 하나의 bounded safe command·bounded disable, 외부 disable 재확인, `/motor_ready false`, 물리 정지 확인을 모두 새로 수행한다. 한 trial의 arm이나 확인은 다음 trial에 승계되지 않는다.

```bash
# AIRBORNE_TRIAL_MATRIX_START
set -Eeuo pipefail

STAGES=("1.0:652" "2.0:1304" "2.3009711818284617:1500")
DIRECTIONS=("forward" "reverse" "left" "right" "ccw" "cw")

best_effort_disable_airborne_trial() {
  timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}" || true
}
interrupt_airborne_trial() {
  exit 130
}

run_airborne_trial() {
  # RUN_AIRBORNE_TRIAL_BEGIN
  local stage_max="$1"
  local stage_tick="$2"
  local direction="$3"
  local -a motion_args
  case "$direction" in
    forward) motion_args=(--linear-x 0.03) ;;
    reverse) motion_args=(--linear-x -0.03) ;;
    left) motion_args=(--linear-y 0.03) ;;
    right) motion_args=(--linear-y -0.03) ;;
    ccw) motion_args=(--angular-z 0.1) ;;
    cw) motion_args=(--angular-z -0.1) ;;
    *) return 2 ;;
  esac

  local expected_confirmation="RUN ${stage_max}:${stage_tick} ${direction}"
  local confirmation
  read -r -p "Fresh motion confirmation: type '${expected_confirmation}': " confirmation
  test "$confirmation" = "$expected_confirmation"

  trap best_effort_disable_airborne_trial EXIT
  trap interrupt_airborne_trial INT TERM

  local ready_response
  ready_response="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || return 1
  printf '%s\n' "$ready_response"
  grep -Eq '^data: false$' <<<"$ready_response" || return 1

  timeout --signal=INT --kill-after=2 20 ros2 run lekiwi_teleop lekiwi_safe_cmd_vel \
    "${motion_args[@]}" \
    --duration 1.0 \
    --max-duration 1.0 \
    --arm-motors \
    --yes-i-confirm-motion

  local disable_response
  disable_response="$(timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}")" || return 1
  printf '%s\n' "$disable_response"
  grep -q "success: true" <<<"$disable_response" || return 1

  local disarmed_response
  disarmed_response="$(timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local)" || return 1
  printf '%s\n' "$disarmed_response"
  grep -Eq '^data: false$' <<<"$disarmed_response" || return 1

  local expected_stop="STOPPED ${stage_max}:${stage_tick} ${direction}"
  local stop_confirmation
  read -r -p "Physical stop confirmation: type '${expected_stop}': " stop_confirmation
  test "$stop_confirmation" = "$expected_stop"
  trap - EXIT INT TERM
  # RUN_AIRBORNE_TRIAL_END
}

for STAGE in "${STAGES[@]}"; do
  IFS=: read -r STAGE_MAX STAGE_TICK <<<"$STAGE"
  printf 'Relaunch hardware bringup with max_linear_x:=0.03 max_linear_y:=0.03 max_linear_speed:=0.03 max_angular_z:=0.1 max_wheel_speed:=%s speed_tick_limit:=%s torque_enable:=false\n' \
    "$STAGE_MAX" "$STAGE_TICK"
  read -r -p "After relaunch, type 'STAGE ${STAGE} READY': " STAGE_CONFIRMATION
  test "$STAGE_CONFIRMATION" = "STAGE ${STAGE} READY"

  ACTUAL_MAX="$(timeout 5 ros2 param get /lekiwi_node max_wheel_speed | awk '{print $NF}')"
  ACTUAL_TICK="$(timeout 5 ros2 param get /lekiwi_node speed_tick_limit | awk '{print $NF}')"
  test "$ACTUAL_MAX" = "$STAGE_MAX"
  test "$ACTUAL_TICK" = "$STAGE_TICK"

  for DIRECTION in "${DIRECTIONS[@]}"; do
    run_airborne_trial "$STAGE_MAX" "$STAGE_TICK" "$DIRECTION"
  done
done
```

전진/후진, 좌/우 횡이동, 반시계/시계 회전을 각각 확인한다. wrapper가 어느 지점에서든 실패하면 installed EXIT trap의 bounded disable은 시도일 뿐이다. operator는 즉시 물리 정지를 확인하고, 확인할 수 없으면 물리 전원을 차단한다. 반대 wheel, 진동, packet/encoder 오류, `/motor_ready false`, command timeout 뒤 회전 지속, zero 또는 torque-off 실패가 한 번이라도 나타나면 그 단계는 실패다. 상한을 올리거나 다음 trial을 시작하지 않는다.

## Encoder feedback loss with write path available

이 검사는 `(1.0, 652)` 단계와 정상 정지를 먼저 통과하고, encoder read만 차단해도 command write 경로는 독립적으로 유지된다는 fault injection 수단이 별도로 입증된 경우에만 non-zero 상태에서 수행한다. 현재 backend는 같은 serial transport로 read와 write를 수행하므로 cable 분리나 serial disconnect는 이 조건을 만족하지 않는다. 독립 수단이 없으면 이 검사는 pending으로 기록하고 non-zero 상태에서 연결을 끊지 않는다.

독립 write 경로가 확인된 경우의 목표 수락 조건은 `encoder_sample_timeout` 안에 zero wheel command 시도, `/motor_ready false`, fault latch가 나타나고 fault 뒤 non-zero 재시도가 거부되는 것이다. 원인을 복구한 뒤에도 `/motor_power false`, `/motor_power true`, fresh encoder sample 순서를 거치기 전에는 ready가 되면 안 된다. software 응답과 packet log만으로 물리 정지를 판정하지 않고 wheel을 직접 관찰한다.

## Serial disconnect at zero/torque-off

serial disconnect 검사는 non-zero 명령과 완전히 분리한다. 먼저 유한 명령이 끝난 뒤 zero wheel을 관찰하고 `/motor_power false`를 요청한다. torque-off와 물리 정지를 확인한 상태에서만 cable을 분리한다. `stop/disable packets are attempts, not observed physical stop`; service success나 write 반환값도 실제 wheel 정지의 증거가 아니다.

연결이 끊기면 software가 더는 zero/disable packet 전달을 보장할 수 없다. 따라서 `physical power cutoff is mandatory on serial disconnect`이며 담당자가 차단 수단을 즉시 실행할 수 있어야 한다. 분리 후 `/motor_ready false`와 fault latch를 확인하되 wheel 움직임이 있거나 torque-off를 관찰로 확인할 수 없으면 즉시 물리 전원을 차단하고 이후 공중/바닥 단계를 중지한다.

## Acceptance record

결과는 repo 밖의 운영 기록에 다음 값만 남긴다.

- 날짜, 작업자, 소프트웨어 commit
- 통과한 wheel/tick 단계와 여섯 방향 부호
- 60초 drift의 시작/끝 odom과 최대 wheel speed
- timeout, 독립 encoder feedback loss, zero/torque-off 뒤 serial disconnect 각각의 `/motor_ready`, fault/recovery, 물리 정지 관찰
- 실패 시점과 중지 이유

map, rosbag, `build/`, `install/`, `log/`, 실제 serial ID, 개인 절대 경로는 커밋하지 않는다.

## 이전 참고 기록

2026-07-02와 2026-07-06에 `4096` raw position modulus와 여섯 방향 부호를 조사한 메모가 있었다. 이는 현재의 `/motor_ready`, 단계별 wheel/tick 상한, 60초 drift, serial/encoder-loss 수락 계약보다 앞선 기록이므로 현행 공중 검증 통과로 간주하지 않는다. 바닥 거리 보정은 raw modulus를 바꾸지 않고 별도 `encoder_odom_scale`로 수행한다.
