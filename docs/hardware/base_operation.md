# Base 운용과 하드웨어 확인

> 이 문서의 수치는 기본 설정값이다. 바퀴 ID·방향·기구학·엔코더 환산값은 출고 장비에서 확인한다.

기본 pub/sub 확인은 [토픽 점검 안내](topic_tests.md)를 따른다.
이 문서는 실물 모터 설정, guarded teleop, 방향·정지·거리 검증에 사용하는 상세 절차다.
ROS 환경과 workspace는 [설치 및 실행 안내](../../README.md)를 먼저 적용한다.
자동 토픽 검사 결과는 [토픽 점검 기록](topic_tests.md), 이전 바닥 거리 실측은
[거리 시험 기록](distance_trial.md), 현재 SLAM·Nav2 현장 결과는
[SLAM·Nav2 안내](slam_navigation.md)를 따른다.

## 현재 지원 범위

- `mode:=base`
  - `robot_state_publisher`
  - `lekiwi_node`
  - 선택적 YDLIDAR launch
  - BMI160 launch (`use_imu:=true`가 표준 조립품 기본값)
  - TF: `odom -> base_footprint -> base_link -> lidar_link`, `base_link -> imu_link`

SOARM 팔 형상은 기본 모델 미리보기에 포함한다. 팔 제어와 MoveIt은 현재 범위에서 제외한다.
별도 브랜치와 기존 Isaac Sim 산출물은 보관한다.

표준 Pi bringup은 `motor_backend: feetech`, `enable_motor_write: true`, `use_imu: true`, `odom_source: encoder`, `encoder_feedback_source: position`, `torque_enable: false`, `cmd_vel_timeout: 0.3`이다. 모터 버스를 열어 토크 해제와 목표 속도 0을 확인하지만 주행 토크는 별도 서비스로 활성화해야 한다. `base.yaml`의 mock/write-disabled 값은 오프라인 검사 기준으로 유지한다. mock에서 encoder feedback이 없으면 `/odom` pose는 정지하며 command odometry로 자동 전환하지 않는다. base controller의 기본 software velocity ceiling은 x/planar `0.11 m/s`, y `0.10 m/s`, yaw `0.75 rad/s`, `max_wheel_speed: 2.3009711818284617`, `speed_tick_scale: 651.8986469044033`, `speed_tick_limit: 1500`이다. 저속 주행용 guarded teleop은 별도로 x/y `0.10 m/s`, yaw `0.30 rad/s`로 제한한다. Feetech backend는 STS/SMS speed mode, torque enable/disable, sync goal speed packet, present speed/position read를 사용한다. `wheel_ids`와 `wheel_directions`는 모델 joint 순서에 맞게 출고 장비에서 검증한다. encoder raw wrap 기본값은 `encoder_position_ticks_per_revolution: 4096.0`, 거리 보정은 `encoder_odom_scale: 1.0`, `encoder_position_tick_deadband: 0`이다. position feedback에서 이 deprecated deadband는 반드시 `0`이어야 하며 non-zero 설정은 시작할 때 거부된다.

`odom_source: command`에서는 `/cmd_vel` command twist를 적분해 `/odom`을 publish한다. `cmd_vel_timeout` 이후 `/odom.twist.twist`는 zero twist로 publish되며, command 기반 odom임을 반영하기 위해 pose/twist covariance는 명시적인 non-zero 대각값을 사용한다. `odom_source: encoder`의 position feedback에서는 수락한 raw encoder delta를 pose에 정확히 한 번 적용하고, odom twist와 `/joint_states` wheel velocity는 최근 세 read interval의 signed moving window로 계산한다. 따라서 `3 * encoder_read_period <= encoder_sample_timeout`이어야 한다. 기본 `0.1`초/`0.5`초 조합은 읽기 주기 세 번에 더해 일시적인 실행 지연 두 주기의 여유를 둔다. encoder read 실패 시 `/odom`은 command로 fallback하지 않고 zero twist를 사용한다. startup 또는 stale deadline을 넘으면 zero wheel command 시도, torque-off, fault latch를 수행하고, `/motor_power false`, `/motor_power true`, fresh encoder sample 순서로 복구한다. 실제 hardware motion readiness는 transient-local `/motor_ready`로 publish된다. `/joint_states`는 fresh encoder sample이 있으면 encoder wheel speed를 쓰고, encoder sample이 없거나 stale이면 현재 command wheel speed로 fallback한다. 이 command wheel speed도 `cmd_vel_timeout` 이후에는 zero가 된다. `encoder_feedback_source: speed`의 speed-register 경로는 변경되지 않으며 `encoder_tick_deadband`가 계속 적용되고, deprecated position deadband와 세 interval timing 제약은 적용되지 않는다.

`tf_time_offset` 기본값은 `0.0`이다. RViz에서 `/scan` 표시 중 `lidar_link -> odom` future extrapolation 경고가 반복될 때만 현장에서 `/scan.header.stamp`와 최신 `/tf` stamp 차이를 확인한 뒤 `0.02`~`0.05`초 범위의 작은 값으로 조정한다. 이 값은 `odom -> base_footprint` TF stamp에만 적용하고 `/odom` 메시지 stamp는 그대로 둔다.

2026-06-20 라즈베리파이 실물 검증 기준 `/cmd_vel` 방향 계약:

- `linear.x > 0`: 전진
- `linear.x < 0`: 후진
- `linear.y > 0`: 좌측 이동
- `linear.y < 0`: 우측 이동
- `angular.z > 0`: 위에서 봤을 때 반시계 회전
- `angular.z < 0`: 위에서 봤을 때 시계 회전

2026-07-02/06의 공중 메모에는 `/cmd_vel` 6방향 부호와 raw wrap 후보 `encoder_position_ticks_per_revolution: 4096.0`이 기록돼 있다. 이 기록은 현재 `/motor_ready`, 단계별 wheel/tick 상한, 60초 drift, serial/encoder-loss 수락 계약보다 앞서므로 현행 공중 검증 통과를 뜻하지 않는다. raw wrap은 `4096.0`으로 유지하고 실제 거리 오차는 바닥에서 `encoder_odom_scale`로 보정한다. 최신 절차는 `docs/hardware/encoder_odom_airborne_check.md`를 따른다.

바닥 주행 보정은 `docs/hardware/encoder_odom_floor_calibration.md` 절차를 따른다. 실측값으로 다음 scale 후보를 계산할 때는 `lekiwi_odom_calibration`을 사용한다.

```bash
ros2 run lekiwi_teleop lekiwi_odom_calibration \
  --current-scale 1.0 \
  --odom-delta 0.60 \
  --measured-delta 0.50
```

출력되는 `encoder_odom_scale`만 launch override로 재검증하고 raw wrap은 바꾸지 않는다. 설정 가능한 body velocity ceiling은 `|linear.x| <= 0.11 m/s`, `|linear.y| <= 0.10 m/s`, planar speed `<= 0.11 m/s`, `|angular.z| <= 0.75 rad/s`다. 이는 즉시 실물 최대 속도를 사용하라는 뜻이 아니다. 이전 환산 600에서 사용한 공중 검증 단계는 `(1.0, 600)`, `(2.0, 1200)`, `(3.5, 2100)`, `(5.4, 3240)`였다. 새 환산에서는 `(1.0, 652)`부터 시작하고 최종 상한은 `(2.3009711818284617, 1500)`이다. 새 수치의 실물 검증은 아직 수행하지 않았다.

BMI160 driver/publisher는 `lekiwi_sensors`에 포함되어 있다. Bosch 공식 SensorAPI 원본은
`lekiwi_sensors/vendor/bosch_bmi160`에 라이선스·고정 commit·SHA-256과 함께 분리 보관한다.
기존 배포 구조와 검증 기록은
[`docs/hardware/bmi160_imu_validation.md`](bmi160_imu_validation.md)를 따른다.
2026-08-19 현재 실물 장비에서는 고정 후 `axis_map: [1, 2, 3]`, 약 100 Hz, 정지 중력
`+z`, 반시계 회전 `angular_velocity.z > 0`을 확인했다.

## 단계별 현장 확인

1. **Offline complete**: 현재 8개 패키지의 build/test와 mock 환경에서 토픽 연결을 확인한다.
2. **Airborne supported robot**: 바퀴를 안전하게 띄우고 `/motor_ready`, 정지 상태의 60-second drift check, 방향과 timeout/fault 정지 처리를 확인한다. 기존 단계별 속도 상한 `(1.0, 600)`, `(2.0, 1200)`, `(3.5, 2100)`, `(5.4, 3240)`은 과거 환산값에 대한 기록이다. 현재 상한 확대 절차로 사용하지 않는다.
3. **Teleop / odom / scan**: 검증된 저속 범위에서 README의 기본 pub/sub 기능을 확인하고 실제 관찰 결과를 기록한다.

수동 wheel-direction non-zero 명령은 [공중 검증](encoder_odom_airborne_check.md)의 유한 명령 절차를 사용하고, 키보드 주행은 아래 guarded teleop을 사용한다. 실제 정지와 토크 해제는 직접 확인한다.
`encoder_odom_scale` 정밀 보정이 필요할 때만 [바닥 보정](encoder_odom_floor_calibration.md)을 별도로 수행한다.
2026-09-23에 지도 저장과 정지 상태의 localization 입력·TF를 확인했다.
방 전체 지도 품질과 Nav2 목표 도달은 아직 확인하지 못했다.

## 라즈베리파이 실사용 Bringup

실제 base motor, `robot_state_publisher`, `/odom`, `/tf`, 선택적 `/scan`을 라즈베리파이에서 띄울 때의 기준 workspace는 `$HOME/lekiwi_ws`다. `colcon build`와 `source install/setup.bash`는 repo 디렉터리가 아니라 workspace 루트에서 실행한다.

표준 장비는 다음 명령으로 base·라이다·IMU를 시작한다. 기본 포트는 모터 `/dev/ttyACM0`, 라이다 `/dev/ttyUSB0`, IMU `/dev/i2c-1`(주소 `0x68`)이다. 장비별 조립 검수에서 포트와 wheel ID를 확인한다. `torque_enable:=false`가 기본이므로 실행만으로 주행하지 않는다.

```bash
ros2 launch lekiwi_bringup robot.launch.py
```

mock 검사는 [개발용 진단 도구](topic_tests.md)를 선택 설치한 뒤 `topic_test.launch.py`를 사용한다. 그때는 motor serial을 열지 않고 command odometry를 사용한다. 아래 명령은 **공중 고정 상태의 저속 진단용 override**이며, 표준 실행에 필요하지 않다.

```bash
MOTOR_PORT="/dev/serial/by-id/REPLACE_WITH_FEETECH_PORT"
ros2 launch lekiwi_bringup robot.launch.py \
  mode:=base \
  use_lidar:=false \
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

Feetech 포트는 `/dev/ttyUSB*`보다 재부팅 후에도 이름이 안정적인 `/dev/serial/by-id/...` 사용을 권장한다. 실제 값은 장비마다 다르므로 아래 명령으로 확인하고 `REPLACE_WITH_FEETECH_PORT`를 셸에서만 교체한다.

```bash
ls -l /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
```

실물 파라미터 참고용 파일은 `lekiwi_bringup/param/base_profile.example.yaml`에 있다. 장비마다 저장소 밖으로 복사하고 wheel ID·방향·반지름·보정값을 확인한 뒤 `base_params_file:=/absolute/path/to/profile.yaml`로 읽는다. 모터 포트와 속도 상한 등 같은 이름의 launch 인자는 YAML 값을 덮으므로 실제 포트는 `serial_port:=/dev/serial/by-id/...`로 지정한다. 예시 파일은 launch 기본값으로 자동 사용되지 않는다. 아직 검수되지 않은 새 장비의 첫 실물 단계는 body test limit `max_linear_x/y/speed:=0.03`, `max_angular_z:=0.1`과 launch override `(1.0, 652)`으로 낮춰 시작한다. `lekiwi_bringup/param/base.yaml`의 mock/write-disabled 값은 오프라인 검사 기준으로 유지한다. 검증된 wheel/tick stage보다 높은 ceiling은 지지대 공중 검증 전까지 바닥 주행에 사용하지 않는다.

bringup 후 기본 토픽 확인:

```bash
ros2 topic list
ros2 topic echo /robot_description --once
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

encoder odom을 켠 경우 `enable_motor_write:=true`가 필요하다. 처음 실물 검증은 `torque_enable:=false`로 시작하며 `/motor_power true`는 아래 일반 참고 명령이 아니라 현장 안전 확인을 거친 공중 검사에서만 실행한다. 바퀴를 띄운 상태에서 먼저 `/odom.twist.twist`가 zero에 가깝게 유지되는지 확인하고, 이후 bounded `/cmd_vel` 명령으로 방향을 하나씩 확인한다. 이 단계에서 보이는 `/odom` pose 증분은 encoder 환산 odom 확인용이며, 줄자/마커로 잰 실제 이동거리 검증을 대체하지 않는다. 기본 `wheel_ids: [8, 9, 7]`, `wheel_directions: [1.0, 1.0, 1.0]`은 장비별 검증값이 아니다. 바닥 거리 시험은 [거리 시험 절차](distance_trial.md)를 따른다.

라이다 포함 실행이면 `/scan`과 `lidar_link` TF도 확인한다.

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link lidar_link
```

odometry pose를 원점으로 되돌릴 때는 `/reset_odometry`를 호출한다. 모터 torque/power 상태를 명시적으로 바꿀 때는 `/motor_power`를 사용한다. `mock` 또는 offline backend에서는 실제 serial write 없이 내부 상태만 바꾸는 성공 응답을 줄 수 있다. 공중 검증의 guarded predicate는 ROS CLI 성공 표현 `success=True`와 `success: true`를 모두 수락하고, `success=False`와 `success: false`는 거부한다. 실제 Feetech write 실패 시 `/motor_power` 응답은 `success: false`와 실패 메시지를 반환한다.

```bash
ros2 service call /reset_odometry std_srvs/srv/Trigger {}
ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"
```

실물 `/motor_power true`는 일반 참고 명령으로 실행하지 않는다. [공중 검증 절차](encoder_odom_airborne_check.md)의 fresh-approved guarded block 또는 아래 guarded teleop 내부에서 bounded timeout, response 검사, 별도 `/motor_ready` 관찰과 cleanup을 함께 수행한다.

### 모터 구동 전 체크리스트

- base를 바닥에서 띄우거나, 주변에 사람과 장애물이 없는 안전한 상태로 둔다.
- base mode에서는 팔을 움직이지 않는다. 팔 제어는 이 bringup 범위에 포함되지 않는다.
- wheel IDs가 `7=left`, `8=back`, `9=right`인지 확인한다.
- Feetech 포트가 실제 base motor bus의 `/dev/serial/by-id/...`인지 확인한다.
- encoder odom 사용 시 `odom_source:=encoder`를 켜고, 바퀴를 손으로 돌려 encoder feedback 방향을 먼저 확인한다.
- 실제 모터 write와 `/cmd_vel` publish는 반드시 현장 사용자 컨펌 후 하나씩 실행한다.
- 긴급 시 zero `/cmd_vel`과 `/motor_power false`를 요청하되 service/packet 성공을 물리 정지로 간주하지 않는다. wheel 정지를 직접 확인할 수 없으면 준비한 물리 전원 차단을 즉시 수행한다.

### 짧은 실물 구동 테스트

실제 base 방향 또는 encoder odom을 확인할 때는 장시간 살아남을 수 있는 직접 `ros2 topic pub --rate ...` 대신 `lekiwi_safe_cmd_vel`을 사용한다. 이 유틸리티는 정해진 횟수만 `/cmd_vel`을 publish하고, 시작/종료/예외 경로에서 zero `Twist`를 반복 publish한다. `--arm-motors`를 사용하면 시작 전 `/motor_power true`, 종료 시 `/motor_power false`를 호출한다.

non-zero motion 또는 `--arm-motors`는 Airborne gate의 동작 없는 검사와 60초 drift를 통과한 뒤에만 실행한다. 배포 기본 안전 상한은 `--max-linear 0.05`, `--max-angular 0.3`, `--max-duration 2.0`, `--max-rate 50.0`이다. 더 큰 실험이 필요하면 현장에서 값을 확인한 뒤 해당 `--max-*` 인자를 명시적으로 올린다. 시작/종료/예외 경로의 zero `/cmd_vel` publish와 `/motor_power false` 정리 계약은 유지한다.

로봇 고정, 비상 차단 담당자, `/motor_ready true`, zero `/cmd_vel`을 현장에서 다시 확인한 다음 여기서 멈춘다.

**STOP: fresh user confirmation required immediately before first non-zero command**

`--yes-i-confirm-motion` is a CLI acknowledgement, not authorization.

미리 작성된 `--yes-i-confirm-motion`은 승인 자체가 아니다. 실제 power-on/readiness/60초 drift/첫 motion은 [공중 검증 절차](encoder_odom_airborne_check.md)의 **authoritative guarded Airborne blocks**만 사용한다. 이 문서에는 cleanup과 별도 readiness 확인이 빠진 standalone physical command를 두지 않는다.

### Base Teleop

#### 실물 전용: 키를 누르는 동안만 움직이는 guarded teleop

실물 키보드 주행에는 그래픽 데스크톱이 있는 PC에서 `lekiwi_guarded_base_teleop`을 사용한다.
`python3-tk`가 필요하다. 실행 파일은 fresh 현장 승인 뒤에도 `--yes-i-confirm-motion`이 없으면
시작을 거부한다. 시작 zero `/cmd_vel`, `/motor_power true` 성공, 별도 transient-local
`/motor_ready true` 확인을 순서대로 마친 뒤 조작 창을 활성화한다. 기본 최대 세션은 3600초다.
CLI acknowledgement는 현장 승인 자체를 대신하지 않는다.

```bash
# STOP 지점에서 fresh 현장 승인을 받은 직후에만 실행
ros2 run lekiwi_teleop lekiwi_guarded_base_teleop --yes-i-confirm-motion
```

필요하면 `--max-session-duration 3600` 형식으로 현장 승인 범위에 맞춘다. 방향 키는 누르는
동안만 움직이고 놓으면 zero `/cmd_vel`을 즉시 보낸다. 창의 포커스를 잃거나 Space를 누르면
정지하며, 그때 누르고 있던 방향 키는 실제로 뗐다가 다시 눌러야 재시작된다. 다른 방향 키를
함께 누를 경우 마지막에 누른 한 방향만 선택하고, 방향 전환 중 zero 명령을 먼저 보낸다.

| 키 | 동작 |
| --- | --- |
| `w` / `s` | 누르는 동안 전진 / 후진 |
| `a` / `d` | 누르는 동안 좌 / 우 이동 |
| `q` / `e` | 누르는 동안 반시계 / 시계 회전 |
| `1` / `2` / `3` | 낮음 / 중간 / 높음 속도 단계 선택 |
| Space | 즉시 정지, 속도 단계는 유지 |
| Esc 또는 창 닫기 | 반복 zero 명령과 모터 토크 해제 후 종료 |

현재 1/2/3단의 임시 상한은 전후·좌우 `0.04/0.05/0.06 m/s`, 회전
`0.23/0.29/0.35 rad/s`다. 회전값은 공중에서 보이는 바퀴의 최대 회전 속도를 각 단계의
직진·후진 바퀴와 비슷하게 맞춘 값이다. 측면 이동은 세 바퀴에 필요한 속도 비율이
`2:1:1`이므로, 두 바퀴가 직진 때보다 느리게 보이는 것은 정상이다. 높은 속도의 실제
동시성과 바닥 이동 속도는 별도 측정이 필요하다. 이 범위를 사용하려면 Pi bringup의
`max_linear_x`, `max_linear_y`, `max_linear_speed`가 각각 최소 `0.06`,
`max_angular_z`가 최소 `0.35`, `max_wheel_speed`가 최소 `1.2`, `speed_tick_limit`가
최소 `783`이어야 한다. 모터 설정의 상한이 단계 수치보다 낮으면 실제 모터 명령은 해당
상한으로 제한된다.
정상 종료·시간 초과·준비 실패·예외에서도
zero `/cmd_vel` 20회와 bounded `/motor_power false`를 시도한다. 종료 메시지나 서비스 성공만으로
물리 정지를 가정하지 말고 바퀴를 직접 확인한다.

#### Mock/write-disabled 전용: Classic base teleop demo

`lekiwi_base_teleop`은 `/scan` 없이 classic 키 매핑과 command odometry를 보는 **mock/write-disabled demonstration only** 유틸리티다. 이 노드는 스스로 hardware write를 켜지 않지만, 살아 있는 동안 키 입력에 따라 제한 시간 없이 `/cmd_vel`을 계속 만들 수 있다. 따라서 **must not be used in physical hardware acceptance**: `motor_backend:=feetech` 또는 `enable_motor_write:=true`인 bringup과 함께 실행하지 않는다. 실물에서는 이 demo가 아니라 위의 **separate guarded bounded wrapper**를 사용한다.

아래 표는 오직 `lekiwi_base_teleop` mock demo의 classic 키 매핑이다. 실물용 `lekiwi_guarded_base_teleop`에는 적용하지 않는다.

| Classic demo 키 | `/cmd_vel` |
| --- | --- |
| `w` | 전진, `linear.x > 0` |
| `s` | 후진, `linear.x < 0` |
| `a` | 좌측 이동, `linear.y > 0` |
| `d` | 우측 이동, `linear.y < 0` |
| `q` | 반시계 회전, `angular.z > 0` |
| `e` | 시계 회전, `angular.z < 0` |
| `+`, `=` | 현재 teleop 선속도/각속도 증가. `/cmd_vel`은 publish하지 않음 |
| `-`, `_` | 현재 teleop 선속도/각속도 감소. `/cmd_vel`은 publish하지 않음 |
| `space` | 정지, zero `Twist` |
| `x`, `Esc`, `Ctrl-C` | zero `Twist` publish 후 종료 |

기본 teleop 속도는 `linear_speed:=0.05`, `angular_speed:=0.3`이고 clamp는 `min_linear_speed:=0.02`, `max_linear_speed:=0.10`, `min_angular_speed:=0.2`, `max_angular_speed:=0.3`, `speed_scale_step:=1.2`다. 아래 예시는 실제 motor bus가 없는 mock/write-disabled terminal 두 개에서만 실행한다.

```bash
# terminal 1: mock/write-disabled demonstration
ros2 launch lekiwi_bringup robot.launch.py \
  mode:=base \
  use_lidar:=false \
  motor_backend:=mock \
  enable_motor_write:=false \
  odom_source:=command

# terminal 2: source한 뒤 mock bringup에만 연결
ros2 run lekiwi_teleop lekiwi_base_teleop --ros-args \
  -p max_linear_speed:=0.10 \
  -p max_angular_speed:=0.3 \
  -p speed_scale_step:=1.2
```

## YDLIDAR 설치와 빌드

SDK → 드라이버 → LeKiwi 순서의 빌드는 [README](../../README.md)를 따른다.
라이다만 실행할 때는 `ros2 launch lekiwi_sensors ydlidar.launch.py`를 사용한다.
기본 port는 `/dev/ttyUSB0`, frame은 `lidar_link`이며 고정 장치 경로는 `lidar_port`로 지정한다.

기본 경로는 driver의 `/scan_raw` → `scan_timing_normalizer` → `/scan`이다.
normalizer는 `time_increment`를 0으로 만드는 기존 호환 설정을 유지한다.
`use_scan_timing_normalizer:=false driver_scan_topic:=/scan`이면 원본 `/scan`만 발행하므로
`/scan_raw`도 확인하는 현재 `lidar` 서비스 검사에는 기본 설정을 사용한다.
Tmini Plus 설정은 `lekiwi_sensors/param/ydlidar.yaml`에 있으며, 자체 가림 필터는 적용하지 않는다.
