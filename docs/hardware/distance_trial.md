# 20cm 이동 비교 시험

`lekiwi_distance_trial`은 `/cmd_vel` 속도를 정해진 시간 동안 발행하고, 시작·정지 후
엔코더 odom 변위와 나중에 입력한 실측 변위를 비교한다. 기본값은 전진 0.02m/s × 10초다.
**odom이 20cm에 도달할 때까지 움직이는 제어기가 아니다.** 실제 20cm 이동을 보장하지 않는다.
형상·모터 속도 계수·odom scale을 변경하거나 odom을 초기화하지 않는다.

## 계획만 확인

```bash
ros2 run lekiwi_teleop lekiwi_distance_trial plan
```

ROS 없이도 저장소 루트에서 실행할 수 있다:

```bash
PYTHONPATH=lekiwi_teleop python3 -m lekiwi_teleop.distance_trial plan
```

단위는 m와 m/s다. `--distance 0.2 --speed 0.02 --direction forward`가 기본이며
`backward`, `left`, `right`도 지원한다. 상한은 0.3m, 0.03m/s, 30초다.

## Mock 확인

별도 ROS domain의 두 터미널에서 동일한 workspace를 source한다. 첫 터미널:

```bash
ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ros2 run lekiwi_node lekiwi_node --ros-args \
  -p motor_backend:=mock -p enable_motor_write:=false \
  -p torque_enable:=false -p odom_source:=command
```

두 번째 터미널:

```bash
ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ros2 run lekiwi_teleop lekiwi_distance_trial run --mock --output /tmp/distance-mock.json
```

mock 모드는 실제 backend와 write-enabled controller를 거부하며 토크를 켜지 않는다.
mock 결과는 명령 odom이므로 실측 비교에 사용할 수 없다.

## 실제 바닥 시험

바닥에 내려놓은 뒤 시험 방향·경로·즉시 전원 차단 가능 여부와 손을 뗀 상태를 확인한다.
공중 고정 승인은 바닥 이동 승인으로 간주하지 않는다. 기존 실물 base controller만 사용하며
이 도구는 controller나 센서를 새로 실행하지 않는다. 다른 teleop publisher는 종료되어야 한다.
controller는 `feetech`, write enabled, `torque_enable=false`, encoder position odom이어야 한다.
`/motor_ready=false`에서 시작하며 현재 설정과 publisher 구성을 검사한 뒤에만 토크를 켠다.

시작 위치와 방향을 바닥에 표시한다. 차체의 **같은 기준점**을 측정한다. 회전이 생겼을 때
팔 끝처럼 기준점에서 멀리 떨어진 점을 측정하면 odom 기준점 변위와 차이가 생긴다.

현장 준비와 해당 동작 승인을 마친 뒤 한 번만 실행한다:

```bash
ros2 run lekiwi_teleop lekiwi_distance_trial run \
  --yes-i-confirm-motion --output /tmp/distance-forward-01.json
```

준비 중 zero 명령, 새 encoder readiness, 1초 정지 구간을 확인한다. 20Hz로 주행 명령을
발행하고 10초 후 zero를 보낸다. 이어서 최대 5초 안에 연속 2초의 odom 정지 구간을
확인해 마지막 위치를 저장하고 토크를 해제한다. 감속 중 변위도 결과에 포함된다.
도중 odom 누락/시간 역행/큰 위치 점프, 준비 해제, publisher 충돌, 명령 주기 지연,
3cm 초과 횡방향 변위·0.2rad 초과 회전·목표+5cm 초과 전진 등이 발생하면 중단한다.
Ctrl-C/SIGTERM/SIGHUP도 zero와 토크 해제를 시도하며 자동 재시도하지 않는다.
프로세스 강제 종료나 통신 단절 시에는 기존 controller timeout에 의존한다.

JSON의 `completed`는 도구의 수집·정지 절차 완료이며 **20cm 정확도 합격이 아니다.**
토크 해제 서비스 성공과 `/motor_ready=false`를 기록하지만 모터 레지스터를 직접 읽지는 않는다.
`physical_stop_confirmed`와 `torque_off_register_verified`는 자동으로 참이 되지 않는다.
눈으로 정상 정지를 확인한 뒤 실측한다. 기록이 실패했거나 정지가 확인되지 않으면 재실행보다
원인 확인을 먼저 한다.

## 실측 입력 및 결과

시작 방향으로 투영한 실제 이동량이 18cm라면:

```bash
ros2 run lekiwi_teleop lekiwi_distance_trial compare \
  --report /tmp/distance-forward-01.json --measured 0.18 \
  --tolerance 0.01 --output /tmp/distance-forward-01-comparison.json
```

요청한 방향으로 움직였으면 양수, 반대로 움직였으면 음수다. 0m도 입력 가능하다.
후진 시험에서 의도한 후진 20cm는 `0.20`이다. 대각선 빗변 길이 대신 시작 방향의 변위를
입력하고 옆으로 벗어난 거리도 별도로 관찰한다. odom의 시작 yaw를 사용하므로 odom 원점이나
시작 방향을 0으로 초기화할 필요가 없다.

- `actual_minus_requested_m`: 실제 이동량 − 명령상 목표 거리.
- `odom_minus_actual_m`: odom 이동량 − 실제 이동량.
- `odom_cross_m`, `odom_yaw_rad`: odom의 횡방향 변위와 방향 변화.
- `published_command_integral_m`: 도구의 실제 발행 시간으로 계산한 명령 거리;
  controller 수신 시각이나 실제 속도 적분값은 아니다.
- 기본 허용오차 0.01m는 이 한 번의 비교 기준이며 최종 성능 보증이 아니다.

보고서에 실제 controller 파라미터와 odom 표본을 보존하며 기존 파일은 덮어쓰지 않는다.
바닥 실측·중심 거리 115mm 검증은 별도이며 직진 시험만으로 회전 형상이 검증되지는 않는다.

## 구현 검증 — 2026-09-21

- `lekiwi_teleop` 패키지 빌드, Python 문법 검사, diff 검사 통과.
- `python3 -m unittest discover -s lekiwi_teleop/test -q`: 기존 회귀 포함 95개 통과.
- 로컬 ROS domain 91의 mock controller로 기본 20cm 시험 완료:
  명령 발행 10.000217초, 명령 적분 0.200004342m, odom 0.200000264m.
- 별도 mock 시험 주행 중 SIGTERM을 보내 중단·zero/토크 해제 서비스·실패 보고서 저장을 확인.
  모든 mock 프로세스는 종료했다. 실제 모터, Pi runtime, 바닥 주행은 실행하지 않았다.
- 로그: `/tmp/lekiwi-distance-trial-20260921-kjzjgjm8/`.
  mock odom 수치는 소프트웨어 연결 검증이며 엔코더·실제 주행 정확도 결과가 아니다.

## 첫 바닥 전진 시험 — 2026-09-21

사용자가 바닥 배치·시작점 표시·전방 공간·손 뗌·즉시 전원 차단 준비를 확인한 뒤,
전진 0.02m/s × 10초를 한 번 실행했다. Pi에도 도구를 설치하고 95개 테스트를 통과했다.
중심 거리 0.115m, 바퀴 반지름 0.0508m, speed_tick_scale 600, encoder_odom_scale 1은 유지했다.
시험용 controller 상한은 평면 0.03m/s, 회전 0.1rad/s, 바퀴 1rad/s였다.

| 항목 | 결과 |
| --- | --- |
| 명령상 목표 | 0.200m |
| 실제 명령 발행 시간 | 10.000790초 |
| 엔코더 odom 전진 변위 | 0.180506188m (18.0506cm) |
| 엔코더 odom 횡방향 변위 | 0.000162175m (0.162mm) |
| 엔코더 odom 방향 변화 | 수치상 약 0rad |
| odom 표본 | 788개 |
| 최대 명령 발행 간격 | 0.052944초 |
| 사용자 육안 관찰 | 약 18cm, 20cm보다 조금 덜 이동한 느낌 |
| 실제 줄자 측정 | 미확인 — 육안 추정과 구분 |

명령상 거리보다 odom이 약 1.95cm 작다. 실제 차체 이동거리의 오차는 실측 입력 전에는
판단하지 않는다. odom의 횡방향·회전 수치도 외부 실측 결과가 아니다.

시험·controller 프로세스는 모두 정상 종료했고 모터 포트 점유도 해제됐다. 종료 후
4.003초 동안 8회 직접 읽은 세 모터의 Torque_Enable·Goal_Speed·Present_Speed·status는
모두 0이며 엔코더 위치 범위도 모두 0 ticks였다. 자동 재시도나 대기 중인 동작은 없다.
사용자는 눈으로 정상 정지를 확인했다. 이동량은 육안으로 약 18cm라고 응답했으며
줄자·자로 측정한 수치는 아니다. odom과 대략 같은 경향이나 정확도 합격·보정 계수 산출에
사용하지 않는다. 원문과 측정 방법은 `user-observation.json`에 별도 기록했다.

- Pi 원본: `/home/roboseasy/youn_ws/diagnostics/floor-20260921-z3lcusbi/`
- PC 복사본: `/tmp/lekiwi-floor-20260921-z3lcusbi/floor-20260921-z3lcusbi/`
- `trial.json`: 도구의 odom·파라미터·시간 기록.
- `supervisor.json`: 별도 감독 프로세스의 시작/종료 레지스터·프로세스 종료 기록.
  도구 자체의 `torque_off_register_verified=false`와 별개로 직접 읽은 근거가 이 파일에 있다.
- `runtime.log`, `trial-console.log`: 실행 로그. 모터 제어 바이너리 SHA-256은 이전 공중 시험과 동일하다.
