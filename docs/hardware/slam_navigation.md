# SLAM·Nav2 사용과 검증

Pi에는 base·센서, PC에는 지도 작성·Nav2를 실행한다.
설정과 각 장비의 지도 작성·목표 주행 검증은 구분한다.

## 출처와 적용 범위

참고 저장소: [yeon-03/lekiwi-pill-pickup](https://github.com/yeon-03/lekiwi-pill-pickup),
commit `9f4a00cfd9dd15e31e73f96825eb86193ef2a20a`.
설정 출처는 `nav/config/lekiwi_cartographer.lua`, `nav/config/nav2_lekiwi.yaml`이며
수치·센서 처리 비교에는 `nav/nodes/lekiwi_base_node.py`, `nav/nodes/ydlidar_node.py`,
`nav/robots/lekiwi06/lekiwi_profile.sh`를 사용했다.
참조 스냅샷에는 별도 LICENSE 파일이 없었다. 원본 설정의 출처를 유지하며
현재 패키지의 라이선스 표기를 원본 저작물의 별도 라이선스 부여로 해석하지 않는다.

| 항목 | 변경 전 | 현재 |
| --- | --- | --- |
| wheel_radius | 0.0508m | 0.05m |
| base_radius | 0.115m | 0.13647m |
| speed_tick_scale | 600 | 4096/(2π) ≈ 651.898647 |
| raw 속도 상한 | 3240 | 1500 |
| 바퀴 각속도 상한 | 5.4rad/s | 1500×2π/4096 ≈ 2.300971rad/s |
| base x / y / planar 상한 | 0.26 / 0.15 / 0.26m/s | 0.11 / 0.10 / 0.11m/s |
| base 회전 상한 | 1.82rad/s | 0.75rad/s |

참고 base_radius는 회전 시험으로 맞춘 **유효 보정값**이다. 기계 치수나
URDF 바퀴 장착 위치를 바꿨다는 뜻이 아니다. 각 장비에서 바퀴 ID·방향,
유효 중심거리와 encoder_odom_scale을 확인한다. 명령 timeout, torque-off·fault 처리를 유지한다.

가져온 구성은 Cartographer의 스캔 정합 가중치·45 scan 서브맵, Omni AMCL,
5Hz MPPI(400 samples×35 steps), 속도 완화, Collision Monitor다.
최종 Nav2 속도는 `[0.08, 0.06, 0.25]` (m/s, m/s, rad/s)다.
동시에 여러 축을 명령하면 바퀴 상한에 따라 비례 축소될 수 있다.
차체 반경은 기존 0.22m, inflation은 0.35m를 사용한다. 팔까지 포함한 실제 점유 면적은 현장 확인 대상이다.
원본의 반경 0.18m, 방 지도, 자동 초기 위치 (0,0), 시간 지연 후 자동 실행, 약 집기·팔·도킹 코드는 가져오지 않았다.

## 라이다와 IMU

기본 센서 경로는 `/scan_raw → scan_timing_normalizer → /scan`을 유지한다.
SLAM/Nav2 launch는 `/scan → navigation_scan_filter → /scan_navigation`을 추가한다.
거리 0.19m 미만의 자기 반사값과 비정상 거리값은 NaN으로 마스킹한다.
가려진 방향을 자유 공간으로 지우도록 무한대 거리로 바꾸지 않는다.
필터 설정은 `lekiwi_sensors/param/navigation_scan.yaml`이다.

기본 자체 반사 제거 기준은 0.19m다. 조립된 장비에서 바퀴 반사 거리와
가까운 외부 물체가 지워지지 않는지를 각각 확인한다. Nav2의 AMCL·costmap 최소 거리도
같은 0.19m로 맞췄다. 로봇 반경 0.22m 설정과 실제 차체 외형의 관계는 주행 전 확인한다.

참고 로봇의 바퀴 가림 각도는 설정에 남겼지만 `mask_wheels: false`다.
각도 전체를 가리면 원거리 반사도 사라지므로 기본으로 켜지 않는다. 각도는 base_footprint 축 기준이며
필터가 TF의 전체 회전(거꾸로 장착한 roll 포함)을 적용한다. TF가 없으면 스캔을 내보내지 않는다.
intensity는 장비 차이가 있어 기본 0이다. 양수 임계값을 지정했는데 intensity가 없으면 입력을 거부한다.

SDK의 시간·각도 처리를 유지하기 위해 원본 serial 파서와 회전 deskew는 이식하지 않았다.
기존 time_increment=0 설정의 이동 중 왜곡은 여전히 실측 점검 대상이다.
필터는 시간·배열 순서를 바꾸지 않는다. 필터 뒤 유효 거리값이 하나도 없으면 출력하지 않아 Nav2의 센서 timeout으로 이어진다. 회전 속도를 낮췄다고 왜곡이 보정된 것은 아니다.

표준 조립품의 Pi bringup은 BMI160을 시작하고, 기본 Cartographer는 `/imu/data_raw`를 사용하는
`lekiwi_2d_imu.lua`를 선택한다. 출고 전에는 IMU 축·정지 bias와 TF를 장비별로 확인한다.
IMU가 없는 장비는 Pi `use_imu:=false`와 PC `configuration_basename:=lekiwi_2d.lua`를 함께
지정한다. IMU 입력이 없는 상태로 기본 프로파일을 실행하면 지도가 진행되지 않는다.

## 실행 순서

아래 명령은 README의 설치·빌드·ROS 환경 설정을 마친 PC 기준이다.
Pi의 base·센서 bringup과 실제 수동 주행은 README의 순서를 따른다.
Pi의 장비별 `base_params_file`과 PC의 Nav2 차체·속도 설정이 같은 기하·주행 한계를 나타내는지 확인한다.
하나의 ROS domain에서 Cartographer와 AMCL을 동시에 실행하지 않는다.

### 1. 지도 작성

Pi에서 `/odom`, `/scan`, `odom → base_footprint → base_link → soarm_base_link → lidar_link` TF가 나오는 상태에서:

```bash
ros2 launch lekiwi_cartographer cartographer.launch.py
```

이 명령은 필터·SLAM·occupancy grid·RViz를 실행한다. teleop은 별도로 실행한다.
RViz는 Fixed Frame `map`, Map `/map`, LaserScan `/scan_navigation`으로 열린다.
화면이 없는 PC는 `use_rviz:=false`를 지정한다.
SLAM 시작 전에 Pi와 PC의 시간이 동기화되어 있어야 한다.

### 2. 지도 저장

로봇을 정지한 뒤, Cartographer가 실행 중인 상태에서 새 PC 터미널에 ROS 환경을 설정한다.

```bash
mkdir -p ~/maps/lekiwi
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/lekiwi/map" --ros-args -p save_map_timeout:=10.0
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$HOME/maps/lekiwi/map.pbstream', include_unfinished_submaps: true}"
```

PGM/YAML은 AMCL용 지도이고 pbstream은 Cartographer 상태다. 각각 명령의 성공 응답과 실제 파일을 확인한다.
저장 후 teleop과 Cartographer를 종료한다. 모터 정지는 base 운용 절차에 따라 확인한다.

### 3. 지도 불러오기와 초기 위치

```bash
ros2 launch lekiwi_navigation2 navigation.launch.py
```

map_server·AMCL과 Nav2가 자동 활성화된다. RViz에서 **2D Pose Estimate**로 실제 위치와 방향을 지정한다.
기본 지도는 `~/maps/lekiwi/map.yaml`이며 다른 지도를 사용할 때만 `map:=/path/to/map.yaml`을 지정한다.
레이저가 지도 벽에 맞고 `map → odom` TF가 생겼는지 확인한다.
초기 위치를 (0,0)으로 가정하지 않는다. RViz의 Startup 버튼은 이미 활성화된 노드를 재시작하므로 누르지 않는다.

### 4. Nav2 활성화

launch가 RViz와 함께 모터 감독 도구를 시작한다. 실행 전 실제 이동 경로, 바퀴 정지, 사람·케이블,
즉시 전원 차단을 현장에서 확인한다. 감독 도구는 Feetech·엔코더 설정, 신선한
`/scan_navigation`·`/odom`, Nav2만의 `/cmd_vel` 발행 경로와 `/motor_ready` 변화를 확인하고
토크를 켠다. 상태가 나빠지거나 지정 시간이 끝나면 토크를 해제한다. 기본 상한은 1시간이며
`motor_guard_duration:=초`로 바꿀 수 있다. 지도만 볼 때는 `auto_arm_motors:=false`를 지정한다.
`Nav2 motor guard active`와 `/motor_ready=true`를 확인한 뒤 RViz의 **Nav2 Goal**로
짧은 목표 한 번을 지정한다. 목표를 보내면 실제 이동 명령이 발생한다.
목표 완료만으로 토크를 끄지 않는다.
`/motor_ready=false`이면 Goal을 보내지 않고 base의 고장 원인과 토크 상태를 확인한다.
Nav2 주행 중 별도 teleop이나 `/cmd_vel` publisher를 동시에 실행하지 않는다.
명령 경로는 `controller/behavior → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel`이다.
수동으로 `/cmd_vel`에 직접 보낸 명령은 이 충돌 감시 경로를 통과하지 않는다.
센서 가림 때문에 Collision Monitor가 모든 장애물을 볼 수 있는 것은 아니다.

### 5. 정지와 종료

RViz에서 진행 중인 goal을 취소하고 실제 정지를 확인한다. RViz를 닫거나 launch 터미널에서
`Ctrl-C`를 누르면 감독 도구가 토크를 해제한다. 실제 바퀴 정지와 base의 torque-off를 확인한다.
서비스 응답이나 프로세스 종료만으로 실제 바퀴 정지를 가정하지 않는다.

## 검증 구분

코드 검사·빌드·독립 ROS domain의 mock 기동과 실제 바닥 주행을 구분해 기록한다.
지도 저장, 위치 추정, 거리·회전 오차, 장애물 회피와 Nav2 목표 도달은
설치 장비와 공간에서 각각 확인한다. 소프트웨어 검사가 실물 주행 결과를 대신하지 않는다.

## 재현 가능한 소프트웨어 검사

저장소 루트에서 ROS와 workspace를 source한 뒤 실행한다.

```bash
colcon test --packages-select lekiwi_node lekiwi_bringup lekiwi_sensors lekiwi_cartographer lekiwi_navigation2
colcon test-result --verbose
/usr/bin/python3 tests/integration/test_navigation_mock.py
/usr/bin/python3 tests/integration/test_navigation_mock.py --imu
```

통합 검사는 빈 localhost domain 94에 mock base·합성 라이다를 실행한다.
다른 노드가 있으면 시작하지 않는다. Cartographer 지도 생성 → YAML/PGM·pbstream 저장 →
AMCL 초기 위치 → Nav2 활성화 → mock 목표 → 횡이동 명령 경로 → 스캔 끊김 정지를 검사한다.
`--imu`는 정지 합성 IMU로 IMU Lua 설정도 실행한다. 실제 IMU 축·잡음 검증은 아니다.
로그·지도는 `/tmp/lekiwi-navigation-mock-*`에 남고 생성한 프로세스만 종료한다.
