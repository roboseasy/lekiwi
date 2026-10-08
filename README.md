# LeKiwi ROS 2

LeKiwi의 바퀴 제어, 엔코더 주행 거리, YDLIDAR, BMI160 IMU, 카메라, 수동 주행, Cartographer 지도 작성과 Nav2 주행을 위한 ROS 2 Jazzy 소스 패키지입니다. PC에서 RViz와 조작 도구를 실행하고, 로봇의 Raspberry Pi에서 모터·센서를 실행합니다. 팔 구동은 포함하지 않습니다.

**대상:** Ubuntu 24.04 / ROS 2 Jazzy의 PC와 Raspberry Pi, Feetech 바퀴 모터 3개, YDLIDAR, BMI160이 장착된 LeKiwi. 출고된 장비의 포트·모터 ID·방향·센서 장착값을 먼저 확인해야 합니다. 이 저장소의 수치는 모든 조립품의 보정 완료값이 아닙니다.

아래 순서는 설치부터 지도 저장, Nav2 목표 한 번의 시험과 정지까지 이어집니다. PC와 Pi는 같은 네트워크에 연결하고 두 장비의 시간을 동기화하며, 로봇 배터리가 충분한 상태에서 시작합니다. **실제 이동 전에 사람·케이블·장애물이 경로에 없는지, 로봇에서 손을 뗐는지, 즉시 주 전원을 끌 수 있는지 현장에서 확인하세요.** 모터 토크는 명시적인 조작 전까지 켜지지 않습니다.

새 조립품의 IP 확인, 독립 센서 점검, 현재 노트북·Pi의 설치 준비부터 진행하려면 [처음부터 실증하는 절차와 명령어](docs/hardware/first_robot_validation.md)를 따르세요.
이 가이드는 센서 점검 후 `./tools/validation.sh prepare`로 Pi 소스 전송·빌드와 노트북 패키지 준비를 묶습니다. 브링업·SLAM·Nav2도 단계별 한 줄로 실행하고, 지도는 `./tools/save_map.sh`로 저장합니다.

ROS 없이 실행하는 [IMU·라이다 점검 도구](lekiwi-sensor-check/README.md)도 이 저장소의
`lekiwi-sensor-check/`에 포함됩니다. LeKiwi 저장소를 한 번 복제하면 센서 점검과
SLAM·Nav2 소스를 함께 받습니다. 노트북의 저장소 루트에서 IP를 수정하고 실행합니다.
로봇을 정지시키고 기존 센서 앱을 정상 종료한 상태에서 검사하세요.

```bash
nano ./lekiwi-sensor-check/check_robot.sh
./lekiwi-sensor-check/check_robot.sh --unit lekiwi01
```

## 1. PC와 Pi에 설치

두 장비에 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 설치한 뒤, Conda를 비활성화한 Bash 터미널에서 각각 실행합니다. 아래는 새 `~/lekiwi_ws` 기준입니다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake \
  python3-colcon-common-extensions python3-vcstool python3-rosdep \
  python3-opencv python3-numpy ros-jazzy-usb-cam ros-jazzy-rmw-fastrtps-cpp
source /opt/ros/jazzy/setup.bash
mkdir -p ~/lekiwi_ws/src
cd ~/lekiwi_ws/src
git clone -b feature/lekiwi-ros2 https://github.com/roboseasy-members/lekiwi.git
vcs import . < lekiwi/lekiwi.repos
cd ~/lekiwi_ws
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

`lekiwi.repos`는 YDLIDAR SDK와 ROS 드라이버의 검증 대상 커밋을 고정합니다. 드라이버의 SDK 의존성·링크 설정 때문에 다음 순서로 빌드합니다. Pi 메모리를 위해 병렬 작업을 제한합니다.

```bash
cd ~/lekiwi_ws
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --symlink-install --executor sequential --packages-select ydlidar_sdk \
  --cmake-args -DBUILD_EXAMPLES=OFF -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE -DSWIG_FOUND=FALSE
source install/setup.bash
colcon build --symlink-install --executor sequential --packages-select ydlidar_ros2_driver \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_EXE_LINKER_FLAGS="-L$HOME/lekiwi_ws/install/ydlidar_sdk/lib"
source install/setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select lekiwi lekiwi_description lekiwi_bringup lekiwi_node \
  lekiwi_sensors lekiwi_teleop lekiwi_cartographer lekiwi_navigation2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 pkg prefix lekiwi_bringup
```

Pi에서만 장치 그룹을 설정한 뒤 로그아웃하고 다시 로그인합니다. BMI160을 쓰는 경우 `/dev/i2c-1`의 그룹도 확인해 사용자 계정에 추가합니다.

```bash
sudo usermod -aG dialout,video "$USER"
ls -l /dev/i2c-1
# 위 장치의 그룹이 i2c라면: sudo usermod -aG i2c "$USER"
```

## 2. 장비별 설정과 ROS 통신

Pi에서 출고된 모터 ID·방향, 바퀴 크기, 엔코더 보정값에 맞춘 설정 파일을 **저장소 밖**에 둡니다. 예시의 ID·방향은 다른 장비에 그대로 적용할 값이 아닙니다. `base_params_file`은 바퀴 ID·방향·반지름과 엔코더 보정값을 읽습니다. 모터 포트와 속도 상한 등은 launch 인자가 같은 이름의 YAML 값을 덮습니다. 엔코더 보정값은 launch 인자를 기본값과 다르게 명시한 진단 실행에서만 그 인자가 우선합니다.

```bash
cp ~/lekiwi_ws/src/lekiwi/lekiwi_bringup/param/base_profile.example.yaml \
  ~/lekiwi_ws/base_profile.yaml
ls -l /dev/serial/by-id/
```

`~/lekiwi_ws/base_profile.yaml`을 장비의 출고 설정에 맞게 편집합니다. 모터 포트는 위에서 확인한 `/dev/serial/by-id/...` 경로를 아래 Pi 실행 명령의 `serial_port`에 지정합니다. 라이다가 `/dev/ttyUSB0`이 아니면 `lidar_port`도 바꿉니다. IMU가 없는 장비는 뒤의 Pi/PC 명령을 함께 변경해야 합니다.

새 터미널을 열 때마다 **PC와 Pi 양쪽**에서 다음 환경을 적용합니다. 도메인 번호는 예시이며 두 장비에 같은 값을 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/lekiwi_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

## 3. Pi에서 로봇 시작

Pi의 새 터미널에 위 ROS 환경을 적용하고 실행합니다. 포트 경로는 실제 장비 값으로 바꿉니다. 모터 버스는 열지만 주행 토크는 꺼진 상태로 시작합니다.

```bash
ros2 launch lekiwi_bringup robot.launch.py \
  base_params_file:=$HOME/lekiwi_ws/base_profile.yaml \
  serial_port:=/dev/serial/by-id/실제-모터-포트 \
  lidar_port:=/dev/ttyUSB0
```

IMU가 없다면 Pi 명령에 `use_imu:=false`를 추가합니다. PC의 새 터미널에 2절의 ROS 환경을 적용하고 다음을 확인합니다. `hz` 명령은 수신 주기를 몇 번 확인한 뒤 `Ctrl-C`로 끝냅니다. IMU 사용 시 `/imu/data_raw`도 같은 방법으로 확인합니다. 시작 직후 `/motor_ready=false`는 토크가 꺼져 있어 정상입니다.

```bash
ros2 topic echo --once /motor_ready
ros2 topic hz /scan
ros2 topic hz /odom
```

센서 좌표와 바퀴 방향이 실제 조립 상태와 다르면 주행 전에 [장비 검수 절차](docs/hardware/base_operation.md)를 완료해야 합니다.

## 4. 지도를 화면에서 보며 작성·저장

PC의 새 터미널에 ROS 환경을 적용하고 Cartographer를 시작합니다. RViz가 함께 열리고 Fixed Frame `map`, Map `/map`, LaserScan `/scan_navigation`이 미리 설정됩니다. 시작 후 로봇을 잠시 정지시켜 IMU 표본을 안정화합니다.

```bash
ros2 launch lekiwi_cartographer cartographer.launch.py
```

IMU가 없는 장비는 이 명령에 `configuration_basename:=lekiwi_2d.lua`를 추가합니다. Pi의 `use_imu:=false`도 함께 설정해야 합니다. RViz가 필요 없는 PC에서는 `use_rviz:=false`를 지정할 수 있습니다.

이동할 경로와 전원 차단 방법을 현장에서 확인한 후, PC의 **별도 터미널**에서 실물용 조작 창을 엽니다. 이 옵션은 현장 확인을 대신하지 않습니다. `W/S`(전후), `A/D`(좌우), `Q/E`(회전)를 누르는 동안만 움직이며, `1`·`2`·`3`은 속도 단계, Space는 정지, Esc는 종료입니다. 1시간 상한은 방치 대비용이고, 지도를 다 그리면 사용자가 Esc로 종료합니다. 더 긴 현장 작업에는 승인한 시간만큼 `--max-session-duration` 값을 지정합니다.

```bash
ros2 run lekiwi_teleop lekiwi_guarded_base_teleop \
  --yes-i-confirm-motion --max-session-duration 3600
```

RViz에서 벽·장애물의 위치가 맞고 지도가 이어지는지 보면서 천천히 주행합니다. 지도가 완성되면 **로봇을 정지한 상태에서** PC의 새 터미널에 ROS 환경을 적용해 저장합니다.

```bash
mkdir -p ~/maps/lekiwi
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/lekiwi/map" \
  --ros-args -p save_map_timeout:=10.0
ls -l ~/maps/lekiwi/map.yaml ~/maps/lekiwi/map.pgm
```

저장 성공과 파일을 확인한 뒤 조작 창에서 Esc로 종료하고 바퀴 정지·토크 해제를 확인합니다. Cartographer 터미널은 `Ctrl-C`로 종료합니다. 지도를 계속 편집하려면 [SLAM·Nav2 상세 안내](docs/hardware/slam_navigation.md)를 봅니다.

## 5. 저장한 지도로 Nav2 주행

Pi의 로봇 bringup은 `torque_enable:=false`로 계속 실행하고 PC에서 Cartographer와 teleop은 종료합니다. 로봇이 정지했고 손을 뗐으며, 주변에 사람·케이블이 없고 즉시 전원을 끌 수 있는지 확인합니다. PC의 새 터미널에서 아래 명령을 실행하면 저장한 지도와 RViz가 열리고 Nav2가 활성화됩니다. 지도 파일이 없으면 launch가 오류를 표시합니다.

```bash
ros2 launch lekiwi_navigation2 navigation.launch.py \
  map:=$HOME/maps/lekiwi/map.yaml
```

같은 launch가 모터 감독 도구도 시작합니다. 이 도구는 Feetech/엔코더 설정, `/scan_navigation`·`/odom` 수신, `/cmd_vel`의 Nav2 단일 발행 경로와 `/motor_ready`를 확인한 뒤 토크를 켭니다. 터미널의 `Nav2 motor guard active`와 `/motor_ready=true`를 확인하세요. 검사 실패 시 토크는 켜지지 않습니다. 지도만 볼 때는 `auto_arm_motors:=false`를 지정합니다.

RViz의 **2D Pose Estimate**로 실제 로봇 위치·방향을 지정하고 레이저 점이 지도 벽과 맞는지 확인합니다. 이동할 경로를 다시 확인한 뒤 상단의 **Nav2 Goal**로 짧은 목표를 지정합니다. **Startup은 누르지 마세요.** 목표 완료 후에도 모터 토크는 유지됩니다. 센서·모터 이상, RViz 종료 또는 기본 1시간 감독 제한 시 토크가 해제됩니다. 더 긴 작업에는 `motor_guard_duration:=초`를 지정합니다.

종료할 때 진행 중인 Goal을 취소하고 바퀴 정지를 직접 확인한 뒤 RViz를 닫거나 Nav2 launch에 `Ctrl-C`를 누릅니다. `/motor_ready=false`이면 목표를 다시 보내지 말고 오류 원인을 확인하세요. 서비스 응답이나 프로세스 종료만으로 실제 바퀴 정지를 가정하지 마세요.

## 검사·지원 범위

- 라이다 드라이버의 `/scan_raw`는 `scan_timing_normalizer`를 거쳐 `/scan`으로 전달됩니다. 호환 설정인 `time_increment=0`은 이동 중 스캔 왜곡이 사라졌다는 뜻이 아니므로, 빠른 회전·주행에서 지도 품질을 현장 확인해야 합니다.
- 현장 진단 도구는 개발용 선택 설치로 분리했습니다. 모의 base와 실제 센서의 연결을 검사하며, **실물 주행·SLAM 지도 품질·Nav2 목표 도달 검사는 아닙니다.** 설치 방법과 기능별 서비스는 [개발용 진단 안내](docs/hardware/topic_tests.md)에 있습니다.
- 모델/TF 출처와 장비별 확인 항목은 [모델 출처](lekiwi_description/docs/IMPORT.md), [바닥 주행·센서 검수](docs/hardware/base_operation.md), [SLAM·Nav2 상세 안내](docs/hardware/slam_navigation.md)에 있습니다.
- ROS 소스의 라이선스는 [LICENSE](LICENSE), 가져온 자료는 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 참고하세요.
