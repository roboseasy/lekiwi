# LeKiwi ROS 2 작업 지침

이 브랜치는 ROS 2 기반 LeKiwi base의 수동 주행과 센서 확인을 다룬다.

## 현재 목표

- 브랜치: `feature/lekiwi-ros2`
- 2026-09-18 축소한 목표: 키보드 teleop, `/cmd_vel` pub/sub, 엔코더 기반 `/odom` 측정, 라이다 `/scan` 수신 확인.
- 2026-09-22 사용자 요청으로 참고 프로젝트 기반 Cartographer, 지도 저장, Omni AMCL·Nav2를 다시 추가한다. SOARM 제어는 제외한다. 실물 SLAM/Nav2 검증 완료와 코드 이식을 구분한다.
- 표시와 실물 bringup의 기본 모델은 교육용 `lekiwi_classroom_members`의 몸체+팔 복사본과 카메라 TF다. 진입점은 `lekiwi_description/urdf/lekiwi.urdf.xacro` 하나이며 팔 제어는 포함하지 않는다.
- 현재 빌드 대상: `lekiwi`, `lekiwi_description`, `lekiwi_bringup`, `lekiwi_node`, `lekiwi_sensors`, `lekiwi_teleop`, `lekiwi_cartographer`, `lekiwi_navigation2`.
- 활성 SLAM/Nav2 패키지는 저장소 루트에 있다. 과거 구현 사본은 Git 이력에서 확인한다.
- ROS 2 기준: 로컬 `ros2_ws`와 호환되는 colcon workspace 패키지

## 모드 구분

- `base mode`: LeKiwi base, wheel, teleop, encoder odometry, lidar, TF, robot description, base bringup을 포함한다.
- 모델 미리보기: `ros2 launch lekiwi_description display.launch.py`. 교육용 모델과 가상 joint state만 사용한다.
- 카메라 미리보기: 교육용 `camera_mounts.json` 복사본의 전방·손목 optical frame과 외형을 표시한다. 원본의 `calibrated: false`를 유지하며 카메라 드라이버는 실행하지 않는다.
- `soarm mode`: 제어 모드는 지원하지 않는다. 별도 브랜치는 보존한다. 최신 ZIP은 `exports/`에서 관리하고 `tools/export_model_bundle.py`로 재생성한다.
- 실물 bringup은 최신 모델에 `use_hardware_sensors:=true`를 적용한다. 주행·센서 좌표 공통 값은 `lekiwi_description/config/base_frame.yaml`, 변경 근거는 `lekiwi_description/docs/FRAME_MIGRATION.md`에 있다.
- 새 좌표계에서 실제 모터 방향과 센서 방향은 현장 검증 전이다. 기존 검증을 새 좌표계의 실물 검증으로 간주하지 않는다.

## 외부 리소스

- LeKiwi STL: `/home/ysj/Downloads/lekiwi_stl`
- 기본 교육용 모델 출처(읽기 전용): `/home/ysj/youn_ws/lekiwi_classroom_members/isaac_sim/assets/lekiwi_soarm`. 현재 복사본과 출처 기록은 `lekiwi_description/docs/IMPORT.md` 참고.
- SO101 description 참고: `/home/ysj/youn_ws/ros2_ws/src/so101_moveit_yolo/so101_moveit_yolo_description`
- YDLIDAR driver: `/home/ysj/youn_ws/ydlidar_ros2_ws/src/ydlidar_ros2_driver`
- YDLidar SDK: `/home/ysj/youn_ws/ydlidar_ros2_ws/src/YDLidar-SDK`
- Feetech driver 참고: `/home/ysj/youn_ws/ros2_ws/src/ros2_feetech_driver`

## 주의사항

- 표준 조립품은 BMI160을 포함하므로 Pi bringup과 Cartographer에서 IMU를 기본 사용한다. 장비별 축·정지 bias·TF 검증을 유지하고, IMU가 없는 장비는 Pi와 PC 양쪽에서 비활성화한다. EKF는 추가하지 않는다.
- 완료 기준은 README의 네 가지 기능 확인이다. `/odom`은 바퀴 회전량 기반 상대 이동 추정이며 지도 기준 localization과 구분한다.
- 모터 timeout, torque-off, `/motor_ready`, fault 처리는 유지한다. 범위 축소를 이유로 실제 장비 안전 처리를 제거하지 않는다.
- TurtleBot3 구조를 참고하되 TurtleBot 이름과 모델 전제는 사용하지 않는다.
- 기존 `feature/ee-gamepad` 작업은 `backup-ee-gamepad-before-lekiwi-ros2` 브랜치의 커밋 `8a69365`에 보존되어 있다.
- 외부 패키지는 직접 수정하지 않는다. 필요한 파일은 현재 repo로 복사해서 사용한다.

## 후속 개발 메모

- 아래 SLAM 메모는 과거 검토 기록으로 보존한다. 2026-09-22 범위 재확대에 따라 점검 대상이며, 현재 구현은 `docs/hardware/slam_navigation.md`를 따른다.
- 2026-09-12 사용자가 하부에 거꾸로 장착한 라이다의 가림 및 이동·회전 중 SLAM 불안정 검토를 후속 개발 항목으로 보관하도록 요청했다.
- 다음 라이다/SLAM 작업 전에 [후속 개발 메모](docs/hardware/lidar_slam_followup.md)를 읽는다. 좌우 반전은 아직 원인으로 확정하지 않았으며, 좌표·측정 시간·장착 시야·자체 가림 필터 순서로 검증한다.
- 메모의 상태는 개발 대기이며 실물 동작 승인을 의미하지 않는다. 아래 2026-06-19 검증 기록과 현재 구현 상태를 구분한다.

## 과거 검증 결과

아래는 초기 구현 당시 기록이며 현재 구현 설명이 아니다. Feetech 실제 통신과 BMI160 드라이버는 이후 추가되었다. 현재 범위와 검증은 README를 기준으로 한다.

2026-06-19 base mode 검증:

- `lekiwi_ros2` conda 환경에서 Python `3.12.13`, `rclpy` import 확인.
- `colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"`: 8개 패키지 build 성공.
- `colcon test --packages-select lekiwi_node --event-handlers console_direct+`: `lekiwi_node` 4개 gtest target, 총 18개 테스트 성공.
- 샌드박스 밖 권한에서 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 launch lekiwi_bringup robot.launch.py mode:=base use_lidar:=false` 실행 시 `robot_state_publisher`, `lekiwi_node` 정상 시작.
- `/robot_description`, `/odom`, `/tf_static`, `/tf` publish 확인.
- 외부 YDLIDAR workspace는 소스 수정 없이 `ydlidar_sdk` 선빌드 후 `ydlidar_ros2_driver` 선택 빌드로 성공했다.
  - SDK 빌드: `colcon build --symlink-install --packages-select ydlidar_sdk --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"`
  - Driver 빌드: `source install/setup.bash` 후 `colcon build --symlink-install --packages-select ydlidar_ros2_driver --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DCMAKE_EXE_LINKER_FLAGS="-L/home/ysj/youn_ws/ydlidar_ros2_ws/install/ydlidar_sdk/lib"`
  - 이유: driver의 `package.xml`에 `ydlidar_sdk` 의존성이 없고, SDK install config의 `YDLIDAR_SDK_LIBRARY_DIRS`가 비어 있어 전체 병렬 빌드와 기본 driver link가 실패한다.
- `/home/ysj/youn_ws/ydlidar_ros2_ws/install/setup.bash`와 driver `local_setup.bash` 존재 확인.
- `ls -l /dev/serial/by-id/* /dev/ttyUSB* 2>/dev/null || true` 출력 없음. 현재 라이다 serial device가 보이지 않아 `/scan` 실제 검증은 보류했다.
- 실제 YDLIDAR 모델과 실제 serial port는 미확인이다. `lekiwi_sensors/param/ydlidar.yaml`은 기본 port `/dev/ttyUSB0`, frame `lidar_link`를 유지한다.
- base motor backend 안전 기본값을 `motor_backend: mock`, `enable_motor_write: false`, `max_wheel_speed: 3.0`, `cmd_vel_timeout: 0.3`으로 고정했다.
- `motor_backend=feetech`와 `enable_motor_write=true` 조합은 Feetech skeleton backend를 선택한다. 현재 skeleton은 `std::fstream`으로 serial port 접근 가능 여부만 probe하고, packet format과 hardware 검증이 끝날 때까지 zero command만 허용한다. non-zero wheel command는 실제 write 없이 거부한다.
- 실제 Feetech non-zero motor write 검증은 speed packet format 확정, hardware backend 구현, 바퀴를 띄운 저속 테스트 전까지 보류한다.
- 샌드박스 내부에서는 ROS 2 DDS가 UDP socket/getifaddrs 권한 오류를 낼 수 있으므로 실제 토픽 검증은 로컬 권한에서 수행한다.
