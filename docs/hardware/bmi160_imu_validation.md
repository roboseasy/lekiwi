# BMI160 IMU 배포 구조와 실물 검증

LeKiwi의 BMI160 지원은 라즈베리파이 Linux I2C와 ROS 2 사이를 연결하는 얇은 래퍼이며,
센서 초기화와 register 처리는 Bosch 공식 SensorAPI를 사용한다. Python SMBus 패키지나
비공식 ROS 라이브러리는 런타임 의존성에 포함하지 않는다.

## 배포 소스 구조

```text
lekiwi_sensors/
├── include/lekiwi_sensors/bmi160_conversion.hpp  # 단위·축 변환
├── launch/bmi160.launch.py                       # 배포 launch 진입점
├── param/bmi160.yaml                             # 하드웨어 기본값
├── src/bmi160_node.cpp                           # Linux I2C ROS 2 래퍼
├── test/                                         # 변환·배포 계약 테스트
└── vendor/bosch_bmi160/
    ├── bmi160.c                                  # Bosch 원본, 수정 없음
    ├── bmi160.h                                  # Bosch 원본, 수정 없음
    ├── bmi160_defs.h                             # Bosch 원본, 수정 없음
    ├── LICENSE                                   # Bosch BSD-3-Clause
    └── VENDOR_SOURCE.md                          # upstream commit와 SHA-256
```

공식 upstream은 `https://github.com/boschsensortec/BMI160_SensorAPI`이고,
배포본은 `VENDOR_SOURCE.md`에 기록한 commit으로 고정한다. 공식 네 파일을 수정하면 안 되며,
업데이트할 때는 commit, SHA-256, 테스트를 함께 갱신한다. 빌드된 설치 트리에는 Bosch
`LICENSE`와 `VENDOR_SOURCE.md`가 다음 경로로 설치되어야 한다.

```bash
test -f install/lekiwi_sensors/share/lekiwi_sensors/vendor/bosch_bmi160/LICENSE
test -f install/lekiwi_sensors/share/lekiwi_sensors/vendor/bosch_bmi160/VENDOR_SOURCE.md
```

## 하드웨어와 권한 사전 확인

배선은 전원을 끈 상태에서 변경한다. Raspberry Pi의 3.3 V I2C 전압을 사용하고, 보드의
전압 허용 범위가 불명확하면 전원을 넣지 않는다. 현재 검증 장비의 기본값은 bus 1,
주소 `0x68`이다.

```bash
sudo apt install -y i2c-tools
ls -l /dev/i2c-1
groups
sudo i2cdetect -y 1
sudo i2cget -y 1 0x68 0x00
```

주소 표에는 `68`, chip ID에는 `0xd1`이 보여야 한다. 일반 실행은 `/dev/i2c-1`에 접근 가능한
사용자 그룹에서 수행하며 ROS node를 `sudo`로 실행하지 않는다.

## IMU 단독 실행

센서를 고정하고 로봇을 움직이지 않은 상태에서 실행한다. 시작 뒤 기본 200 sample 동안
자이로 bias를 계산하므로 이 구간의 `/imu/data_raw` publish는 의도적으로 지연된다.

```bash
cd "$HOME/lekiwi_ws"
conda activate lekiwi_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch lekiwi_sensors bmi160.launch.py \
  i2c_device:=/dev/i2c-1 \
  i2c_address:=104 \
  frame_id:=imu_link
```

다른 터미널에서 확인한다.

```bash
timeout 10 ros2 topic hz /imu/data_raw
timeout 5 ros2 topic echo /imu/data_raw --once
```

정지 상태 수락 기준은 다음과 같다.

- topic rate: 95~105 Hz
- `header.frame_id`: `imu_link`
- 가속도 벡터 크기: 9.0~10.6 m/s²
- bias 보정 뒤 각 축 각속도 절댓값: 0.05 rad/s 이하
- `orientation_covariance[0]`: `-1.0` (BMI160 원시 데이터에는 자세 quaternion이 없음)

## 축 방향 확인

기본 `axis_map: [1, 2, 3]`은 센서 좌표를 그대로 사용한다. 실제 보드 장착 방향을 확인하기
전에는 이 값이 LeKiwi의 `imu_link`와 일치한다고 가정하지 않는다. 각 숫자의 절댓값은 원본
센서 축 1=x, 2=y, 3=z를 뜻하고 음수는 방향 반전이다. 예를 들어 `[2, -1, 3]`은 출력
x=원본 y, 출력 y=-원본 x, 출력 z=원본 z다.

로봇 모터 torque가 꺼진 상태에서 다음 세 관측으로 확정한다.

1. 수평 정지: `linear_acceleration.z`가 약 +9.81 m/s²가 되도록 축 순서와 부호를 맞춘다.
2. 정면을 유지하며 앞쪽을 살짝 들어 올림: LeKiwi 기준 pitch 축 반응을 확인한다.
3. 바닥에서 위를 보며 반시계 방향으로 수동 회전: `angular_velocity.z`가 양수인지 확인한다.

관측 중에는 motor command를 보내지 않는다. 축이 확정되면 `param/bmi160.yaml`의
`axis_map`과 URDF `imu_link`의 실제 위치를 갱신하고, 정지 기준을 다시 통과시킨다.

장비별 축 방향과 장착각은 위 절차로 확인하고 기록한다.

## Bringup과 Cartographer 연결

표준 조립품은 BMI160을 포함하므로 Pi의 기본 bringup에서 `use_imu:=true`다.
출고 전에 이 문서의 장치·축·정지 검사를 장비마다 통과시킨다. 표준 실행은 아래와 같다.
모터는 실물 버스에 연결되지만 토크는 꺼진 상태로 시작한다.

```bash
ros2 launch lekiwi_bringup robot.launch.py
```

Cartographer 기본값은 IMU용 `lekiwi_2d_imu.lua`다. Pi의 `/imu/data_raw`와 TF가
나오는 것을 확인한 뒤 로컬 PC에서 실행한다.

```bash
ros2 launch lekiwi_cartographer cartographer.launch.py
```

IMU 설정은 `tracking_frame = "imu_link"`, `use_imu_data = true`를 사용한다. 실행 전 로컬
PC에서 `/scan`, `/odom`, `/imu/data_raw`, `base_link -> imu_link` TF와 Pi-PC clock 동기화를
각각 관측해야 한다.
