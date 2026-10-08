# 개발용 Teleop 및 토픽 통신 진단

## 선택 설치

진단 도구의 소스는 `lekiwi_bringup/tools/diagnostics/`에 있다. 기본 설치 옵션
`LEKIWI_INSTALL_DIAGNOSTICS=OFF`에서는 검사 실행 파일과 `topic_test.launch.py`를
설치하지 않는다. 주행·SLAM·Nav2에 필요한 패키지는 [README](../../README.md)대로 설치한다.
현장 진단이 필요한 PC와 Pi에서 기존 workspace를 빌드한 뒤 선택 설치한다.

```bash
cd ~/lekiwi_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --executor sequential --packages-select lekiwi_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DLEKIWI_INSTALL_DIAGNOSTICS=ON
source install/setup.bash
```

이후 아래의 `ros2 run`·`ros2 launch` 진단 명령을 사용할 수 있다. 기존 설치에 추가한
도구는 옵션을 OFF로 바꾸는 것만으로 삭제되지 않으므로, 고객용 산출물은 새 workspace에서
기본 옵션으로 생성한다. 진단 결과는 `~/.ros/lekiwi-tests/`에 저장한다.

## ROS 2 service 진입점 (2026-09-21)

설치와 ROS 환경 설정은 [README](../../README.md)를 따르고, 기능별 검사 절차는 이 문서를 따른다.
`ros2 run lekiwi_bringup test_service_server.py`를 실행하면
`/lekiwi_test/{all,teleop,cmd_vel,odom,lidar,camera,camera0,camera2}/{start,status,cancel}`을 제공한다.
기본 일괄 실행은 Pi의 `ros2 launch lekiwi_bringup topic_test.launch.py`와 PC의
`ros2 run lekiwi_bringup test_all.py` 두 터미널만 사용한다. PC 명령이 서버 실행, `/all/start`
호출, `/all/status` 조회, 항목별 결과 출력과 서버 종료를 담당한다.
전체 검사는 teleop·cmd_vel 뒤 odom·라이다·카메라를 관찰하며, 개별 실패가 다른 검사를 막지 않는다.
취소 시 완료된 결과는 보존하고 나머지는 미완료로 표시한다. `/all/cancel`로 전체 작업을 취소한다.
현재 PC의 Fast DDS 기본 전송 설정에서 명령 미수신이 재현되므로 검사 터미널에
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를 함께 적용한다. 상세 환경 설정은 README를 따른다.
각 서비스는 `std_srvs/srv/Trigger`이며 기본 응답 `message`는 한국어 한 줄 요약이다.
`[진행 중]`, `[통과]`, `[실패]`와 검사 내용 또는 실패 원인을 표시하고, 최종 응답에는
`report.json` 경로를 붙인다. 서버 터미널에는 상태가 바뀔 때 같은 요약이 자동 출력된다.
start의 성공은 접수, status의 성공은 조회 성공이며 **`[통과]`가 자동 검사 통과**다.
전체 JSON은 `status.json`·`report.json`에 보관한다. 기존 JSON 응답이 필요한 도구는 서버에
`-p response_format:=json`을 지정하고 status의 `state=PASSED`를 확인한다.
실물 방향·거리·시야·카메라 식별은 `manual_checks`로 남는다.

기본 관찰 시간은 30초이며 수신율·최대 수신 공백·header 간격을 기록한다.
기본 최대 공백은 0.5초다. 아래 날짜별 실행 결과의 8초·12초·15초는 당시 조건이다.
새 teleop 검사는 기존 설치 실행 파일의 키 입력 검사 뒤, production guarded 프로파일의
속도 단계·방향 전환 zero·Space를 mock에 발행해 검사한다. 실물 readiness/토크 수명주기
검사를 mock 통과로 대신하지 않는다. 실물 20cm 시험은 `distance_trial.md`에 별도 기록한다.

## 검사 코드

- `lekiwi_bringup/tools/diagnostics/test_ros_topics.py`: 실행 중인 ROS 노드와 실제 메시지를 주고받아 검사한다.
- `lekiwi_bringup/tools/diagnostics/topic_test.launch.py`: Pi에서 mock base와 실제 라이다·USB 카메라 두 대를 실행한다.
  선택 설치 시 `lekiwi_bringup`의 launch로 제공한다. 기본 600초 후 종료하며 모터 serial을 열지 않는다.
- `lekiwi_bringup/tools/diagnostics/test_all.py`: 한 터미널에서 ROS 서비스 서버와 클라이언트를 실행하고 전체 결과를 출력한다.
- `lekiwi_bringup/test/diagnostics/test_ros_topic_checks.py`: 실물 설정, 무효 거리값, 손상 영상,
  갱신되지 않는 timestamp, 수신 중단을 성공으로 처리하지 않는지 확인한다.

| 검사 | 통과 기준 |
| --- | --- |
| `teleop` | 설치된 teleop 실행 파일에 가상 터미널로 w/s/a/d/q/e 입력 → `/cmd_vel`와 mock `/odom` 일치. Space, 입력 timeout, 움직이는 명령 중 x 종료 후 zero 확인 |
| `cmd_vel` | 별도 publisher의 x/y/yaw 복합 명령 → Pi 제어 노드의 mock odom 응답, timeout 정지 및 pub/sub 연결 확인 |
| `lidar` | `/scan`, `/scan_raw`에서 새 timestamp와 유효 거리, `lidar_link`, 최소 5Hz 확인 |
| `camera` | 두 `/image_raw/compressed`에서 JPEG 실제 디코딩, 640×480, 새 timestamp, 최소 10Hz, `/camera_info`와 frame·해상도 일치 |

자동 명령 검사는 `motor_backend=mock`, `enable_motor_write=false`, `torque_enable=false`,
`odom_source=command`를 원격 조회한 뒤에만 실행한다. 다른 명령 publisher나 예상하지 않은
`/cmd_vel` subscriber가 있으면 거부한다. 실물 이동과 엔코더 정확도 검사를 대신하지 않는다.
`--checks lidar camera`는 수신만 하며 명령 publisher를 만들지 않는다.
카메라 이름 0/2는 장치 번호이며 전방·손목 구분은 아직 하지 않았다.

## 재실행

먼저 기존 검사 노드를 종료한다. 아래 Pi와 PC 터미널의 domain과 RMW를 동일하게 맞춘다.
PC는 ROS 2 Jazzy와 LeKiwi가 설치되어 있어야 한다. 카메라 검사는 시스템 Python의
`cv2`, `numpy`를 사용한다(`python3-opencv`, `python3-numpy`).

Pi 터미널(`ROBOT_HOST`와 workspace 경로를 설치 환경에 맞게 지정):

```bash
ssh "$ROBOT_HOST"
source ~/lekiwi_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch lekiwi_bringup topic_test.launch.py
```

PC 터미널:

```bash
cd "$HOME/lekiwi_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
cd src/lekiwi
/usr/bin/python3 lekiwi_bringup/tools/diagnostics/test_ros_topics.py --report /tmp/lekiwi-topic-test.json
```

Pi 자체에서 검사하려면 같은 환경에서 `~/lekiwi_ws/src/lekiwi`로 이동해 동일한 Python 명령을
실행한다. PC에서 실행해야 Wi-Fi를 통한 PC↔Pi 통신까지 확인할 수 있다.
완료되면 Pi의 launch 터미널에서 Ctrl-C로 종료한다. 자동 제한 시간은 기본 600초다.

검사를 골라 실행하거나 관찰 시간을 늘릴 수 있다.

```bash
/usr/bin/python3 lekiwi_bringup/tools/diagnostics/test_ros_topics.py --checks teleop cmd_vel
/usr/bin/python3 lekiwi_bringup/tools/diagnostics/test_ros_topics.py --checks lidar camera --duration 15
/usr/bin/python3 -m unittest discover -s lekiwi_bringup/test/diagnostics -v
```

각 검사의 PASS/FAIL, 수신 수·주기·frame, pub/sub 노드, 실패 이유가 JSON에 저장된다.
전체 통과는 종료 코드 0, 하나라도 실패하면 1이다. 카메라 드라이버 종료 처리는 수신 검사의
범위에 포함하지 않으므로 launch 로그의 종료 코드도 별도로 확인한다.

## 2026-09-22 카메라 종료 오류 대응

Pi의 Ubuntu 24.04 / ROS 2 Jazzy, `usb_cam 0.8.1`, OpenCV 4.6.0,
TBB 2021.11 조합에서 영상 수신 후 종료할 때 `SIGSEGV(-11)`가 재현됐다.
카메라 한 대만 실행해도 발생했다. GDB의 충돌 주소를 종료 직전 메모리 배치와
비교하니 `libtbb.so.12.11`의 실행 영역이었고, 충돌 시점에는 해당 영역이 이미
해제되어 있었다. 남은 작업 스레드가 해제된 TBB 코드를 실행하는 종료 오류다.
아래 09-21의 단발 정상 종료 기록만으로 이 문제를 해결된 것으로 판단하지 않는다.

`cameras.launch.py`는 각 카메라 `Node`의 `additional_env`에
`LD_PRELOAD`로 `libtbb.so.12`를 추가해 프로세스 종료까지 유지한다.
기존 `LD_PRELOAD` 값과 순서를 보존하고 부모 launch의 환경을 변경하지 않는다.
따라서 이 launch를 포함하는 전체 검사에도 자동 적용되며, 사용자가 별도로
환경 변수를 export할 필요는 없다. `lekiwi_sensors`는 `tbb` 실행 의존성을 선언한다.
현재 Jazzy/Ubuntu 24.04 환경의 종료 오류를 피하는 우회 설정이며, 라이브러리
업데이트 후에는 영상 구독과 정상 종료를 함께 재검증해 제거 여부를 판단한다.

진단 비교에서 `OPENCV_FOR_THREADS_NUM=1`만 적용한 경우에는 같은 오류가 발생했다.
TBB 선행 로드는 단일 카메라의 약 30fps 수신과 정상 종료를 통과했다.
원인 분석과 비교 로그는 Pi의
`~/youn_ws/diagnostics/camera-exit-20260922-01i43u1q/`에 보존한다.

최종 코드를 PC와 Pi에 빌드하고 기존 `lekiwi_sensors` 검사를 통과했다.
Pi의 설치된 `ros2 launch lekiwi_sensors cameras.launch.py`로 실행한 뒤
PC에서 60초간 관찰한 결과는 다음과 같다.

| 항목 | 카메라 0 | 카메라 2 |
| --- | --- | --- |
| 영상 수신 | 1,798장, 29.97Hz | 1,798장, 29.97Hz |
| 최대 수신 공백 | 0.082초 미만 | 0.073초 미만 |
| 영상·정보 | 640×480 JPEG 디코딩 및 camera_info 일치 | 동일 |
| 종료 | 정상 종료 | 정상 종료 |

실행 중 `/proc` 환경으로 카메라 두 프로세스에만 선행 로드가 적용된 것을 확인했고,
종료 후 카메라 장치 점유가 해제됐다. 기존 `LD_PRELOAD` 값 보존과 부모 환경 유지도
launch 치환 검사로 확인했다. 최종 수신·종료 로그와 변경 전 파일 백업은 Pi의
`~/youn_ws/diagnostics/camera-tbb-deploy-20260922-96iblecv/`에 보존한다.

수신 검사 서비스의 통과 범위에는 드라이버 종료가 포함되지 않는다.
종료 검사는 launch 부모에 SIGINT를 한 번 보내고, 두 카메라의
`process has finished cleanly`와 장치 점유 해제를 별도로 확인한다.
보정 파일 미등록 및 지원되지 않는 카메라 제어 항목 경고는 별도 항목으로 남는다.

## 2026-09-21 ROS 서비스 검증

PC의 서비스 서버/검사 worker와 Pi의 mock base·실제 센서를 연결했다.
ROS domain 96, Fast DDS, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 조건이다.
센서와 odom 관찰은 서비스별 30초이며, 모터 serial은 사용하지 않았다.

| 서비스 | 결과 | 관측값 |
| --- | --- | --- |
| `teleop` | 통과 | 6방향, timeout·Space·종료 정지, guarded 속도 단계·방향 전환 |
| `cmd_vel` | 통과 | x/y/yaw 복합 명령 수신, mock odom 응답, timeout 정지 |
| `odom` | 통과 | 1,496개, 49.88Hz, 최대 수신 공백 0.182초. `odom_source=command` |
| `lidar` | 통과 | `/scan` 301개, `/scan_raw` 300개, 각각 10.02Hz, 최대 공백 0.117초 미만 |
| `camera` | 통과 | 0번 899장·29.97Hz, 2번 900장·29.99Hz, 최대 공백 0.063초 |
| `camera0` | 통과 | 900장·30.01Hz, 최대 공백 0.055초 미만 |
| `camera2` | 통과 | 900장·30.00Hz, 최대 공백 0.062초 미만 |

카메라 JPEG 디코딩, 640×480, `camera_info`의 frame·해상도 일치도 통과했다.
두 카메라 모두 내부 보정값은 미설정(`calibrated=false`)이다. 라이다의 마지막 유효 거리
범위는 약 9.0~16.1cm였으며, 수신 통과를 가림 없는 시야나 실물 방향 검증으로 보지 않는다.
이 odom 검사는 mock 값이다. 실제 엔코더/바닥 거리 기록은 [거리 시험](distance_trial.md)을 따른다.

추가 확인:

- 진행 중 다른 검사를 시작하면 `BUSY`, 카메라 검사 취소 시 `CANCELLED`를 반환했다.
- 센서가 없는 별도 domain에서 `/scan` 수신 0개와 `FAILED`를 확인했다.
- 서버가 준비되기 전에 클라이언트가 대기해도 시작하며, 이미 실행 중인 서버가 있으면
  새 서버는 종료 코드 1로 거부했다. 다른 결과 폴더로 실행한 중복 서버도 포함한다.
- 서버 종료 후 검사 worker/키보드 자식 프로세스가 남지 않았다.
- PC Python 검사 144개(teleop 95, 검사 도구 20, bringup 29), Pi 검사 도구 20개 통과.
  PC/Pi의 bringup·sensors 빌드와 PC의 두 패키지 `colcon test`도 통과했다.
- PC Fast DDS 기본 전송 설정에서는 mock 명령 미수신이 재현됐다.
  UDP 전용 설정 및 별도 Cyclone DDS 비교 검사에서는 서비스 통합 검사가 통과했다.

원본은 Pi `~/youn_ws/diagnostics/service-validation-20260921/`에 보존한다.
센서 실행 로그는 `~/youn_ws/diagnostics/service-sensors-tz_5w_yf/runtime.log`다.
주행 방향·모터 보정값·거리 scale은 이번 변경에서 수정하지 않았다.

종료는 수신 판정과 별도로 확인했다. 일괄 실행의 프로세스 그룹 전체에 SIGINT를 보냈을 때는
launch가 자식에게 신호를 다시 전달해 일부 종료 코드가 -2였다. 이후 카메라만 실행하고
launch 부모에 SIGINT를 한 번 전달한 확인에서는 두 카메라 모두 정상 종료했다.
해당 로그는 `~/youn_ws/diagnostics/camera-shutdown-nf6y9up5/runtime.log`다.
09-18의 SIGSEGV 기록은 과거 관측으로 남기며, 이번에 외부 드라이버를 수정한 것은 아니다.
