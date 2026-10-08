# 새 르키위 실증: 센서 점검 → SLAM → 저장 지도 → Nav2

노트북과 Pi는 **Ubuntu 24.04 / ROS 2 Jazzy**를 사용한다.
ROS 없는 센서 검사를 먼저 통과한 뒤 아래 순서로 진행한다.
Jazzy가 없으면 두 장비에 [ROS 2 Jazzy 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 먼저 완료한다.
최초 패키지 준비에는 두 장비 모두 인터넷 연결이 필요하다.

## 저장소 받기·업데이트와 시작 위치

**처음 사용하는 경우:** 원하는 작업 폴더에서 저장소를 복제하고 들어간다.

```bash
git clone -b feature/lekiwi-ros2 https://github.com/roboseasy-members/lekiwi.git
cd lekiwi
```

**이미 복제한 경우:** 해당 `lekiwi` 폴더에서 최신 변경을 받는다.
실행 중인 센서 검사·브링업·SLAM·Nav2는 먼저 종료한다.

```bash
git status --short
git pull --ff-only
git log -5 --oneline
```

`Already up to date.`이면 최신 상태다. 마지막 명령은 최근 변경의 커밋 목록을 보여준다.
IP를 편집한 센서 스크립트 등 로컬 수정이 있으면 업데이트가 중단될 수 있다.
충돌이나 오류가 나오면 **기존 수정·설정·지도는 보존하고 원인을 확인한다.**
임의로 초기화하거나 파일을 삭제하지 않는다. 업데이트 후 점검 대상 IP도 다시 확인한다.
ROS 패키지나 준비·실행 도구가 바뀌었으면 **4절의 자동 준비를 다시 실행**해
노트북과 Pi에도 반영한다. `git pull`만으로 Pi 소스·빌드가 업데이트되지는 않는다.

**노트북 명령은 모두 `lekiwi` 저장소 루트에서 실행한다.**
새 터미널은 홈(`~`)에서 열릴 수 있으므로 5절의 폴더 이동 명령을 붙여 넣는다.
설치·지도 경로는 현재 저장소에서 계산하므로 개인별 절대경로를 입력하지 않는다.

- 노트북 ROS 설치: `./install`, 외부 의존성: `../lekiwi_deps`
- Pi ROS 설치: 로그인 계정 홈의 `lekiwi_ws`
- 노트북 지도: `../maps/lekiwi`

## 필요한 터미널

처음에는 노트북 **B** 하나만 연다. 필요한 시점에 A·C·D를 추가해 **최대 4개 창/탭**을 사용한다.
Pi에 직접 연결한 화면이나 별도 터미널은 필요 없다.

| 터미널 | 실행 위치 | 용도 |
| --- | --- | --- |
| A | 노트북에서 SSH로 접속한 Pi | Wi-Fi 설정, 브링업. 마지막까지 유지 |
| B | 노트북 | IP·센서 검사, 자동 준비, 상태 확인, 지도 저장 |
| C | 노트북 | SLAM + RViz. 종료 후 같은 창에서 Nav2 + RViz |
| D | 노트북 | 수동 주행. 지도 저장 후 종료 |

각 실행 스크립트가 필요한 ROS 환경을 적용한다. 터미널마다 따로 `source`하거나
다른 창에서 설정한 변수를 복사할 필요가 없다.
**5절의 무선 접속 확인이 끝날 때까지 랜선을 유지한다.** 확인을 통과한 뒤 다음 절로 진행한다.

## 1. IP 취득과 접속 확인 — 노트북 B

르키위에 전원을 넣고 랜선을 노트북에 연결한다. 노트북의 유선 연결은
IPv4를 다른 컴퓨터와 공유하도록 설정돼 있어야 Pi가 IP를 받을 수 있다.

```bash
./tools/validation.sh network
```

`TYPE=ethernet`, `STATE=connected`인 줄의 `DEVICE`가 유선 장치다.
주소가 `10.42.0.1/24`이면 노트북의 공유망 주소이며, Pi 주소는 다음 검색으로 찾는다.

```bash
./tools/validation.sh scan
```

스크립트가 연결된 유선 장치와 대역을 찾아 검색한다. `nmap`이 없으면 설치한다.
유선 장치가 여러 개면 `./tools/validation.sh scan 실제장치이름`으로 지정한다.
`eno1` 등 다른 노트북의 장치 이름을 그대로 사용하지 않는다.

검색된 Pi 후보 주소로 접속한다. 아래 IP는 **실제 검색 결과로 바꾼다.**

```bash
./tools/validation.sh identify 10.42.0.136
```

**확인:** ping 응답과 해당 르키위의 hostname을 확인하고 제품 번호·IP를 기록한다.
검색 결과만으로 장비를 확정하지 않는다.

## 2. 테스트 스크립트의 IP 수정 — 노트북 B

현재 디렉토리가 `lekiwi`인 상태에서 연다.

```bash
nano ./lekiwi-sensor-check/check_robot.sh
```

다음 값을 제품에 맞춘다. 계정이 `roboseasy`이면 IP만 바꾼다.

```text
ROBOT_IP="10.42.0.136"
ROBOT_USER="roboseasy"
```

`Ctrl-O`, Enter로 저장하고 `Ctrl-X`로 종료한다.

## 3. IMU·라이다 테스트 — 노트북 B

ROS 센서 드라이버가 종료된 상태에서 로봇을 정지시킨다.
라이다 검사 중에는 스캐너가 회전한다. 제품 번호는 실제 번호로 바꾼다.

```bash
./lekiwi-sensor-check/check_robot.sh --unit lekiwi01
```

스크립트가 Pi에 점검 도구를 준비하고 검사한 뒤 결과를 노트북으로 가져온다.
SSH·sudo 비밀번호 요청에는 직접 입력한다. 최초 설치에는 Pi의 인터넷 연결이 필요하다.
실행 파일이 비어 있거나 손상됐으면 이전 빌드를 백업 없이 정리하고 새로 빌드한다.
빌드 검증이 실패하면 센서 측정 전에 중단한다.

**확인:** `[IMU] PASS`, `[LIDAR] PASS`, `IMU·라이다 자동 검사: PASS`와
`노트북 기록`, `판정 보고서` 경로가 모두 출력돼야 한다.
결과는 `lekiwi-sensor-check/reports/`에 저장되며 Git 관리에서 제외된다.
FAIL이면 보고서의 실패 원인을 먼저 확인한다. 센서 불량과 점검 도구 실행 오류를 구분한다.
장착 방향·거리 정확도는 현장에서 별도로 확인한다.

## 4. ROS 패키지 자동 준비 — 노트북 B

최초 준비나 ROS 소스·도구 업데이트 후 실행한다.
기존 브링업·SLAM·Nav2·센서 검사 앱은 종료한다.
접속 대상은 **2절에서 수정한 센서 스크립트의 IP·계정**이다.

```bash
./tools/validation.sh prepare
```

이 명령이 **현재 노트북의 소스 전송 → Pi 의존성·빌드·장치 그룹·설정 파일 준비 →
노트북 의존성·빌드**를 수행한다. Pi에서 따로 clone하거나 빌드 명령을 입력하지 않는다.
노트북 바이너리는 전송하지 않고 Pi에서 ARM용으로 빌드한다.
SSH·sudo 비밀번호 요청에는 직접 입력한다. 최초 빌드는 시간이 걸린다.

**확인:** `[pi] ROS 준비 완료`, `[pc] ROS 준비 완료`, 마지막 `ROS 준비 완료`가 나와야 한다.
같은 소스로 준비됐다면 설치·빌드를 생략한다. 실패하면 다음으로 넘어가지 않는다.
노트북 단계만 실패했다면 원인 해결 후 `./tools/validation.sh prepare --pc-only`로 재개한다.
기존 Pi 패키지나 외부 의존성이 다르면 보존하고 중단한다.
이전에 배포한 것으로 확인된 준비 도구는 갱신하지만, 사용자 수정은 덮어쓰지 않는다.
차이 때문에 중단되면 [준비 장애 안내](ros_setup_troubleshooting.md)를 본다.

Pi의 `lekiwi_ws/base_profile.yaml`은 없을 때만 예시를 복사한다.
**바퀴 ID·방향·크기·엔코더 보정값은 제품 검수값과 같아야 한다.**
새 제품에서 모터 ID·방향이 미확인이면 주행 전에 [공중 검증](encoder_odom_airborne_check.md)을 완료한다.
센서 PASS는 바퀴 검수 완료를 뜻하지 않는다. 기존 설정 파일은 보존한다.

## 5. Wi-Fi 연결 후 랜선 분리 — Pi A + 노트북 B

노트북을 사용할 Wi-Fi에 연결한다. **B**에서 현재 저장소의 폴더 이동 명령을 출력한다.

```bash
./tools/validation.sh folder
```

**A를 새로 열고 출력된 `cd && cd ...` 한 줄을 붙여 넣는다.**
나중에 여는 C·D에서도 같은 줄을 사용한다. 폴더 이동 후 **노트북 A**에서 Pi에 접속한다.

```bash
./tools/validation.sh ssh
```

접속 전은 노트북 계정(예: `ysj@ysj`), 접속 후는 Pi 계정(예: `roboseasy@lekiwi01`)이다.
**Pi A**에서 workspace로 이동하고 무선 주소를 확인한다.

```bash
cd lekiwi_ws
./src/lekiwi/tools/validation.sh wifi-show
```

예: `wlan0 UP 192.168.0.218/24`이면 **Pi Wi-Fi 주소는 `192.168.0.218`**이며 `/24`는 입력하지 않는다.
NetworkManager를 사용하면 연결 이름과 `*`로 표시된 현재 Wi-Fi의 SSID·모드도 나온다.
`lekiwi-ap`처럼 Pi 자체 AP에 연결된 상태는 사용할 공유기 Wi-Fi 연결 완료로 판단하지 않는다.
이미 원하는 Wi-Fi에 연결돼 IPv4 주소가 있으면 아래 **초기 설정·활성화 두 단계**를 생략한다.
주소가 없으면 **Pi A**에서 Wi-Fi 이름·암호를 입력한다. 암호는 화면에 표시되지 않는다.

```bash
./src/lekiwi/tools/validation.sh wifi-apply
```

`Do you want to keep these settings?`에서는 **아직 Enter를 누르지 않는다.**
180초 안에 **노트북 B**에서 연결을 활성화하고 Pi의 무선 주소를 확인한다.

```bash
./tools/validation.sh wifi-activate
```

IPv4 주소가 나오면 **Pi A로 돌아가 Enter**를 눌러 확정한다.
`Configuration accepted.`만으로 Wi-Fi 성공을 판단하지 않는다.
주소가 없거나 시간이 끝났으면 랜선을 유지하고 [Wi-Fi 장애 안내](ros_setup_troubleshooting.md#wi-fi-주소가-없을-때)를 따른다.

**노트북 B:** Pi의 무선 IPv4 주소를 입력하고 확인한다.

```bash
./tools/validation.sh wifi-check
```

**확인:** 무선 경로·ping·해당 Pi hostname 확인 완료가 나와야 한다.
`nmcli`에서 `TYPE=wifi`, `STATE=connected`인 줄의 `DEVICE`가 노트북 무선 장치다.
`ip route get`의 **`dev`가 그 장치**여야 한다. `src`는 노트북 주소다.
예: `192.168.0.218 dev wlp3s0 src 192.168.0.81`에서 목적지 `.218`은 Pi, `.81`은 노트북이다.
Pi에 주소가 여러 개면 이 확인을 통과한 주소 하나를 사용한다.

<details>
<summary>SSH 키 변경 경고가 나온 경우: 예전 IP 기록 제거</summary>

`REMOTE HOST IDENTIFICATION HAS CHANGED!`가 나오면 랜선을 유지하고 **노트북 B**에서 실행한다.
이 명령은 유선으로 접속한 Pi의 지문과 해당 Wi-Fi 주소의 지문을 비교한 뒤,
**일치하는 경우에만 해당 IP의 예전 기록을 교체**하고 재접속한다.

```bash
./tools/validation.sh ssh-reset
```

지문 일치와 해당 Pi hostname을 확인하면 **무선 경로 확인을 다시 실행**한다.

```bash
./tools/validation.sh wifi-check
```

지문이 다르거나 유선 접속도 실패하면 기록을 변경하지 않고 중단한다.
[SSH 키 충돌 처리](ros_setup_troubleshooting.md#ssh-키-변경-경고가-나올-때)를 따른다.

</details>

**Pi A:** 유선 SSH를 종료한다.

```bash
exit
```

**A의 프롬프트가 노트북 계정으로 돌아왔는지 확인한다.**
아직 `roboseasy@lekiwi01`이면 Pi 안에서 SSH를 더 연 상태이므로 `exit`로 그 접속도 종료한다.
**노트북 A**에서 B가 확인한 무선 주소로 접속한다. 주소는 파일로 공유돼 다시 입력하지 않는다.

```bash
./tools/validation.sh ssh wifi
```

**지금 랜선을 분리한다.** B의 무선 경로·ping·hostname 확인과 A의 무선 SSH 접속이
모두 성공한 경우에만 분리한다. **노트북 B**에서 랜선 없이 통신을 확인한다.

```bash
./tools/validation.sh wifi-ping
```

응답이 계속 오고 Pi A의 SSH가 유지돼야 한다. 끊기면 랜선을 다시 연결해 원인을 확인한다.
이후 센서 검사 재실행 시에는 센서 스크립트의 IP도 현재 접속 가능한 주소로 맞춘다.

## 6. 브링업 — Pi A

새 무선 SSH는 Pi 홈에서 시작하므로 **Pi A**에서 다시 이동하고 시각을 확인한다.

```bash
cd lekiwi_ws
./src/lekiwi/tools/validation.sh time
```

**노트북 B**에서도 확인한다.

```bash
./tools/validation.sh time
```

두 장비의 UTC 시각이 맞고 각각 `NTPSynchronized=yes`여야 한다.
**Pi A**에서 브링업을 시작한다. 초기 IMU 측정 동안 로봇을 정지시킨다.

```bash
./src/lekiwi/tools/validation.sh bringup
```

스크립트가 표준 모터 CH34 계열과 라이다 CP2102 포트를 각각 식별하고 연결·권한·점유 상태를 확인한다.
두 장치가 각각 하나가 아니면 중단한다. 다른 연결은 실제 배선을 확인한 뒤 포트를 지정한다.
IMU 기본 주소는 `0x68`(`104`)이다. 센서 보고서가 `0x69`이면 마지막에 `105`를 붙인다.

**확인:** 실제 모터·라이다 포트와 ROS 노드 시작 로그가 나오고 오류가 없어야 한다.
바퀴 토크는 꺼진 상태로 시작한다. **Pi A의 브링업은 마지막까지 유지한다.**

## 7. ROS 입력 확인 — 노트북 B

```bash
./tools/validation.sh check
```

**확인:** `/motor_ready=false`, `/scan` 약 10Hz, `/imu/data_raw` 약 100Hz,
`/odom` 수신과 `odom → base_footprint`, `base_footprint → lidar_link`,
`base_footprint → imu_link`의 세 TF가 나와야 한다. 엔코더 오류도 없어야 한다.
CLI 관측 주기는 네트워크 부하에 따라 달라질 수 있다.
관찰 시간 종료만으로 성공으로 처리하지 않으며, 실제 출력이 없으면 중단한다.
실패하면 브링업·무선 통신을 확인하고 다음으로 넘어가지 않는다.

## 8. SLAM 시작 — 노트북 C

C를 새로 열고 **5절에서 B가 출력한 폴더 이동 명령**을 붙여 넣은 뒤 실행한다.

```bash
./tools/validation.sh slam
```

**확인:** RViz의 `Fixed Frame=map`에서 주변 벽이 보이고 라이다 점이 실제 물체 방향과 맞아야 한다.
지도 없이 이동을 시작하지 않는다. 지도가 안 보이면 **노트북 B**에서 확인한다.

```bash
./tools/validation.sh map-check
```

`/scan_navigation`의 수신 주기와 크기가 0보다 큰 `/map` 정보가 나와야 한다.
`/scan`만 수신되면 필터 로그와 실제 거리값을 확인한다.
10Hz 수신만으로 유효한 거리 측정이 확인되지는 않는다.
IMU 축·회전 부호, 라이다 방향·가림도 제품별 검수와 일치해야 한다.

## 9. 수동 주행으로 지도 취득 — 노트북 D

D를 새로 열고 **5절의 폴더 이동 명령**을 붙여 넣는다.
**실제 토크가 켜지고 바퀴가 움직일 수 있다.** 바퀴 검수가 완료됐고,
경로에 사람·케이블·장애물이 없으며 손을 뗐고 즉시 전원을 끌 수 있는지 현장에서 확인한다.

```bash
./tools/validation.sh teleop
```

조작 창에서 `1`로 가장 낮은 속도를 선택하고 첫 이동은 짧게 실행해 방향·정지를 관찰한다.
`W/S`: 전후, `A/D`: 좌우, `Q/E`: 반시계/시계 회전,
**Space: 정지**, **Esc: 정지·토크 해제·종료**.
천천히 공간을 돌며 지도가 이어지는지 확인한다.
예상과 다르게 움직이면 중단하고 원인을 확인한다. 다른 teleop이나 Nav2를 동시에 실행하지 않는다.

## 10. 지도 저장 후 SLAM 종료 — 노트북 B·C·D

D의 조작 창에서 **Space로 정지**한 뒤 **B**에서 저장한다. **C의 SLAM은 아직 유지한다.**

```bash
./tools/save_map.sh
```

스크립트가 ROS 환경·지도 경로를 적용하고 YAML·PGM·pbstream을 저장한다.
기존 지도는 보존하며 같은 이름이 있으면 날짜를 붙인다.

**확인:** 지도 저장 성공, `/write_state` 성공 응답과 크기가 0보다 큰 세 파일을 확인한다.
마지막 **`Nav2에서 사용할 지도:` 안내가 나와야 완료**다.
YAML·PGM은 Nav2 지도, pbstream은 Cartographer 상태 보관용이다.
출력된 YAML 경로를 기록한다. 저장에 성공한 지도는 이 저장소에 기록돼 11절에서 자동 선택된다.
실패했으면 C의 SLAM을 유지하고 오류를 확인한다. 없는 파일을 저장 완료로 판단하지 않는다.

D에서 **Esc**로 조작 창을 종료하고 실제 바퀴 정지·토크 해제를 확인한다.
C에서 **Ctrl-C**로 SLAM과 해당 RViz를 종료한다. **Pi A의 브링업은 유지한다.**
B에서 종료 상태를 확인한다.

```bash
./tools/validation.sh safe-check
```

`/motor_ready=false`이고 이전 SLAM·주행 노드가 끝났어야 한다.
이 출력만으로 실제 토크 해제를 가정하지 않는다.
종료 처리가 실패하거나 장비 상태가 불명확하면 Nav2로 넘어가지 않는다.

## 11. 저장 지도로 Nav2 — 노트북 C

**Nav2는 한 번만 실행한다.** 실행 시 모터 감독 도구가 토크를 켤 수 있다.
이동 경로·사람·케이블·손·즉시 전원 차단 방법을 현장에서 확인한다.
C에서 방금 저장에 성공한 지도로 실행한다.

```bash
./tools/validation.sh nav2
```

출력의 **`사용할 지도:` 경로가 10절의 YAML과 같은지 확인한다.**
다른 지도를 사용하려면 `./tools/validation.sh nav2 실제지도.yaml`로 지정한다.
지도 화면만 확인할 경우 `./tools/validation.sh nav2-view`를 사용하며 목표 주행은 하지 않는다.

RViz에서 순서대로 진행한다.

1. **2D Pose Estimate**로 실제 지도 위치에서 로봇 전방을 향해 화살표를 그린다.
2. 라이다 점이 지도 벽과 맞는지 확인한다. **Startup은 누르지 않는다.**
3. 터미널의 **`Nav2 motor guard active`**를 확인한다. **B**에서 준비 상태를 확인한다.

```bash
./tools/validation.sh nav-check
```

**확인:** `/motor_ready=true`, `/amcl`·`/bt_navigator`·`/controller_server`가 `active`,
`/cmd_vel` 발행자가 `/collision_monitor` 하나여야 한다.
벽 정합과 준비 상태가 맞으면 **Nav2 Goal**로 가까운 목표 **한 번**을 지정한다.
`Goal succeeded`와 실제 도착·정지를 함께 확인한다.
실패한 목표를 자동으로 반복하지 않는다. Nav2 중에는 teleop을 실행하지 않는다.
도착 후에도 토크는 유지된다.

## 12. 종료와 기록

진행 중인 Goal을 취소하고 실제 정지를 확인한다.
**C에서 Ctrl-C**로 Nav2를 종료해 torque-off 처리와 바퀴 정지를 확인한다.
**B**에서 종료 상태를 확인한다.

```bash
./tools/validation.sh safe-check
```

`/motor_ready=false`를 확인한 뒤 **A에서 Ctrl-C**로 브링업을 종료한다.
정상 종료가 실패하거나 실제 움직임이 있으면 현장 전원 차단 절차를 따른다.
프로세스 종료만으로 토크 해제를 가정하지 않는다.

제품 번호·소스 버전·IP·센서 보고서 경로·실제 지도 경로·목표 도착·정지·torque-off 결과를 기록한다.
실제 관측이 없는 항목은 미검증으로 남긴다.
상세 설정·한계는 [SLAM·Nav2 안내](slam_navigation.md), 설치·Wi-Fi 문제는 [장애 안내](ros_setup_troubleshooting.md)를 본다.
