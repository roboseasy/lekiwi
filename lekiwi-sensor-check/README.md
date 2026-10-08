# LeKiwi 센서 출고 점검

ROS 없이 **BMI160 IMU와 YDLIDAR Tmini Plus**의 통신·데이터 수신을 검사합니다. 노트북에서 IP를 지정해 Pi를 점검하거나, Pi에서 직접 실행할 수 있습니다. 제품마다 한 명령을 실행하고 PASS/FAIL과 측정 기록을 남깁니다. LeKiwi ROS 패키지 빌드, colcon workspace, Conda, pip 패키지가 필요하지 않습니다.

이 도구는 LeKiwi 저장소의 `lekiwi-sensor-check/`에 포함되므로 LeKiwi를 한 번 복제해 받습니다.
아래 사용 명령은 모두 **LeKiwi 저장소 루트**에서 실행합니다. `COLCON_IGNORE`로 ROS 빌드에서
제외되며, 점검 도구 폴더만 복사해도 독립 실행할 수 있습니다.

대상 환경은 Ubuntu 24.04 또는 Raspberry Pi OS의 Linux입니다. `/dev/i2c-1`과 CP2102 USB 직렬 장치를 사용합니다. 실제 검증은 Ubuntu 24.04 ARM64 Pi에서 진행해야 합니다. 현재 확인한 것은 노트북의 빌드·장비 없는 자동 테스트이며, 실물 출고 검증 완료를 뜻하지 않습니다.

## 노트북에서 IP로 점검

Linux 노트북과 Pi를 랜선 등으로 연결하고, Pi에 SSH로 접속할 수 있어야 합니다. 모든 제품의 계정이 `roboseasy`이면 `check_robot.sh`의 IP 한 줄만 바꿉니다. 계정이 다른 제품은 바로 아래 `ROBOT_USER`도 바꿉니다.

```bash
ROBOT_IP="10.42.0.136"
ROBOT_USER="roboseasy"
```

노트북 터미널에서 실행합니다. Pi에 미리 이 저장소를 복사하거나 별도로 로그인해 설치할 필요는 없습니다.

```bash
nano ./lekiwi-sensor-check/check_robot.sh
./lekiwi-sensor-check/check_robot.sh
```

스크립트는 다음 작업을 순서대로 진행합니다.

1. 지정한 IP의 Pi에 SSH로 접속합니다. 처음 접속하는 장비는 호스트 확인을 요청할 수 있으며 SSH 비밀번호는 터미널에서 입력합니다.
2. Pi의 `~/.cache/lekiwi-sensor-check/releases/<소스 해시>/`에 필요한 소스를 전송합니다. Pi에 빌드 도구가 없으면 `sudo apt-get update/install`로 설치합니다. 최초 실행·소스 버전 변경·실행 파일 손상 시 이전 `build` 폴더를 백업 없이 정리하고 Pi에서 전체를 새로 빌드합니다. 정상 ELF 형식과 센서를 작동시키지 않는 `--help` 실행을 확인한 뒤에만 빌드 완료로 표시합니다. 정상 빌드는 다음 검사에서 재사용합니다.
3. `sudo`로 센서 점유·권한을 확인하고 IMU와 라이다를 검사합니다. sudo 비밀번호를 물으면 Pi 계정 비밀번호를 입력합니다. I2C 그룹 변경이나 재로그인은 필요하지 않습니다. 검사 종료 후 해당 작업의 결과 소유권을 SSH 계정으로 돌려줘서 관리자 전용 폴더도 가져올 수 있게 합니다.
4. PASS/FAIL과 측정 기록을 노트북의 `lekiwi-sensor-check/reports/<시각>_<IP>_<구분자>/`에 복사합니다. Pi에도 해당 작업의 결과가 남습니다. 검사 기록·빌드·SDK 캐시는 Git 관리에서 제외됩니다.

**최초 설치에는 Pi에서 인터넷 접속이 필요합니다.** 랜선 연결만으로 인터넷까지 제공되는 것은 아닙니다. 필요한 도구와 같은 소스 버전의 빌드가 준비된 뒤에는 오프라인 점검이 가능합니다. 노트북에는 `/usr/bin/python3`, `ssh`, `scp`가 필요하며 OpenSSH가 없으면 `sudo apt install openssh-client`로 설치합니다. SSH 및 sudo 권한이 있는 Linux 계정이 필요합니다. 비밀번호를 스크립트에 적거나 저장하지 않습니다.

로봇은 정지한 상태로 두고, 기존 센서 앱을 정상 종료한 뒤 실행하세요. 라이다 검사 중에는 스캐너가 회전합니다. 바퀴·팔 모터 명령은 포함하지 않습니다. 측정은 센서가 연결된 Pi에서 수행하고, 실행 화면과 결과는 노트북에서 확인합니다.

파일을 수정하지 않고 IP를 지정하거나 제품 이름·센서 옵션을 전달할 수도 있습니다.

```bash
./lekiwi-sensor-check/check_robot.sh --ip 10.42.0.136 --unit lekiwi01
./lekiwi-sensor-check/check_robot.sh --ip 10.42.0.137 --unit lekiwi02 --only imu
```

`--duration`, `--imu-device`, `--imu-address`, `--lidar-port`, `--only`는 아래 Pi 직접 실행과 동일합니다. 원격 실행의 `--output`은 **노트북 결과 저장 위치**입니다. 제품 이름을 생략하면 Pi의 hostname을 사용합니다.

새 작업은 각각 다른 번호로 기록하므로 이전 결과를 덮어쓰지 않습니다. `connection.json`에는 대상 IP·계정·소스 해시와 수집 상태가 있고, 센서 판정은 `reports/.../report.json`에 있습니다. SSH 연결 또는 결과 수집에 실패하면 종료 코드 2를 반환하며, 센서가 정상이라고 판단하지 않습니다. 센서 실패도 결과를 가져와 종료 코드 1을 반환합니다. 준비 단계에서 실패하면 센서를 실행하지 않습니다.

Pi의 기존 `~/lekiwi-sensor-check`, ROS workspace와 센서 앱 설정을 덮어쓰지 않습니다. Pi별 잠금으로 이 원격 도구의 준비·측정을 중복 실행하지 않습니다. 실행 중인 다른 앱을 종료하지 않으며 실패한 센서 측정을 자동 재시도하지 않습니다. Ctrl-C나 SSH 터미널 종료 시 작업자는 자신이 시작한 측정기를 중단합니다. 통신 장애로 완료 기록을 가져오지 못할 때와 중단 후에는 라이다의 실제 회전 정지를 직접 확인하세요.

## Pi에서 직접 실행: 처음 한 번 설치

Pi에 LeKiwi 소스가 준비되어 있으면 저장소 루트로 이동한 뒤 실행합니다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3 psmisc util-linux
./lekiwi-sensor-check/setup.sh
```

`setup.sh`를 실행할 때마다 기존 `build` 폴더를 백업 없이 정리하고 SDK·측정 프로그램 두 개를 처음부터 빌드합니다. 기존 `.o`와 실행 파일을 재사용하지 않습니다. 소스·SDK 다운로드 캐시·검사 기록은 보존합니다. 빌드 후 두 프로그램의 정상 실행을 확인하고 디스크 쓰기를 동기화하며, 실패하면 설치 성공으로 표시하지 않습니다.

설치 중 인터넷으로 고정 버전 YDLIDAR SDK를 내려받습니다. SDK와 빌드 파일은 점검 도구 폴더 안에만 둡니다. 시스템 라이브러리 설치나 ROS 설정 변경은 하지 않습니다. SDK의 Python 바인딩도 설치하지 않습니다. 기본 빌드 병렬 수는 2이며 메모리가 부족하면 `CHECK_BUILD_JOBS=1 ./lekiwi-sensor-check/setup.sh`로 줄입니다.

장치 권한을 확인합니다.

```bash
ls -l /dev/i2c-1 /dev/serial/by-id/
id
```

I2C 장치 그룹이 `i2c`, USB 직렬 장치 그룹이 `dialout`인 Pi에서는 다음을 한 번 실행한 뒤 SSH를 종료하고 다시 로그인합니다.

```bash
sudo usermod -aG i2c,dialout "$USER"
```

이 도구는 `/usr/bin/python3`와 정적 C++ SDK를 사용하므로 `lerobot` 가상환경이 활성화되어 있어도 Python 패키지가 섞이지 않습니다. 설치 후 검사에는 인터넷이 필요하지 않습니다.

## Pi에서 직접 실행: 제품마다 점검

센서 드라이버·ROS bringup 등 I2C나 라이다 포트를 사용하는 앱을 먼저 정상 종료합니다. 로봇은 정지한 상태로 두고 라이다 주변 시야를 확보합니다. IMU를 초기화해 100Hz로 설정하며, 라이다 검사에서는 스캐너가 회전합니다. 바퀴·팔 모터 명령은 포함하지 않습니다.

```bash
./lekiwi-sensor-check/check_sensors.sh --unit lekiwi01
```

제품 이름을 생략하면 Pi의 hostname을 사용합니다. 기본 측정 시간은 센서별 8초이며 장치 초기화 시간은 별도입니다. 검사 전에 실행 파일을 검증합니다. 파일이 비어 있거나 손상됐다면 장치 접근 전에 종료 코드 2로 중단하므로 `setup.sh`로 다시 빌드합니다. IMU를 먼저 검사한 뒤 라이다를 검사합니다. IMU에서 실패해도 라이다 결과를 남깁니다. 검사 실패 후에는 결과의 원인을 확인하고 다시 실행하세요.

정상 결과의 예:

```text
[IMU] PASS
  실제 수신 주기: 100.00 Hz
[LIDAR] PASS
  실제 수신 주기: 10.00 Hz

IMU·라이다 자동 검사: PASS
기록: .../reports/20261007-..._lekiwi01_.../report.json
```

`PASS`는 아래 **자동 검사 범위**의 통과입니다. 아래 **별도 현장 확인**까지 완료해야 제품 검수 완료로 기록할 수 있습니다. 예시 결과는 실행 방법 설명용이며 실물 측정 기록이 아닙니다.

### 주소·포트 지정과 개별 검사

BMI160 주소는 `0x68`·`0x69`에서 식별값 `0xd1`을 확인해 자동 선택합니다. 두 주소에서 모두 BMI160이 응답하면 주소를 지정해야 합니다. 다른 버스는 자동으로 훑지 않습니다.

```bash
./lekiwi-sensor-check/check_sensors.sh --unit lekiwi02 --imu-address 0x69
```

라이다는 USB ID `10c4:ea60`의 CP2102 한 개만 자동 선택합니다. 복수의 어댑터가 있으면 실제 라이다의 안정된 장치 이름을 지정합니다.

```bash
./lekiwi-sensor-check/check_sensors.sh --unit lekiwi02 \
  --lidar-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

`ttyACM` 모터 포트와 다른 USB 칩은 거부합니다. 이 USB ID만으로 실제 연결된 장비를 완전히 식별할 수는 없으므로, CP2102 어댑터를 모터에 사용하는 다른 조립품에서는 라이다 포트를 현장에서 반드시 구분해야 합니다. SDK에서 Tmini Plus 모델 코드도 확인하며 다른 모델이면 검사를 중단합니다.

문제 분석을 위한 개별 검사:

```bash
./lekiwi-sensor-check/check_sensors.sh --unit lekiwi01 --only imu
./lekiwi-sensor-check/check_sensors.sh --unit lekiwi01 --only lidar
```

개별 검사 결과는 선택한 센서만의 PASS/FAIL입니다. 두 센서 출고 검사에는 `--only`를 생략합니다. `--duration 15`로 측정 시간을 늘릴 수 있으며 3~60초를 지원합니다.

## 자동 검사 범위

| 대상 | 기준 |
|---|---|
| IMU 식별 | BMI160 `0xd1`, 주소 `0x68` 또는 `0x69` |
| IMU 데이터 갱신 | 가속도·자이로 data-ready 확인, sensor-time 갱신, 원시값 변화 |
| IMU 수신 | 실제 새 표본 95~105Hz, 0.1초 초과 끊김 없음 |
| 정지 중력 | 가속도 벡터 크기의 평균 9.0~10.6m/s² |
| 정지 자이로 | 각 축 원시 평균 절댓값 0.05rad/s 이하 |
| 정지 흔들림 | 가속도 각 축 표준편차 0.20m/s² 이하, 자이로 0.015rad/s 이하 |
| 라이다 식별 | SDK 모델 코드 151 또는 152, Tmini Plus 계열 |
| 라이다 수신 | 10Hz 설정에서 실제 스캔 8~12Hz, 시각 증가 |
| 라이다 거리 | 0.05~12m의 유한한 거리값, 전체 유효 비율 10% 이상 |
| 라이다 유효 점 수 | 스캔의 90% 이상에서 유효 거리값 30개 이상 |
| 라이다 값 갱신 | 측정 구간 내 거리 배열 변화, 수신 오류 비율 10% 이하 |
| 공통 | 충분한 측정 시간·표본 수, 측정 프로그램 종료·cleanup 경로 확인 |

IMU는 bias를 자동으로 빼지 않습니다. 큰 정지 편차를 보정해 정상으로 숨기지 않고 원시 편차를 기록합니다. 정지 가속도 검사는 크기를 사용하므로 센서를 뒤집어 장착해도 이것만으로 실패하지는 않습니다.

원시값이 측정 구간 내내 완전히 동일하면 검토 대상으로 FAIL을 표시합니다. 이 검사는 센서의 전기적 고장을 단독으로 확정하지 않습니다. 실제 정지 환경·필터·해상도와 기록을 함께 확인하세요.

판정 수치는 `check_sensors.py`의 `LIMITS`에 있고 각 보고서에도 저장됩니다. 정지 IMU의 기존 검수 범위를 참고했으며, 추가 흔들림·라이다 범위는 이 도구의 초기 검수 기준입니다. 실제 정상 제품과 결함 사례로 확인한 뒤 출고 기준으로 확정하세요.

## 검사 기록

매번 새로운 폴더를 만들어 기존 결과를 덮어쓰지 않습니다. `--output /path/to/reports`로 기록 위치를 바꿀 수 있습니다.

| 파일 | 내용 |
|---|---|
| `report.json` | 제품 이름, 장치·센서 식별, 시각, 기준값, 측정 통계, 실패 항목 |
| `imu.csv` | 새 IMU 표본의 시각·가속도(m/s²)·각속도(rad/s) |
| `lidar.csv` | 스캔 시각·전체/유효 점 수·최소/최대 거리 |
| `last_scan.json` | 마지막 스캔의 유효 점: `[각도(rad), 거리(m)]` |
| `imu.log`, `lidar.log` | 측정 프로그램 출력과 SDK 진단, 전체 측정 데이터 |

장치가 없거나 권한·점유 검사에서 막힌 경우에도 `report.json`에 실패 이유를 기록합니다. 측정 전 실패한 센서는 CSV가 생성되지 않습니다. 도구 설치 미완료, 동시 검사, 기록 경로 오류는 실행 오류로 안내합니다.

종료 코드는 `0=선택한 모든 검사 PASS`, `1=센서 검사 FAIL`, `2=설치·실행 환경 오류`, `130=사용자 중단`입니다. 생산 관리 스크립트에서 이 코드와 JSON을 읽을 수 있습니다.

## 별도 현장 확인

다음 항목은 자동 수신 검사로 검증되지 않습니다. 보고서의 `manual_checks_remaining`에도 남깁니다.

- IMU의 차체 기준 축·장착각과 반시계 회전 시 각속도 부호.
- 라이다 스캔의 실제 앞뒤·좌우와 바퀴·차체에 가리는 구간.
- 위치를 실측한 물체까지의 거리 정확도.
- ROS 토픽·TF 연결, 이동 중 SLAM 품질과 Nav2 주행.

프로그램은 실행 중인 다른 앱을 종료하지 않습니다. `Ctrl-C` 또는 제한 시간 초과 시 이 도구가 만든 측정 프로세스만 종료를 요청합니다. 정상 종료에서는 IMU를 suspend로 전환하고 SDK의 라이다 stop/disconnect를 호출합니다. SDK 응답이나 프로세스 종료가 실제 라이다 회전 정지를 보장하는 것은 아니므로 직접 확인하세요. 배선 변경은 전원을 끈 상태에서 진행합니다.

## 개발 검증

다음 개발 명령은 `lekiwi-sensor-check` 폴더에서 실행합니다.

```bash
cd ./lekiwi-sensor-check
./setup.sh
/usr/bin/python3 -m unittest discover -s tests -v
bash -n setup.sh check_sensors.sh check_robot.sh
env -i PATH=/usr/bin:/bin ./check_sensors.sh --help
env -i PATH=/usr/bin:/bin ./check_robot.sh --help
```

테스트는 합성 측정값으로 PASS/FAIL 판정, 오래된 데이터, 정지 편차, 잘못된 포트, 시간 초과, 실패 기록 저장을 확인합니다. 일반 파일로 실제 `fuser`를 호출해 사용 중인 파일과 사용하지 않는 파일도 구분합니다. 빈 실행 파일 재빌드, 손상된 `.o` 제거, 빌드 실패 시 완료 표시 차단, 소스·기록 보존, 검사 중 빌드 차단도 확인합니다. 빌드 명령 검증은 임시 폴더와 장비에 접근하지 않는 ELF 시험 프로그램을 사용합니다. 원격 테스트는 소스 전송 범위, 설치 실패 시 측정 차단, 실패 결과 수집, 결과 소유권 반환, 명령 인수의 인용 처리, 연결 종료 시 자식 측정기 중단을 확인합니다. 실제 SSH·sudo·센서를 실행하지 않습니다. 실제 Pi의 SSH 인증·ARM 빌드·센서 판정·중단 처리는 현장 검증이 필요합니다.

코드 라이선스는 Apache-2.0이며 Bosch 원본은 별도 BSD-3-Clause입니다. 가져온 범위와 고정 버전은 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)를 참고하세요.
