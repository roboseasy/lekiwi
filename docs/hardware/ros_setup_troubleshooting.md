# ROS 준비·Wi-Fi 장애 안내

정상 진행은 [첫 로봇 실증 가이드](first_robot_validation.md)를 따른다.
아래는 자동 준비나 무선 접속 확인이 실패했을 때만 사용한다.
기존 소스·설정·빌드·검사 결과를 임의로 삭제하지 않는다.

## ROS 준비가 중단됐을 때

| 메시지·상태 | 확인할 내용 |
| --- | --- |
| ROS 2 Jazzy가 없음 | 노트북·Pi 모두 `/opt/ros/jazzy/setup.bash`가 필요하다. [공식 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 완료한다. |
| SSH 접속 실패 | 센서 스크립트의 IP와 계정, 랜선, ping·hostname을 확인한다. |
| 기존 Pi ROS 소스가 다름 | 노트북과 Pi의 소스가 같은 버전인지 먼저 비교한다. 자동 준비는 기존 소스를 덮어쓰지 않는다. |
| 기존 외부 소스 버전·수정 상태가 다름 | `lekiwi.repos`의 고정 커밋과 기존 `YDLidar-SDK`, `ydlidar_ros2_driver`의 HEAD·수정 내역을 비교한다. 자동 checkout하지 않는다. |
| apt·rosdep·git 오류 | 두 장비의 인터넷 연결과 마지막 오류를 확인한다. 해결 후 같은 명령으로 재개한다. |
| CMake 호환성 오류 | 자동 준비는 시스템 CMake와 `CMAKE_POLICY_VERSION_MINIMUM=3.5`를 사용한다. 전체 오류와 `/usr/bin/cmake --version`을 확인한다. |
| 노트북 단계만 실패 | 원인 해결 후 `./tools/validation.sh prepare --pc-only`로 재개한다. Pi만 준비할 때는 `--pi-only`를 사용한다. |

Pi의 기존 소스 버전·설치와 설정을 확인하려면 노트북에서 접속한 뒤 다음을 실행한다.
Git으로 복제한 소스이면 `git -C ./src/lekiwi status --short`와 `rev-parse HEAD`도 확인한다.
노트북에서 전송한 소스는 Git 저장소가 아닐 수 있다.

```bash
cd lekiwi_ws
./src/lekiwi/tools/validation.sh devices
```

새 그룹은 **새 SSH 접속부터** 적용된다. 그룹 추가 후 열려 있던 SSH 터미널을 계속 사용하지 않는다.
`base_profile.yaml`의 제품별 값이 다르면 검수값을 확인한 뒤 그 파일만 편집한다.
자동 준비는 기존 `base_profile.yaml`을 보존한다.

## SSH 키 변경 경고가 나올 때

`REMOTE HOST IDENTIFICATION HAS CHANGED!`는 노트북에 저장된 IP의 SSH 키와
현재 응답한 장비의 키가 다르다는 뜻이다. 다른 르키위가 같은 IP를 받거나 Pi를 재설치하면
발생할 수 있다. **랜선을 유지하고 실제 Pi의 키를 먼저 확인한다.**

**유선 접속 복구 — 노트북 A:** 이미 유선 SSH를 종료했다면 저장소 루트에서 다시 접속한다.
센서 스크립트의 IP는 1절에서 확인한 현재 Pi의 유선 IP여야 한다.
아직 A가 유선으로 Pi에 접속돼 있으면 이 블록은 생략한다.

```bash
./tools/validation.sh ssh
```

**지문 확인 — Pi A:** 경고에 `ED25519 key`가 표시됐다면 다음을 실행한다.
유선 접속에서도 키 변경 경고가 나오면 Pi에 직접 연결한 화면·키보드에서 확인한다.

```bash
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

출력의 `SHA256:...` 전체가 Wi-Fi SSH 경고에 나온 **현재 장비의 ED25519 지문**과 같아야 한다.
`Offending ECDSA key` 등은 노트북의 예전 기록을 가리킨다.
지문이 다르면 접속 기록을 변경하지 말고 Pi의 무선 IP를 다시 확인한다.

**노트북 B:** 먼저 5절의 `wifi-check`로 현재 Wi-Fi IP를 입력한다. 키 경고가 나면 다음을 실행한다.
스크립트는 유선 Pi의 지문과 Wi-Fi에서 받은 키의 지문을 비교해 **일치한 경우에만**
해당 IP의 예전 기록을 제거하고 확인한 키를 등록한다. 지문이 다르면 기록을 바꾸지 않고 중단한다.

```bash
./tools/validation.sh ssh-reset
./tools/validation.sh wifi-check
```

새 키와 해당 Pi의 hostname까지 확인한 뒤 5절부터 이어간다.
A도 노트북으로 돌아온 뒤 Wi-Fi IP로 접속해야 한다.
노트북의 무선 경로·ping·hostname·A의 무선 SSH 로그인을 모두 확인한 뒤 랜선을 분리한다.

## Wi-Fi 주소가 없을 때

**랜선을 유지하고 브링업을 시작하지 않는다.** Pi 터미널의 `lekiwi_ws`에서 확인한다.

```bash
./src/lekiwi/tools/validation.sh wifi-diagnose
```

| 결과 | 다음 행동 |
| --- | --- |
| 무선 장치 이름이 `wlan0`과 다름 | 스크립트는 실제 무선 장치를 찾는다. 여러 장치가 있으면 연결할 장치부터 확인한다. |
| `Soft blocked: yes` | `sudo rfkill unblock wifi` 후 다시 확인한다. `Hard blocked`이면 장비 설정부터 확인한다. |
| NetworkManager가 없거나 inactive | `nmcli` 활성화 단계는 생략한다. `networkctl`과 `netplan-wpa-*` 로그를 확인한다. |
| `wlan0 connected lekiwi-ap`, 다른 설정이 이미 활성화됨 | NetworkManager가 관리하는 장치인지 확인하고 아래 장치별 관리자 보완을 적용한다. |
| `ctrl_iface exists ... in use` | 같은 무선 장치를 두 관리자가 동시에 사용 중인지 확인한다. 사용 중인 소켓을 삭제하거나 전체 네트워크 서비스를 종료하지 않는다. |
| 인증 실패·SSID 검색 실패 | 노트북과 같은 Wi-Fi 이름·암호·수신 상태를 확인한다. |

### NetworkManager가 wlan0를 관리하는 기존 Pi

기존 `/etc/netplan/90-lekiwi-wifi.yaml`이 있는데 `wlan0`는 NetworkManager가 관리하고,
기존 파일이 networkd를 사용한다면 아래로 **해당 장치만** 보완한다.
SSID·암호는 유지하고 파일을 백업한다. Ethernet 설정은 편집하지 않는다.

```bash
./src/lekiwi/tools/validation.sh wifi-fix-manager
```

이후 첫 실증 가이드 5절의 **시험 적용 → 연결 활성화 → 무선 주소·경로·SSH 확인**부터 진행한다.
`netplan try`의 시간이 끝났다면 실제 연결과 설정이 되돌아갔는지 확인한 뒤 다시 진행한다.
[`netplan try`](https://netplan.readthedocs.io/en/stable/netplan-try/)와
[장치별 renderer](https://netplan.readthedocs.io/en/stable/netplan-yaml/#properties-for-all-device-types)를 참고한다.

### Wi-Fi 이름·암호를 잘못 입력한 경우

Pi에서 기존 파일을 백업한 뒤 편집한다. 암호가 들어 있는 파일 내용을 채팅·로그·Git에 붙이지 않는다.

```bash
./src/lekiwi/tools/validation.sh wifi-edit
```

수정 후 5절의 시험 적용부터 진행한다. Wi-Fi 주소가 보여도
노트북에서 **무선 경로·ping·해당 Pi hostname·무선 SSH**가 모두 확인되기 전에는 랜선을 빼지 않는다.
