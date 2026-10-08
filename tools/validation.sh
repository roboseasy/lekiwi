#!/usr/bin/env bash
# First-use commands. Each command runs only the requested validation stage.
set -eo pipefail

usage() {
  cat <<'HELP'
사용법: ./tools/validation.sh 명령 [값]
노트북: network, scan [유선장치], identify IP, prepare [--pc-only|--pi-only], folder,
        ssh [wifi], wifi-activate, wifi-check [IP], wifi-ping, ssh-key, ssh-reset,
        check, slam, map-check, teleop, save-map, safe-check, nav2 [지도YAML],
        nav2-view [지도YAML], nav-check
Pi: devices, wifi-show, wifi-apply, wifi-up, wifi-diagnose, wifi-fix-manager, wifi-edit, bringup [104|105]
공통: time
브링업은 토크를 끈 상태로 시작합니다. teleop·nav2는 토크를 켤 수 있습니다.
HELP
}
if [[ "${1:-}" == --help || "${1:-}" == -h || $# -eq 0 ]]; then usage; exit 0; fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
STATE="$ROOT/.colcon"
ACTION="$1"
shift
cd -- "$ROOT"

fail() { printf '%s\n' "$*" >&2; exit 1; }
case "$ACTION" in
  prepare|save-map) ;;  # The delegated command validates its own arguments.
  scan|identify|ssh|wifi-check|bringup|nav2|nav2-view)
    (( $# <= 1 )) || fail '값은 하나만 지정하세요. validation.sh --help를 확인하세요.' ;;
  *) (( $# == 0 )) || fail '이 명령은 추가 값을 받지 않습니다. validation.sh --help를 확인하세요.' ;;
esac
case "$ACTION" in
  devices|wifi-show|wifi-apply|wifi-up|wifi-diagnose|wifi-fix-manager|wifi-edit|bringup)
    [[ "$ROOT" == */lekiwi_ws/src/lekiwi ]] || fail 'Pi에 SSH로 접속한 터미널에서 실행하세요.' ;;
esac
target() { /usr/bin/python3 "$ROOT/tools/prepare_ros.py" --target; }
ipv4() {
  /usr/bin/python3 - "$1" <<'PY'
import ipaddress
import sys
try:
    print(ipaddress.IPv4Address(sys.argv[1]))
except ipaddress.AddressValueError:
    sys.exit('IPv4 주소를 확인하세요.')
PY
}
wifi_interface() {
  local interfaces=(/sys/class/net/*/wireless)
  [[ ${#interfaces[@]} -eq 1 && -d "${interfaces[0]}" ]] || fail '무선 장치를 하나로 식별하지 못했습니다.'
  local interface="${interfaces[0]%/wireless}"
  printf '%s\n' "${interface##*/}"
}
write_state() {
  local name="$1" value="$2" temporary
  mkdir -p -- "$STATE"
  temporary="$(mktemp "$STATE/.validation.XXXXXXXX")"
  chmod 600 "$temporary"
  printf '%s %s\n' "$(target)" "$value" > "$temporary"
  mv -- "$temporary" "$STATE/$name"
}
wifi_address() {
  local file="$1" saved_target saved_address
  [[ -r "$STATE/$file" ]] || fail '노트북 B에서 wifi-check를 먼저 실행하세요.'
  read -r saved_target saved_address < "$STATE/$file"
  [[ "$saved_target" == "$(target)" ]] || fail '점검 대상이 바뀌었습니다. wifi-check를 다시 실행하세요.'
  if [[ "$file" == wifi_verified.txt ]]; then
    [[ -r "$STATE/wifi_candidate.txt" && "$(cat "$STATE/wifi_candidate.txt")" == "$saved_target $saved_address" ]] ||
      fail '새 Wi-Fi 접속 확인이 완료되지 않았습니다. wifi-check를 다시 실행하세요.'
  fi
  ipv4 "$saved_address"
}
pc_environment() { source "$ROOT/tools/ros_env.sh" pc; }
pi_environment() { source "$ROOT/tools/ros_env.sh" pi; }
ensure_absent() {
  local nodes node
  nodes="$(timeout 10 ros2 node list)"
  for node in "$@"; do
    if printf '%s\n' "$nodes" | grep -Fxq -- "$node"; then
      fail "기존 $node 노드가 있습니다. 해당 터미널에서 종료한 뒤 진행하세요."
    fi
  done
}
observe() {
  local seconds="$1" expected="$2" log status=0
  shift 2
  log="$(mktemp /tmp/lekiwi-observe.XXXXXXXX)"
  timeout "$seconds" "$@" > "$log" 2>&1 || status=$?
  cat -- "$log"
  if [[ "$status" != 0 && "$status" != 124 ]] || ! grep -Eq -- "$expected" "$log"; then
    rm -- "$log"
    fail '필요한 데이터가 확인되지 않았습니다. 위 오류를 확인하고 다음 단계로 넘어가지 마세요.'
  fi
  rm -- "$log"
}
motor_ready() {
  observe 5 "^data: $1$" ros2 topic echo /motor_ready --once \
    --qos-reliability reliable --qos-durability transient_local
}
start_navigation() {
  local arm="$1" map_file saved_name
  shift
  pc_environment
  if [[ $# -gt 0 ]]; then
    map_file="$1"
  elif [[ -r "$STATE/last_map.txt" ]]; then
    IFS= read -r saved_name < "$STATE/last_map.txt"
    [[ "$saved_name" =~ ^map(_[0-9_]+)?\.yaml$ ]] || fail '저장 지도 기록이 올바르지 않습니다. 지도 YAML을 직접 지정하세요.'
    map_file="$LEKIWI_MAP_DIR/$saved_name"
  else
    map_file="$LEKIWI_MAP_DIR/map.yaml"
  fi
  map_file="$(realpath -e -- "$map_file")"
  /usr/bin/python3 - "$map_file" <<'PY'
from pathlib import Path
import sys
import yaml
try:
    path = Path(sys.argv[1])
    if not path.is_file() or not path.stat().st_size:
        raise ValueError('지도 YAML이 없거나 비어 있습니다.')
    data = yaml.safe_load(path.read_text())
    image = path.parent / data['image']
    if not image.is_file() or not image.stat().st_size:
        raise ValueError('지도 YAML의 이미지 파일이 없거나 비어 있습니다.')
except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
    sys.exit(str(error))
PY
  ensure_absent /cartographer_node /amcl /bt_navigator /lekiwi_nav2_motor_guard /lekiwi_guarded_base_teleop /rviz2
  printf '사용할 지도: %s\n' "$map_file"
  [[ "$arm" == false ]] || echo 'Nav2가 토크를 켤 수 있습니다. 이동 경로와 즉시 전원 차단 방법을 확인하세요.'
  exec ros2 launch lekiwi_navigation2 navigation.launch.py \
    map:="$map_file" use_sim_time:=false use_rviz:=true \
    auto_arm_motors:="$arm" motor_guard_duration:=3600
}

case "$ACTION" in
  network)
    nmcli -f DEVICE,TYPE,STATE,CONNECTION device
    ip -br -4 addr
    ;;
  scan)
    interface="${1:-}"
    if [[ -z "$interface" ]]; then
      mapfile -t interfaces < <(LC_ALL=C nmcli -t -f DEVICE,TYPE,STATE device | awk -F: '$2 == "ethernet" && $3 == "connected" {print $1}')
      [[ ${#interfaces[@]} -eq 1 ]] || fail '유선 장치를 하나로 식별하지 못했습니다. scan 뒤에 실제 유선 장치 이름을 지정하세요.'
      interface="${interfaces[0]}"
    fi
    cidr="$(ip -o -4 addr show dev "$interface" scope global | awk '{print $4}')"
    network="$(/usr/bin/python3 - "$cidr" <<'PY'
import ipaddress
import sys
try:
    network = ipaddress.IPv4Interface(sys.argv[1]).network
    if network.num_addresses > 256:
        sys.exit('유선 공유망이 /24보다 큽니다. Pi 전용 공유망 설정을 확인하세요.')
    print(network)
except ValueError:
    sys.exit('유선 장치의 IPv4 주소를 하나로 식별하지 못했습니다.')
PY
    )"
    if ! command -v nmap >/dev/null; then sudo apt install -y nmap; fi
    exec sudo nmap -sn -e "$interface" "$network"
    ;;
  identify)
    address="$(ipv4 "${1:?identify 뒤에 Pi IP를 지정하세요}")"
    user="$(target)"; user="${user%@*}"
    ping -c 3 -W 2 "$address"
    exec ssh "$user@$address" hostname
    ;;
  prepare) exec /usr/bin/python3 "$ROOT/tools/prepare_ros.py" "$@" ;;
  folder) printf 'cd && cd %q\n' "$(realpath --relative-to="$HOME" "$ROOT")" ;;
  ssh)
    wired="$(target)"
    if [[ "${1:-}" == wifi ]]; then
      address="$(wifi_address wifi_verified.txt)"
      wired="${wired%@*}@$address"
    elif [[ $# -gt 0 ]]; then fail '사용법: validation.sh ssh [wifi]'; fi
    exec ssh "$wired"
    ;;
  devices)
    workspace="$(cd -- "$ROOT/../.." && pwd -P)"
    ls -ld -- "$ROOT" "$workspace/install"
    id -nG
    ls -l /dev/i2c-1 /dev/serial/by-id/
    ;;
  wifi-show)
    interface="$(wifi_interface)"
    if systemctl is-active --quiet NetworkManager; then
      nmcli -f GENERAL.CONNECTION device show "$interface"
      LC_ALL=C nmcli -f IN-USE,SSID,MODE device wifi list ifname "$interface" --rescan no |
        awk 'NR == 1 || $1 == "*"'
    fi
    ip -br -4 addr show dev "$interface"
    ;;
  wifi-apply)
    sudo /usr/bin/python3 "$ROOT/tools/configure_wifi.py"
    sudo netplan generate
    exec sudo netplan try --timeout 180
    ;;
  wifi-up)
    if systemctl is-active --quiet NetworkManager; then
      sudo nmcli --wait 45 connection up id lekiwi-validation-wifi
    fi
    ip -br -4 addr show dev "$(wifi_interface)"
    ;;
  wifi-activate)
    exec ssh -t "$(target)" 'bash "$HOME/lekiwi_ws/src/lekiwi/tools/validation.sh" wifi-up'
    ;;
  wifi-check)
    address="${1:-}"
    if [[ -z "$address" ]]; then read -r -p 'Pi의 Wi-Fi IPv4 주소: ' address; fi
    address="$(ipv4 "$address")"
    nmcli -f DEVICE,TYPE,STATE,CONNECTION device
    route="$(ip route get "$address")"
    printf '%s\n' "$route"
    interface="$(printf '%s\n' "$route" | awk '{for (i=1;i<NF;i++) if ($i=="dev") {print $(i+1); exit}}')"
    [[ -n "$interface" && -d "/sys/class/net/$interface/wireless" ]] || fail 'Pi로 가는 경로가 노트북 Wi-Fi가 아닙니다. 랜선을 유지하고 경로를 확인하세요.'
    ping -c 3 -W 2 "$address"
    write_state wifi_candidate.txt "$address"
    wired="$(target)"
    ssh "${wired%@*}@$address" hostname
    write_state wifi_verified.txt "$address"
    echo '무선 경로·ping·SSH 확인 완료. 노트북 A에서 validation.sh ssh wifi로 접속하세요.'
    ;;
  wifi-ping) exec ping -c 3 -W 2 "$(wifi_address wifi_verified.txt)" ;;
  ssh-key)
    exec ssh "$(target)" 'ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub'
    ;;
  ssh-reset)
    address="$(wifi_address wifi_candidate.txt)"
    trusted="$(ssh "$(target)" 'ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub' | awk '/SHA256:/ {print $2}')"
    [[ "$trusted" =~ ^SHA256:[A-Za-z0-9+/]+$ ]] || fail '유선 접속에서 Pi 지문을 확인하지 못했습니다.'
    scanned="$(mktemp /tmp/lekiwi-host-key.XXXXXXXX)"
    trap 'rm -f -- "$scanned"' EXIT
    ssh-keyscan -T 5 -t ed25519 "$address" > "$scanned"
    fingerprint="$(ssh-keygen -lf "$scanned" | awk '/SHA256:/ {print $2}')"
    [[ "$trusted" == "$fingerprint" ]] || fail '유선 Pi와 Wi-Fi 장비의 지문이 다릅니다. 접속 기록을 변경하지 않고 중단합니다.'
    echo '유선 Pi와 Wi-Fi 장비의 ED25519 지문이 일치합니다.'
    (umask 077; mkdir -p "$HOME/.ssh"; touch "$HOME/.ssh/known_hosts")
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$address"
    cat -- "$scanned" >> "$HOME/.ssh/known_hosts"
    wired="$(target)"
    ssh "${wired%@*}@$address" hostname
    echo '접속 기록을 갱신했습니다. wifi-check를 다시 실행하세요.'
    ;;
  wifi-diagnose)
    ip -br link
    rfkill list wifi
    if command -v nmcli >/dev/null; then nmcli -f DEVICE,TYPE,STATE,CONNECTION device; fi
    networkctl list --no-pager
    sudo journalctl -b --no-pager -u NetworkManager -u 'netplan-wpa-*' -u systemd-networkd -n 60
    ;;
  wifi-fix-manager)
    interface="$(wifi_interface)"
    systemctl is-active --quiet NetworkManager
    sudo test -f /etc/netplan/90-lekiwi-wifi.yaml
    sudo cp -a -- /etc/netplan/90-lekiwi-wifi.yaml "/etc/netplan/90-lekiwi-wifi.yaml.bak.$(date +%Y%m%d-%H%M%S-%N)"
    sudo netplan set --origin-hint 90-lekiwi-wifi \
      "network.wifis.$interface={renderer: NetworkManager, networkmanager: {name: lekiwi-validation-wifi, passthrough: {connection.autoconnect-priority: \"999\"}}}"
    sudo netplan generate
    echo 'Wi-Fi 관리자 보완 완료. wifi-apply부터 이어가세요.'
    ;;
  wifi-edit)
    sudo cp -a -- /etc/netplan/90-lekiwi-wifi.yaml "/etc/netplan/90-lekiwi-wifi.yaml.bak.$(date +%Y%m%d-%H%M%S-%N)"
    exec sudo nano /etc/netplan/90-lekiwi-wifi.yaml
    ;;
  time)
    date -u '+%Y-%m-%d %H:%M:%S UTC'
    synchronized="$(timedatectl show -p NTPSynchronized)"
    printf '%s\n' "$synchronized"
    [[ "$synchronized" == NTPSynchronized=yes ]] || fail '시각 동기화가 완료되지 않았습니다.'
    ;;
  bringup)
    address="${1:-104}"
    [[ "$address" == 104 || "$address" == 105 ]] || fail 'IMU 주소는 104(0x68) 또는 105(0x69)입니다.'
    pi_environment
    workspace="$(cd -- "$ROOT/../.." && pwd -P)"
    [[ -s "$workspace/base_profile.yaml" ]] || fail '제품 설정 파일 base_profile.yaml을 먼저 준비하세요.'
    serial_root=/dev/serial/by-id
    shopt -s nullglob
    motor_ports=("$serial_root"/usb-1a86_USB_Single_Serial_*-if00)
    lidar_ports=("$serial_root"/*CP2102*)
    if [[ -z "${MOTOR_PORT:-}" ]]; then
      [[ ${#motor_ports[@]} -eq 1 ]] || fail '모터 포트를 하나로 식별하지 못했습니다. MOTOR_PORT를 실제 포트로 지정하세요.'
      MOTOR_PORT="${motor_ports[0]}"
    fi
    if [[ -z "${LIDAR_PORT:-}" ]]; then
      [[ ${#lidar_ports[@]} -eq 1 ]] || fail '라이다 포트를 하나로 식별하지 못했습니다. LIDAR_PORT를 실제 포트로 지정하세요.'
      LIDAR_PORT="${lidar_ports[0]}"
    fi
    [[ -e "$MOTOR_PORT" && -e "$LIDAR_PORT" ]] || fail '모터·라이다 장치가 연결되지 않았습니다.'
    [[ "$(readlink -f -- "$MOTOR_PORT")" != "$(readlink -f -- "$LIDAR_PORT")" ]] || fail '모터와 라이다가 같은 포트입니다.'
    [[ "$(readlink -f -- "$LIDAR_PORT")" != */ttyACM* ]] || fail 'ttyACM 모터 포트를 라이다에 사용할 수 없습니다.'
    command -v fuser >/dev/null || fail 'fuser가 없습니다. 먼저 psmisc를 설치하세요.'
    for device in "$MOTOR_PORT" "$LIDAR_PORT" /dev/i2c-1; do
      [[ -r "$device" && -w "$device" ]] || fail "장치 연결·권한을 확인하세요: $device"
      if fuser -- "$device" >/dev/null 2>&1; then fail "다른 프로그램이 장치를 사용 중입니다: $device"; fi
    done
    printf '모터: %s\n라이다: %s\n' "$MOTOR_PORT" "$LIDAR_PORT"
    echo '바퀴 토크는 꺼진 상태로 시작합니다. 초기 IMU 측정 동안 로봇을 정지시키세요.'
    exec ros2 launch lekiwi_bringup robot.launch.py \
      base_params_file:="$workspace/base_profile.yaml" serial_port:="$MOTOR_PORT" lidar_port:="$LIDAR_PORT" \
      use_imu:=true imu_i2c_device:=/dev/i2c-1 imu_i2c_address:="$address" torque_enable:=false
    ;;
  check)
    pc_environment
    motor_ready false
    for topic in /scan /imu/data_raw /odom; do observe 10 'average rate:' ros2 topic hz "$topic"; done
    observe 5 'Translation:' ros2 run tf2_ros tf2_echo odom base_footprint
    observe 5 'Translation:' ros2 run tf2_ros tf2_echo base_footprint lidar_link
    observe 5 'Translation:' ros2 run tf2_ros tf2_echo base_footprint imu_link
    echo '브링업 입력 확인 완료. 측정 주기와 장착 방향은 실제 값으로 확인하세요.'
    ;;
  slam)
    pc_environment
    ensure_absent /cartographer_node /amcl /bt_navigator /rviz2
    exec ros2 launch lekiwi_cartographer cartographer.launch.py \
      configuration_basename:=lekiwi_2d_imu.lua use_sim_time:=false use_rviz:=true
    ;;
  map-check)
    pc_environment
    observe 10 'average rate:' ros2 topic hz /scan_navigation
    observe 10 'width: [1-9][0-9]*' ros2 topic echo /map --once --field info
    ;;
  teleop)
    pc_environment
    ensure_absent /amcl /bt_navigator /lekiwi_nav2_motor_guard /lekiwi_guarded_base_teleop
    echo '토크가 켜질 수 있습니다. 주변과 전원 차단 방법을 확인하고 가장 낮은 속도로 시작하세요.'
    exec ros2 run lekiwi_teleop lekiwi_guarded_base_teleop --yes-i-confirm-motion --max-session-duration 3600
    ;;
  save-map) exec bash "$ROOT/tools/save_map.sh" "$@" ;;
  safe-check)
    pc_environment
    motor_ready false
    ensure_absent /cartographer_node /amcl /bt_navigator /lekiwi_nav2_motor_guard /lekiwi_guarded_base_teleop
    ros2 node list
    ;;
  nav2) start_navigation true "$@" ;;
  nav2-view) start_navigation false "$@" ;;
  nav-check)
    pc_environment
    motor_ready true
    for node in /amcl /bt_navigator /controller_server; do
      observe 5 '^active \[' ros2 lifecycle get "$node"
    done
    info="$(ros2 topic info /cmd_vel --verbose)"
    printf '%s\n' "$info"
    printf '%s\n' "$info" | grep -Eq '^Publisher count: 1$' || fail '/cmd_vel 발행자가 하나가 아닙니다.'
    printf '%s\n' "$info" | grep -Eq '^Node name: collision_monitor$' || fail '/cmd_vel 발행자를 확인하세요.'
    echo 'Nav2 준비 상태 확인 완료. RViz 벽 정합과 실제 이동 안전을 확인한 뒤 목표 한 번을 지정하세요.'
    ;;
  *) usage >&2; exit 2 ;;
esac
