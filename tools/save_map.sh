#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo '사용법: ./tools/save_map.sh'
  echo '노트북에서 실행하세요. SLAM을 유지한 상태에서 YAML·PGM·pbstream을 저장합니다.'
  exit 0
fi
if (( $# )); then
  echo '사용법: ./tools/save_map.sh' >&2
  exit 2
fi

LEKIWI_SAVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
LEKIWI_SAVE_STEP='ROS 환경 적용'
trap 'printf "지도 저장 중단(%s). SLAM을 유지하고 위 오류를 확인하세요.\n" "$LEKIWI_SAVE_STEP" >&2' ERR
cd -- "$LEKIWI_SAVE_ROOT"
source ./tools/ros_env.sh pc

LEKIWI_SAVE_STEP='지도 폴더 준비'
mkdir -p -- "${LEKIWI_MAP_DIR:?ROS 환경의 지도 경로가 없습니다. 오류를 확인하세요}"
exec 9>"$LEKIWI_MAP_DIR/.save-map.lock"
if ! flock -n 9; then
  echo '다른 지도 저장이 실행 중입니다. 완료된 뒤 진행하세요.' >&2
  exit 1
fi

LEKIWI_MAP_BASE="$LEKIWI_MAP_DIR/map"
for extension in yaml pgm pbstream; do
  if [[ -e "$LEKIWI_MAP_BASE.$extension" || -L "$LEKIWI_MAP_BASE.$extension" ]]; then
    LEKIWI_MAP_BASE="$LEKIWI_MAP_DIR/map_$(date +%Y%m%d_%H%M%S_%N)"
    break
  fi
done
for extension in yaml pgm pbstream; do
  if [[ -e "$LEKIWI_MAP_BASE.$extension" || -L "$LEKIWI_MAP_BASE.$extension" ]]; then
    echo '선택한 지도 이름이 이미 사용 중입니다. 기존 파일을 보존하고 중단합니다.' >&2
    exit 1
  fi
done

LEKIWI_SAVE_STEP='YAML·PGM 저장'
ros2 run nav2_map_server map_saver_cli -f "$LEKIWI_MAP_BASE" \
  --ros-args -p save_map_timeout:=10.0

LEKIWI_SAVE_STEP='Cartographer 상태 저장'
request="$(/usr/bin/python3 - "$LEKIWI_MAP_BASE.pbstream" <<'PY'
import json
import sys
print(json.dumps({'filename': sys.argv[1], 'include_unfinished_submaps': True}))
PY
)"
echo 'Cartographer 상태 저장 응답을 기다립니다(최대 60초).'
status=0
response="$(timeout 60 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "$request" 2>&1)" || status=$?
printf '%s\n' "$response"
if (( status != 0 )); then
  echo 'Cartographer 상태 저장에 실패했습니다. SLAM과 /write_state 서비스를 확인하세요.' >&2
  exit "$status"
fi
# ros2 service call exits successfully even for an application-level error.
# Jazzy prints the generated message repr; require StatusCode.OK explicitly.
if ! grep -Eq 'StatusResponse\(code=0,' <<< "$response"; then
  echo 'Cartographer 상태 저장 성공 응답을 확인하지 못했습니다.' >&2
  exit 1
fi

for extension in yaml pgm pbstream; do
  if [[ ! -s "$LEKIWI_MAP_BASE.$extension" ]]; then
    printf '지도 저장 중단: 파일이 없거나 비어 있습니다: %s\n' "$LEKIWI_MAP_BASE.$extension" >&2
    exit 1
  fi
done
LEKIWI_SAVE_STEP='저장 파일 확인'
ls -lh -- "$LEKIWI_MAP_BASE.yaml" "$LEKIWI_MAP_BASE.pgm" "$LEKIWI_MAP_BASE.pbstream"
mkdir -p -- "$LEKIWI_SAVE_ROOT/.colcon"
record="$(mktemp "$LEKIWI_SAVE_ROOT/.colcon/.last-map.XXXXXXXX")"
printf '%s.yaml\n' "${LEKIWI_MAP_BASE##*/}" > "$record"
mv -- "$record" "$LEKIWI_SAVE_ROOT/.colcon/last_map.txt"
printf 'Nav2에서 사용할 지도: %s.yaml\n' "$LEKIWI_MAP_BASE"
