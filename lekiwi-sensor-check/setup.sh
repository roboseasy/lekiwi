#!/usr/bin/env bash
set -euo pipefail
CHECK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_COMMIT=42a82ed10d2304094c111fc63dee8e4a229b79b7
for tool in git cmake gcc g++ /usr/bin/python3 fuser flock; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo '빌드 도구가 없습니다. 다음 명령으로 한 번 설치하세요:' >&2
    echo 'sudo apt install -y git build-essential cmake python3 psmisc util-linux' >&2
    exit 2
  fi
done
# build 폴더 밖의 잠금으로 빌드 정리와 측정을 함께 보호한다.
exec 9>"$CHECK_ROOT/.sensor-check.lock"
flock -n 9 || { echo '이 도구의 다른 준비·검사가 실행 중입니다.' >&2; exit 2; }
# 이전 버전의 build/run.lock을 사용 중인 검사도 정리 전에 확인한다.
if [[ -d "$CHECK_ROOT/build" && ! -L "$CHECK_ROOT/build" ]]; then
  exec 8>"$CHECK_ROOT/build/run.lock"
  flock -n 8 || { echo '이전 버전의 검사가 실행 중입니다.' >&2; exit 2; }
fi
mkdir -p "$CHECK_ROOT/.deps"
SDK_DIR="$CHECK_ROOT/.deps/YDLidar-SDK"
if [[ ! -d "$SDK_DIR" ]]; then
  git clone --no-checkout --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git "$SDK_DIR"
  git -C "$SDK_DIR" fetch --depth 1 origin "$SDK_COMMIT"
  git -C "$SDK_DIR" checkout --detach "$SDK_COMMIT"
fi
if [[ "$(git -C "$SDK_DIR" rev-parse HEAD)" != "$SDK_COMMIT" ]] ||
   [[ -n "$(git -C "$SDK_DIR" status --porcelain)" ]]; then
  echo 'SDK 버전 또는 소스 상태가 다릅니다. 기존 파일을 보존하고 설치를 중단합니다.' >&2
  exit 2
fi
/usr/bin/python3 - "$CHECK_ROOT" <<'PY'
import hashlib
from pathlib import Path
import sys
root = Path(sys.argv[1]) / 'vendor/bosch_bmi160'
expected = {
    'bmi160.c': 'df3ff5e019db827287be52619d30ee647bcabee65f9a0bd3ddacef2ea908b591',
    'bmi160.h': 'c28edf703c23f4c25ee4a8a08ff06e4b705c106638ee62e2e9116bf01639628c',
    'bmi160_defs.h': 'abcb11556b25f374f4f8da05ee5f7f80d758df93abc32129ecb63131dcf78960',
}
for name, digest in expected.items():
    if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
        sys.exit(f'Bosch 원본 해시 불일치: {name}')
PY
/usr/bin/python3 "$CHECK_ROOT/probe_build.py" clean
cmake -S "$CHECK_ROOT" -B "$CHECK_ROOT/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$CHECK_ROOT/build" --target imu_probe lidar_probe --parallel "${CHECK_BUILD_JOBS:-2}"
/usr/bin/python3 "$CHECK_ROOT/probe_build.py" verify
echo '설치 완료. ./check_sensors.sh --unit 제품이름 으로 검사하세요.'
