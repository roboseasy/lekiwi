#!/usr/bin/env bash
set -euo pipefail

# 점검할 르키위의 IP만 바꾸고 노트북에서 이 파일을 실행하세요.
ROBOT_IP="10.42.0.67"
ROBOT_USER="roboseasy"

CHECK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$CHECK_ROOT/check_remote.py" --ip "$ROBOT_IP" --user "$ROBOT_USER" "$@"
