#!/usr/bin/env bash
set -euo pipefail
CHECK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$CHECK_ROOT/check_sensors.py" "$@"
