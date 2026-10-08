#!/usr/bin/env bash
# source ./tools/ros_env.sh pc  /  source ./src/lekiwi/tools/ros_env.sh pi

lekiwi_ros_environment() {
  local role="${1:-pc}" root workspace dependencies
  root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
  case "$role" in
    pc) workspace="$root"; dependencies="$root/../lekiwi_deps" ;;
    pi) workspace="$(cd -- "$root/../.." && pwd -P)"; dependencies="$workspace" ;;
    *) echo '사용법: source ros_env.sh pc|pi' >&2; return 2 ;;
  esac
  if [[ ! -r /opt/ros/jazzy/setup.bash || ! -r "$workspace/install/local_setup.bash" ||
        ! -r "$dependencies/install/local_setup.bash" ]]; then
    echo 'ROS 설치가 준비되지 않았습니다. 노트북에서 ./tools/prepare_ros.py를 실행하세요.' >&2
    return 1
  fi
  # 시스템 Python/CMake를 사용하고 이전 overlay·가상환경 경로를 제거한다.
  export PATH=/usr/bin:/bin:/usr/sbin:/sbin
  unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
  unset CMAKE_PREFIX_PATH VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
  hash -r
  source /opt/ros/jazzy/setup.bash || return
  source "$dependencies/install/local_setup.bash" || return
  if [[ "$workspace" != "$dependencies" ]]; then
    source "$workspace/install/local_setup.bash" || return
  fi
  unset ROS_LOCALHOST_ONLY
  export ROS_DOMAIN_ID="${LEKIWI_ROS_DOMAIN_ID:-42}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export LEKIWI_MAP_DIR="$(realpath -m "$root/../maps/lekiwi")"
  printf 'ROS 준비: %s / domain %s\n' "$role" "$ROS_DOMAIN_ID"
}

lekiwi_ros_environment "$@"
