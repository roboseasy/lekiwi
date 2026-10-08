#!/usr/bin/env bash
set -eo pipefail

ROLE="${1:?pc 또는 pi를 지정하세요}"
SOURCE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
case "$ROLE" in
  pc) WORKSPACE="$SOURCE_ROOT"; DEPS_WORKSPACE="$(realpath -m "$SOURCE_ROOT/../lekiwi_deps")" ;;
  pi) WORKSPACE="$(cd -- "$SOURCE_ROOT/../.." && pwd -P)"; DEPS_WORKSPACE="$WORKSPACE" ;;
  *) echo '사용법: install_ros.sh pc|pi' >&2; exit 2 ;;
esac
if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy가 없습니다. first_robot_validation.md의 설치 전제를 확인하세요.' >&2
  exit 1
fi
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
unset PYTHONHOME PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
hash -r
source /opt/ros/jazzy/setup.bash
mkdir -p "$WORKSPACE/.colcon"
exec 9>"$WORKSPACE/.colcon/lekiwi-setup.lock"
flock -n 9 || { echo '다른 ROS 준비가 실행 중입니다.' >&2; exit 1; }
SOURCE_HASH="$(/usr/bin/python3 "$SOURCE_ROOT/tools/prepare_ros.py" --source-hash)"
READY_FILE="$WORKSPACE/.colcon/lekiwi-setup.ready"
PACKAGES=(lekiwi lekiwi_description lekiwi_bringup lekiwi_node lekiwi_sensors
          lekiwi_teleop lekiwi_cartographer lekiwi_navigation2)
PACKAGE_PATHS=()
for package in "${PACKAGES[@]}"; do PACKAGE_PATHS+=("$SOURCE_ROOT/$package"); done
DEPENDENCY_PATHS=("$DEPS_WORKSPACE/src/YDLidar-SDK" "$DEPS_WORKSPACE/src/ydlidar_ros2_driver")

installed_packages_ready() {
  [[ -r "$DEPS_WORKSPACE/install/local_setup.bash" && -r "$WORKSPACE/install/local_setup.bash" ]] || return 1
  (
    source "$DEPS_WORKSPACE/install/local_setup.bash"
    source "$WORKSPACE/install/local_setup.bash"
    for package in "${PACKAGES[@]}"; do
      [[ "$(ros2 pkg prefix "$package" 2>/dev/null)" == "$WORKSPACE/install/$package" ]] || exit 1
    done
    [[ -s "$DEPS_WORKSPACE/install/ydlidar_sdk/lib/libydlidar_sdk.a" ||
       -s "$DEPS_WORKSPACE/install/ydlidar_sdk/lib/libydlidar_sdk.so" ]] || exit 1
    [[ "$(ros2 pkg prefix ydlidar_ros2_driver 2>/dev/null)" == "$DEPS_WORKSPACE/install/ydlidar_ros2_driver" ]] || exit 1
    for package in cartographer_ros nav2_map_server rviz2 rmw_fastrtps_cpp; do
      ros2 pkg prefix "$package" >/dev/null 2>&1 || exit 1
    done
    if [[ "$ROLE" == pc ]]; then /usr/bin/python3 -c 'import tkinter' || exit 1; fi
  )
}

if [[ -r "$READY_FILE" && "$(cat "$READY_FILE")" == "$SOURCE_HASH" ]] && installed_packages_ready; then
  echo "[$ROLE] 같은 소스의 ROS 패키지가 준비돼 있습니다. 설치·빌드를 생략합니다."
else
  echo "[$ROLE] 의존성 설치와 SDK → 드라이버 → LeKiwi 빌드"
  sudo apt-get update
  sudo apt-get install -y git build-essential cmake python3-colcon-common-extensions \
    python3-vcstool python3-rosdep python3-yaml python3-opencv python3-numpy \
    ros-jazzy-usb-cam ros-jazzy-rmw-fastrtps-cpp
  if [[ "$ROLE" == pc ]]; then
    sudo apt-get install -y python3-tk
  else
    sudo apt-get install -y wpasupplicant rfkill psmisc
  fi
  mkdir -p "$DEPS_WORKSPACE/src"
  # 기존 외부 소스는 교체하거나 checkout하지 않는다.
  /usr/bin/python3 - "$SOURCE_ROOT/lekiwi.repos" "$DEPS_WORKSPACE/src" <<'PY'
from pathlib import Path
import subprocess
import sys
import yaml

repositories = yaml.safe_load(Path(sys.argv[1]).read_text())['repositories']
for name, spec in repositories.items():
    destination = Path(sys.argv[2]) / name
    if destination.exists():
        commit = subprocess.check_output(['git', '-C', str(destination), 'rev-parse', 'HEAD'], text=True).strip()
        modified = subprocess.check_output(['git', '-C', str(destination), 'status', '--porcelain', '--untracked-files=no'], text=True)
        if commit != spec['version'] or modified:
            sys.exit(f'기존 외부 소스를 보존하고 중단합니다. 버전·수정 상태를 확인하세요: {destination}')
    else:
        subprocess.run(['git', 'clone', '--no-checkout', spec['url'], str(destination)], check=True)
        subprocess.run(['git', '-C', str(destination), 'checkout', '--detach', spec['version']], check=True)
PY
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
  rosdep update
  rosdep install --from-paths "${PACKAGE_PATHS[@]}" "${DEPENDENCY_PATHS[@]}" \
    --ignore-src --rosdistro jazzy -y
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  export MAKEFLAGS=-j2
  cd "$DEPS_WORKSPACE"
  colcon build --base-paths "${DEPENDENCY_PATHS[@]}" --symlink-install --executor sequential --packages-select ydlidar_sdk \
    --cmake-clean-cache --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_EXAMPLES=OFF -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE -DSWIG_FOUND=FALSE
  source install/local_setup.bash
  colcon build --base-paths "${DEPENDENCY_PATHS[@]}" --symlink-install --executor sequential --packages-select ydlidar_ros2_driver \
    --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
    "-DCMAKE_EXE_LINKER_FLAGS=-L\"$DEPS_WORKSPACE/install/ydlidar_sdk/lib\""
  source install/local_setup.bash
  cd "$WORKSPACE"
  colcon build --base-paths "${PACKAGE_PATHS[@]}" --symlink-install --executor sequential \
    --packages-select "${PACKAGES[@]}" --cmake-clean-cache \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
  installed_packages_ready || { echo '빌드 후 패키지 확인에 실패했습니다.' >&2; exit 1; }
  printf '%s\n' "$SOURCE_HASH" > "$READY_FILE"
fi

if [[ "$ROLE" == pi ]]; then
  MISSING_GROUPS=()
  for group in i2c dialout; do
    getent group "$group" >/dev/null || { echo "장치 그룹이 없습니다: $group" >&2; exit 1; }
    if [[ " $(id -nG) " != *" $group "* ]]; then MISSING_GROUPS+=("$group"); fi
  done
  if (( ${#MISSING_GROUPS[@]} )); then
    GROUP_LIST="$(IFS=,; echo "${MISSING_GROUPS[*]}")"
    sudo usermod -aG "$GROUP_LIST" "$(id -un)"
    echo '장치 그룹을 추가했습니다. 준비 완료 후 새 SSH 접속부터 적용됩니다.'
  fi
  if [[ ! -e "$WORKSPACE/base_profile.yaml" ]]; then
    cp "$SOURCE_ROOT/lekiwi_bringup/param/base_profile.example.yaml" "$WORKSPACE/base_profile.yaml"
  fi
  test -s "$WORKSPACE/base_profile.yaml"
  echo '제품 설정 준비: ~/lekiwi_ws/base_profile.yaml (기존 파일 보존)'
fi
echo "[$ROLE] ROS 준비 완료"
