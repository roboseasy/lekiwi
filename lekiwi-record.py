#!/usr/bin/env python
r"""LeKiwi 데이터셋 녹화 (CLI 인자로 전부 설정, 코드 수정 불필요).

`lerobot-record` 는 teleop 을 하나만 받기 때문에 LeKiwi 에 쓸 수 없다.
LeKiwi 는 팔(리더암) + 베이스(키보드) 두 개의 teleop 이 동시에 필요하다.
이 스크립트는 lerobot 과 동일한 draccus CLI 파싱(`--dataset.*` 등)을 쓰면서
그 조합을 대신 만들어 준다. 녹화 루프 자체는 lerobot 의 `record_loop` 를 그대로 쓴다.

먼저 라즈베리파이(LeKiwi) 쪽에서 호스트를 띄워 둘 것:

    python -m lerobot.robots.lekiwi.lekiwi_host \
        --robot.id=lekiwi01 \
        --robot.cameras='{
          front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
          wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
        }' \
        --host.connection_time_s=14400

호스트 카메라 설정 규칙: `width`/`height` 는 **회전을 적용한 뒤의 출력 크기**이고, 캡처
해상도는 rotation 이 90/270 일 때만 뒤집혀 설정된다 (`cameras/opencv/camera_opencv.py:124`).
센서가 640x480 인 웹캠이면 → rotation 0/180 은 `width 640, height 480`,
rotation 90/-90 은 `width 480, height 640`. 다른 조합은 캡처 설정에 실패한다.
rotation 값은 카메라가 물리적으로 어떻게 달렸는지의 문제일 뿐, 두 카메라가 같을 필요는 없다.

그 다음 PC 에서 (so101 로 쓰던 `lerobot-record` 와 같은 스타일):

    HF_USER=your_hf_id
    TASK_NAME=lekiwi_pick_cube

    python lekiwi-record.py \
        --robot.remote_ip=192.168.0.201 \
        --robot.id=lekiwi01 \
        --robot.cameras='{
          front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30},
          wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30},
      }' \
        --teleop.port=/dev/so101-leader \
        --teleop.id=leader \
        --dataset.single_task=${TASK_NAME} \
        --dataset.repo_id=${HF_USER}/${TASK_NAME} \
        --dataset.num_episodes=10 \
        --dataset.episode_time_s=15 \
        --dataset.reset_time_s=3 \
        --dataset.streaming_encoding=true \
        --display_data=true \
        --dataset.push_to_hub=false

--robot.* / --teleop.* 기본값이 이미 위 장비에 맞춰져 있으므로, 실제로는 이 정도만 쓰면 된다:

    python lekiwi-record.py \
        --dataset.repo_id=${HF_USER}/${TASK_NAME} \
        --dataset.single_task="Pick up the cube and place it in the box" \
        --dataset.num_episodes=10 \
        --dataset.episode_time_s=15 \
        --dataset.reset_time_s=3 \
        --display_data=true \
        --dataset.push_to_hub=false

전체 옵션은 `python lekiwi-record.py --help`.

카메라 주의: 클라이언트는 카메라를 열지 않는다. 여기 적은 width/height 는 "호스트가 이런
모양으로 보내온다"는 선언이고, 그 값이 곧 데이터셋 이미지 shape 이 된다. 호스트의 rotation
설정에 따라 가로/세로가 뒤집히므로(lekiwi 기본값은 wrist 를 90도 회전 → 480x640) 값이 다르면
연결 직후 실제 프레임에 맞춰 자동 보정하고 경고를 남긴다. 엄격하게 검사하려면
`--auto_camera_shape=false`.

조작키
    리더암      : 그대로 따라 움직인다
    베이스 주행 : W/S 전후, A/D 좌우, Q/E 제자리 좌/우 회전, R/F 속도 단계, SPACE 정지
    녹화 제어   : →(또는 N) 현재 에피소드 종료, ←  재녹화, ESC 녹화 중단

키 배치를 바꾸고 싶으면 `--robot.teleop_keys` 에 dict 전체를 넘긴다 (일부만 주면 나머지가 사라진다):

    --robot.teleop_keys='{forward: w, backward: s, left: a, right: d,
                          rotate_left: z, rotate_right: x,
                          speed_up: r, speed_down: f, quit: esc}'

주의: 베이스 주행용 R/F 와 lerobot 기본 녹화 단축키(r=재녹화, q=중단)가 겹치기
때문에, 여기서는 재녹화를 ← 키에만, 녹화 중단을 ESC 에만 할당했다 (q 는 제자리 좌회전에 쓴다).
"""

import getpass
import logging
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from pprint import pformat

from lerobot.cameras import CameraConfig  # noqa: F401  (draccus 서브클래스 등록용)
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.common.control_utils import sanity_check_dataset_robot_compatibility
from lerobot.configs import parser
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import (
    TerminalKeyListener,
    apply_recording_control,
    pynput_can_capture,
)
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

def default_teleop_keys() -> dict[str, str]:
    """베이스 주행 키 배치 (LeKiwiClientConfig.teleop_keys 와 같은 형식).

    lerobot 기본값에서 제자리 회전만 Z/X → Q/E 로 옮겼다. `--robot.teleop_keys` 로
    통째로 바꿀 수 있으며, 여기 적힌 키만 주행 입력으로 인식한다.
    """
    return {
        "forward": "w",
        "backward": "s",
        "left": "a",
        "right": "d",
        "rotate_left": "q",
        "rotate_right": "e",
        "speed_up": "r",
        "speed_down": "f",
        # LeKiwiClient 는 이 키를 쓰지 않는다 (녹화 중단은 이 스크립트가 ESC 로 직접 처리한다).
        "quit": "esc",
    }


def base_keys(teleop_keys: dict[str, str]) -> frozenset[str]:
    """주행 입력으로 인식할 키 집합 ("quit" 은 제외)."""
    return frozenset(v.lower() for name, v in teleop_keys.items() if name != "quit")


def base_help(teleop_keys: dict[str, str]) -> str:
    """현재 키 배치에 맞춘 한 줄짜리 안내 문구."""
    k = {name: teleop_keys.get(name, "?").upper() for name in default_teleop_keys()}
    return (
        f"{k['forward']}/{k['backward']} 전후, {k['left']}/{k['right']} 좌우, "
        f"{k['rotate_left']}/{k['rotate_right']} 제자리 회전, "
        f"{k['speed_up']}/{k['speed_down']} 속도, SPACE 정지"
    )


def default_cameras() -> dict[str, CameraConfig]:
    """클라이언트 쪽 카메라 설정.

    LeKiwiClient 는 카메라를 직접 열지 않는다. 이 값은 "호스트가 보내오는 프레임의
    이름과 해상도"를 선언하는 용도이며, 여기 적힌 (height, width) 가 데이터셋의
    이미지 feature shape 이 된다. 따라서 라즈베리파이 호스트의 --robot.cameras 와
    반드시 같은 해상도여야 한다 (index_or_path 는 클라이언트에서는 쓰이지 않음).
    """
    return {
        "front": OpenCVCameraConfig(index_or_path="/dev/video0", width=640, height=480, fps=30),
        "wrist": OpenCVCameraConfig(index_or_path="/dev/video2", width=640, height=480, fps=30),
    }


@dataclass
class LeKiwiDatasetConfig(DatasetRecordConfig):
    """lerobot 의 DatasetRecordConfig 와 동일하되 기본값만 LeKiwi 녹화에 맞게 바꾼 것.

    (draccus 는 중첩 dataclass 필드의 기본값을 그 클래스 자체의 필드 기본값에서 가져가므로,
    default_factory 로는 기본값을 바꿀 수 없어 서브클래스로 재정의한다.)
    """

    fps: int = 30
    episode_time_s: int | float = 30
    reset_time_s: int | float = 10
    num_episodes: int = 10
    # 로컬 녹화가 기본. 허브에 올리려면 --dataset.push_to_hub=true
    push_to_hub: bool = False
    # 실시간 인코딩: save_episode() 가 거의 즉시 끝난다. 문제가 있으면 false 로.
    streaming_encoding: bool = True
    encoder_threads: int | None = 2


@dataclass
class LeKiwiRobotArgs:
    """라즈베리파이에서 돌고 있는 lekiwi_host 에 붙기 위한 설정."""

    remote_ip: str = "192.168.0.201"
    id: str = "lekiwi01"
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556
    connect_timeout_s: int = 5
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras)
    # 베이스 주행 키 배치. 일부만 바꾸는 게 아니라 dict 전체를 주어야 한다.
    teleop_keys: dict[str, str] = field(default_factory=default_teleop_keys)

    def to_config(self) -> LeKiwiClientConfig:
        return LeKiwiClientConfig(
            remote_ip=self.remote_ip,
            id=self.id,
            port_zmq_cmd=self.port_zmq_cmd,
            port_zmq_observations=self.port_zmq_observations,
            connect_timeout_s=self.connect_timeout_s,
            cameras=self.cameras,
            teleop_keys=self.teleop_keys,
        )


@dataclass
class LeaderArmArgs:
    """PC 에 USB 로 연결된 SO100/SO101 리더암."""

    port: str = "/dev/so101-leader"
    id: str = "leader"
    use_degrees: bool = True

    def to_config(self) -> SOLeaderTeleopConfig:
        return SOLeaderTeleopConfig(port=self.port, id=self.id, use_degrees=self.use_degrees)


@dataclass
class LeKiwiRecordConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    teleop: LeaderArmArgs = field(default_factory=LeaderArmArgs)
    dataset: LeKiwiDatasetConfig = field(default_factory=LeKiwiDatasetConfig)

    # 베이스 주행 키보드 백엔드:
    #   auto     - X11 이면 pynput, Wayland/헤드리스면 터미널 입력 (권장)
    #   pynput   - 전역 키 후킹 (X11 전용, 키를 누르고 있는 동안만 이동)
    #   terminal - 이 터미널의 stdin 을 읽음 (Wayland 에서도 동작, 창 포커스 필요)
    #   none     - 베이스 주행 없음 (팔만 녹화, 베이스 속도는 0 으로 기록)
    base_control: str = "auto"
    # 터미널 백엔드에서 "키를 뗐다"고 볼 때까지의 시간(초). 키 자동반복 간격보다 커야 한다.
    base_hold_s: float = 0.35

    display_data: bool = True
    display_mode: str = "rerun"
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    # 기존 데이터셋에 이어서 녹화
    resume: bool = False
    # repo_id 뒤에 날짜/시간을 붙여 매 세션마다 새 이름을 만든다 (lerobot-record 기본 동작)
    stamp_repo_id: bool = False
    # 첫 관측을 받아 카메라 이름/해상도가 호스트와 맞는지 검사
    check_cameras: bool = True
    # 호스트가 보내는 프레임 shape 이 --robot.cameras 와 다르면 자동으로 설정을 맞춘다.
    # false 면 불일치 시 에러를 내고 멈춘다.
    auto_camera_shape: bool = True

    def validate(self) -> None:
        """필수 인자 확인 및 repo_id 보정. 실패 시 짧은 메시지와 함께 종료한다."""
        errors = []
        if self.base_control not in ("auto", "pynput", "terminal", "none"):
            errors.append(
                f"--base_control 은 auto/pynput/terminal/none 중 하나여야 합니다 (받은 값: {self.base_control})"
            )
        if not self.dataset.repo_id:
            errors.append(
                "--dataset.repo_id 가 필요합니다. 예: --dataset.repo_id=my_user/lekiwi_pick_cube "
                "(HF_USER 환경변수를 설정했다면 --dataset.repo_id=lekiwi_pick_cube 만 써도 됩니다)"
            )
        if not self.dataset.single_task:
            errors.append(
                '--dataset.single_task 가 필요합니다. 예: --dataset.single_task="Pick up the cube and place it in the box"'
            )
        if errors:
            raise SystemExit("\n".join(f"error: {e}" for e in errors))

        if "/" not in self.dataset.repo_id:
            user = os.environ.get("HF_USER") or getpass.getuser()
            self.dataset.repo_id = f"{user}/{self.dataset.repo_id}"

        namespace, _, name = self.dataset.repo_id.partition("/")
        if not namespace or not name:
            raise SystemExit(
                f"error: --dataset.repo_id 형식이 이상합니다: '{self.dataset.repo_id}'. "
                "'계정/데이터셋이름' 이어야 합니다. "
                "환경변수를 쓴다면 값이 비어있지 않은지 확인하세요 (예: export TASK_NAME=lekiwi_pick_cube)."
            )

        if self.resume:
            # LeRobotDataset.resume() 은 root 를 반드시 요구한다 (허브 스냅샷 캐시에 쓰면 캐시가 깨진다).
            # 지정하지 않았으면 기본 녹화 위치를 대신 채워 준다.
            if not self.dataset.root:
                self.dataset.root = str(HF_LEROBOT_HOME / self.dataset.repo_id)
                logging.info("--resume=true: --dataset.root 이 없어 기본 경로를 씁니다: %s", self.dataset.root)
            if not Path(self.dataset.root).exists():
                raise SystemExit(
                    f"error: 이어서 녹화할 데이터셋이 없습니다: {self.dataset.root}\n"
                    "  - --dataset.repo_id 가 맞는지 확인하세요.\n"
                    "  - 데이터셋이 다른 폴더에 있다면 --dataset.root=/경로 를 지정하세요.\n"
                    "  - 처음 녹화하는 것이라면 --resume 를 빼세요."
                )
            if self.stamp_repo_id:
                raise SystemExit(
                    "error: --resume 와 --stamp_repo_id 는 함께 쓸 수 없습니다 "
                    "(이어 녹화하려면 기존 repo_id 를 그대로 써야 합니다)."
                )


class _KeyState:
    """마지막으로 눌린 시각을 키별로 기억한다 (터미널에는 key-release 가 없으므로)."""

    def __init__(self, hold_s: float):
        self.hold_s = hold_s
        self._last: dict[str, float] = {}

    def press(self, key: str) -> None:
        self._last[key] = time.perf_counter()

    def clear(self) -> None:
        self._last.clear()

    def pressed(self) -> set[str]:
        now = time.perf_counter()
        return {k for k, t in self._last.items() if now - t < self.hold_s}


class TerminalKeyboardTeleop(KeyboardTeleop):
    """pynput 대신 터미널 입력으로 동작하는 KeyboardTeleop.

    `record_loop` 는 다중 teleop 을 쓸 때 `isinstance(t, KeyboardTeleop)` 로 베이스용
    teleop 을 찾기 때문에 KeyboardTeleop 을 상속하되, pynput 을 요구하는
    `KeyboardTeleop.__init__` 은 건너뛴다. 실제 키 입력은 `_KeyState` 를 통해
    바깥의 TerminalKeyListener 가 채워 준다.
    """

    def __init__(self, key_state: _KeyState):
        Teleoperator.__init__(self, KeyboardTeleopConfig(id="terminal_keyboard"))
        self.key_state = key_state
        self._connected = False

    @property
    def action_features(self) -> dict:
        return {}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        self._connected = True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict:
        return dict.fromkeys(self.key_state.pressed(), None)

    def send_feedback(self, feedback: dict) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False


def make_events_and_keyboard(cfg: LeKiwiRecordConfig):
    """녹화 제어 리스너와 베이스 주행용 keyboard teleop 을 만든다.

    반환: (listener, events, keyboard_teleop)

    터미널 백엔드에서는 stdin 을 읽는 주체가 하나뿐이어야 하므로, 하나의
    TerminalKeyListener 가 녹화 제어키와 베이스 주행키를 모두 받아 나눠준다.
    """
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    backend = cfg.base_control
    if backend == "auto":
        backend = "pynput" if pynput_can_capture() else "terminal"

    key_state = _KeyState(cfg.base_hold_s)
    drive_keys = base_keys(cfg.robot.teleop_keys)

    def dispatch(name: str) -> None:
        key = name.lower()
        if key in ("right", "n"):
            apply_recording_control("right", events)
        elif key == "left":
            apply_recording_control("left", events)
        elif key == "esc":
            apply_recording_control("esc", events)
        elif key == "space":
            key_state.clear()
        elif key in drive_keys:
            key_state.press(key)

    if backend == "pynput":
        # 베이스 주행은 진짜 press/release 를 쓰는 lerobot 기본 KeyboardTeleop 이 담당하고,
        # 녹화 제어키만 별도 리스너로 받는다 (r/f 가 재녹화로 오인되지 않도록).
        from pynput import keyboard as pynput_keyboard

        def on_press(key):
            name = getattr(key, "char", None)
            if name is None:
                name = {
                    pynput_keyboard.Key.right: "right",
                    pynput_keyboard.Key.left: "left",
                    pynput_keyboard.Key.esc: "esc",
                }.get(key)
            if name:
                dispatch(name)

        listener = pynput_keyboard.Listener(on_press=on_press)
        listener.start()
        logging.info("베이스 주행: pynput (X11). 키를 누르고 있는 동안 이동합니다.")
        return listener, events, KeyboardTeleop(KeyboardTeleopConfig(id="lekiwi_base"))

    if backend == "none":
        logging.info("베이스 주행 비활성화 (--base_control=none). 베이스 속도는 0 으로 기록됩니다.")
        listener = TerminalKeyListener(dispatch)
        listener.start()
        return listener, events, TerminalKeyboardTeleop(_KeyState(cfg.base_hold_s))

    listener = TerminalKeyListener(dispatch)
    listener.start()
    logging.info(
        "베이스 주행: 터미널 입력 (Wayland/헤드리스 호환). 이 터미널 창이 포커스를 갖고 있어야 합니다."
    )
    return listener, events, TerminalKeyboardTeleop(key_state)


def sync_cameras(robot: LeKiwiClient, auto_adapt: bool = True, timeout_s: float = 10.0) -> None:
    """호스트가 실제로 보내는 프레임과 --robot.cameras 설정을 맞춘다.

    LeKiwiClient 는 카메라를 직접 열지 않으므로, 설정된 width/height 는 "호스트가 이런
    모양으로 보내온다"는 선언일 뿐이다. 실제로 오는 프레임이 다르면 데이터셋에 프레임을
    넣을 때 터진다. auto_adapt=True 면 실제로 받은 shape 에 맞춰 설정을 고쳐 준다
    (호스트의 카메라 회전 설정 때문에 세로/가로가 바뀌는 경우가 흔하다).
    """
    expected = {name: shape for name, shape in robot.observation_features.items() if isinstance(shape, tuple)}
    deadline = time.perf_counter() + timeout_s
    frames: dict = {}
    while time.perf_counter() < deadline:
        obs = robot.get_observation()
        frames = {name: obs[name] for name in expected if name in obs and obs[name] is not None}
        if len(frames) == len(expected):
            break
        time.sleep(0.1)

    missing = sorted(set(expected) - set(frames))
    if missing:
        raise RuntimeError(
            f"호스트로부터 카메라 {missing} 프레임을 받지 못했습니다. "
            "라즈베리파이의 lekiwi_host --robot.cameras 에 있는 카메라 이름과 "
            "이 스크립트의 --robot.cameras 이름이 같은지 확인하세요."
        )

    adapted = False
    for name, frame in frames.items():
        actual = tuple(frame.shape)
        if actual == tuple(expected[name]):
            continue
        cam_cfg = robot.config.cameras[name]
        if not auto_adapt:
            raise RuntimeError(
                f"카메라 '{name}' 해상도 불일치: 호스트가 보낸 프레임 {actual} != "
                f"설정값 {tuple(expected[name])} (height, width, 3).\n"
                "  - 설정을 프레임에 맞추려면: --robot.cameras='{"
                f"{name}: {{type: opencv, index_or_path: {cam_cfg.index_or_path}, "
                f"width: {frame.shape[1]}, height: {frame.shape[0]}, fps: {cam_cfg.fps}}}}}'\n"
                "  - 또는 라즈베리파이의 lekiwi_host 를 원하는 해상도/rotation 으로 다시 띄우세요.\n"
                "  - --auto_camera_shape=true 로 두면 받은 프레임에 맞춰 자동으로 보정합니다."
            )
        logging.warning(
            "카메라 '%s' 설정 %s → 호스트가 실제로 보낸 %s 로 자동 보정합니다. "
            "의도한 해상도가 아니라면 라즈베리파이의 lekiwi_host --robot.cameras 를 확인하세요.",
            name,
            tuple(expected[name]),
            actual,
        )
        cam_cfg.height, cam_cfg.width = int(frame.shape[0]), int(frame.shape[1])
        adapted = True

    if adapted:
        # observation_features / _cameras_ft 는 cached_property 라 다시 계산되게 캐시를 비운다.
        for cached in ("_cameras_ft", "observation_features"):
            robot.__dict__.pop(cached, None)

    logging.info("카메라 확인 완료: %s", {k: tuple(v.shape) for k, v in frames.items()})


@parser.wrap()
def record(cfg: LeKiwiRecordConfig) -> LeRobotDataset:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    leader_arm = SOLeader(cfg.teleop.to_config())

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if cfg.display_data:
        init_visualization(
            cfg.display_mode, session_name="lekiwi_record", ip=cfg.display_ip, port=cfg.display_port
        )
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    dataset = None
    listener = None
    keyboard = None
    num_cameras = len(cfg.robot.cameras)

    try:
        # 데이터셋 폴더를 만들기 전에 먼저 연결한다. 연결 실패 시 빈 데이터셋이 남아
        # 다음 실행에서 "이미 존재하는 repo_id" 로 막히는 일을 막기 위함이다.
        logging.info(
            "LeKiwi 호스트(%s:%s)에 접속 중... 라즈베리파이에서 lekiwi_host 가 실행 중이어야 합니다.",
            cfg.robot.remote_ip,
            cfg.robot.port_zmq_cmd,
        )
        try:
            robot.connect()
        except DeviceNotConnectedError as e:
            raise SystemExit(
                f"error: LeKiwi 호스트({cfg.robot.remote_ip})에 연결하지 못했습니다: {e}\n"
                "  - 라즈베리파이에서 lekiwi_host 가 실행 중인지 확인하세요.\n"
                f"  - IP 가 맞는지 확인하세요: ping {cfg.robot.remote_ip}\n"
                f"  - 포트({cfg.robot.port_zmq_cmd}/{cfg.robot.port_zmq_observations})가 호스트 설정과 같은지, "
                "방화벽에 막히지 않는지 확인하세요."
            ) from e
        leader_arm.connect()

        if cfg.check_cameras:
            sync_cameras(robot, auto_adapt=cfg.auto_camera_shape)

        # 데이터셋 feature 는 카메라 shape 이 확정된 뒤에 만든다.
        dataset_features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_action_processor,
                initial_features=create_initial_features(action=robot.action_features),
                use_videos=cfg.dataset.video,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=robot_observation_processor,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=cfg.dataset.video,
            ),
        )
        logging.info("데이터셋 feature: %s", list(dataset_features))

        if cfg.resume:
            dataset = LeRobotDataset.resume(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                rgb_encoder=cfg.dataset.rgb_encoder,
                depth_encoder=cfg.dataset.depth_encoder,
                encoder_threads=cfg.dataset.encoder_threads,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras,
            )
            sanity_check_dataset_robot_compatibility(dataset, robot, cfg.dataset.fps, dataset_features)
        else:
            if cfg.stamp_repo_id:
                cfg.dataset.stamp_repo_id()
            dataset = LeRobotDataset.create(
                cfg.dataset.repo_id,
                cfg.dataset.fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                rgb_encoder=cfg.dataset.rgb_encoder,
                depth_encoder=cfg.dataset.depth_encoder,
                encoder_threads=cfg.dataset.encoder_threads,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
            )

        listener, events, keyboard = make_events_and_keyboard(cfg)
        keyboard.connect()

        print(
            f"\n조작키 | 베이스: {base_help(cfg.robot.teleop_keys)}"
            "\n       | 녹화  : →(N) 에피소드 종료, ← 재녹화, ESC 녹화 중단\n"
        )

        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=cfg.dataset.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    dataset=dataset,
                    teleop=[leader_arm, keyboard],
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    display_mode=cfg.display_mode,
                    display_compressed_images=display_compressed_images,
                )

                # 마지막 에피소드가 아니면 환경을 되돌릴 시간을 준다 (녹화하지 않음)
                if not events["stop_recording"] and (
                    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
                ):
                    log_say("Reset the environment", cfg.play_sounds)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=cfg.dataset.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=[leader_arm, keyboard],
                        control_time_s=cfg.dataset.reset_time_s,
                        single_task=cfg.dataset.single_task,
                        display_data=cfg.display_data,
                        display_mode=cfg.display_mode,
                        display_compressed_images=display_compressed_images,
                    )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        log_say("Stop recording", cfg.play_sounds, blocking=True)

        if dataset is not None:
            dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if leader_arm.is_connected:
            leader_arm.disconnect()
        if keyboard is not None and keyboard.is_connected:
            keyboard.disconnect()
        if listener is not None:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)

        if cfg.dataset.push_to_hub:
            if dataset is not None and dataset.num_episodes > 0:
                dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            else:
                logging.warning("저장된 에피소드가 없어 허브 업로드를 건너뜁니다.")

        if dataset is not None:
            logging.info("데이터셋 위치: %s", dataset.root)


if __name__ == "__main__":
    record()
