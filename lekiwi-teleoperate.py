#!/usr/bin/env python
r"""LeKiwi 텔레오퍼레이션 (녹화 없음, CLI 인자로 전부 설정).

`lerobot-teleoperate` 는 teleop 을 하나만 받기 때문에 LeKiwi 에 쓸 수 없다
(`lerobot/scripts/lerobot_teleoperate.py` 의 TODO 참고). LeKiwi 는 팔(리더암) +
베이스(키보드) 두 개가 동시에 필요해서, 이 스크립트가 그 조합을 만들어 준다.
설정 인자는 `lekiwi-record.py` 와 동일하다 (`--robot.*`, `--teleop.*`, `--display_data`).

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

그 다음 PC 에서:

    python lekiwi-teleoperate.py \
        --display_data=true

원하면 lekiwi-record.py 와 똑같이 전부 인자로 줄 수 있다:

    python lekiwi-teleoperate.py \
        --robot.remote_ip=192.168.0.201 \
        --robot.id=lekiwi01 \
        --teleop.port=/dev/so101-leader \
        --teleop.id=leader \
        --fps=30 \
        --teleop_time_s=60 \
        --display_data=true

전체 옵션은 `python lekiwi-teleoperate.py --help`.

조작키
    리더암      : 그대로 따라 움직인다
    베이스 주행 : W/S 전후, A/D 좌우, Z/X 회전, R/F 속도 단계, SPACE 정지
    종료        : ESC (Ctrl+C 도 됨). 종료 시 베이스를 정지시키고 연결을 끊는다.

호스트의 카메라 rotation 설정에 따라 프레임의 가로/세로가 바뀐다. 이 스크립트는 화면
표시에만 쓰므로 해상도가 달라도 그냥 돌아가지만, 시작할 때 실제로 받은 카메라 이름과
shape 을 한 번 찍어 준다 (`--check_cameras=false` 로 끌 수 있음).
"""

import getpass  # noqa: F401  (lekiwi-record.py 와 인자 구조를 맞추기 위해 유지)
import logging
import time
from dataclasses import dataclass, field
from pprint import pformat

from lerobot.cameras import CameraConfig  # noqa: F401  (draccus 서브클래스 등록용)
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.keyboard_input import TerminalKeyListener, pynput_can_capture
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import (
    init_visualization,
    log_visualization_data,
    shutdown_visualization,
)

# LeKiwi 베이스 주행에 쓰는 키 (LeKiwiClientConfig.teleop_keys 기본값과 동일)
BASE_KEYS = frozenset("wasdzxrf")
SPEED_KEYS = ("r", "f")
# 속도 단계 키가 매 프레임 먹혀서 순식간에 최대/최소로 튀는 것을 막는다.
SPEED_KEY_INTERVAL_S = 0.4


def default_cameras() -> dict[str, CameraConfig]:
    """클라이언트 쪽 카메라 설정 (호스트가 보내오는 프레임의 이름과 크기 선언).

    LeKiwiClient 는 카메라를 직접 열지 않는다. 여기서는 화면 표시에만 쓰이므로
    크기가 달라도 동작하지만, 이름은 호스트와 같아야 프레임을 받는다.
    """
    return {
        "front": OpenCVCameraConfig(index_or_path="/dev/video0", width=640, height=480, fps=30),
        "wrist": OpenCVCameraConfig(index_or_path="/dev/video2", width=640, height=480, fps=30),
    }


@dataclass
class LeKiwiRobotArgs:
    """라즈베리파이에서 돌고 있는 lekiwi_host 에 붙기 위한 설정."""

    remote_ip: str = "192.168.0.201"
    id: str = "lekiwi01"
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556
    connect_timeout_s: int = 5
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras)

    def to_config(self) -> LeKiwiClientConfig:
        return LeKiwiClientConfig(
            remote_ip=self.remote_ip,
            id=self.id,
            port_zmq_cmd=self.port_zmq_cmd,
            port_zmq_observations=self.port_zmq_observations,
            connect_timeout_s=self.connect_timeout_s,
            cameras=self.cameras,
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
class LeKiwiTeleopConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    teleop: LeaderArmArgs = field(default_factory=LeaderArmArgs)

    # 제어 루프 주기
    fps: int = 30
    # 이 시간(초)이 지나면 자동 종료. None 이면 ESC/Ctrl+C 까지 계속.
    teleop_time_s: float | None = None

    # 베이스 주행 키보드 백엔드:
    #   auto     - X11 이면 pynput, Wayland/헤드리스면 터미널 입력 (권장)
    #   pynput   - 전역 키 후킹 (X11 전용, 키를 누르고 있는 동안만 이동)
    #   terminal - 이 터미널의 stdin 을 읽음 (Wayland 에서도 동작, 창 포커스 필요)
    #   none     - 베이스 주행 없음 (팔만 조작)
    base_control: str = "auto"
    # 터미널 백엔드에서 "키를 뗐다"고 볼 때까지의 시간(초). 키 자동반복 간격보다 커야 한다.
    base_hold_s: float = 0.35

    display_data: bool = True
    display_mode: str = "rerun"
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    # 시작할 때 호스트에서 실제로 오는 카메라 이름/shape 을 한 번 확인해 로그로 남긴다.
    check_cameras: bool = True
    # 한 줄짜리 상태 표시 (베이스 속도, 루프 주파수)
    print_status: bool = True

    def validate(self) -> None:
        if self.base_control not in ("auto", "pynput", "terminal", "none"):
            raise SystemExit(
                f"error: --base_control 은 auto/pynput/terminal/none 중 하나여야 합니다 "
                f"(받은 값: {self.base_control})"
            )
        if self.fps <= 0:
            raise SystemExit(f"error: --fps 는 1 이상이어야 합니다 (받은 값: {self.fps})")


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

    pynput 을 요구하는 `KeyboardTeleop.__init__` 은 건너뛰고, 실제 키 입력은
    바깥의 TerminalKeyListener 가 `_KeyState` 에 채워 준다.
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


class SpeedKeyThrottle:
    """R/F(속도 단계) 키가 매 프레임 반복 적용되어 순식간에 끝까지 튀는 것을 막는다.

    `_from_keyboard_to_base_action` 은 키가 눌려 있는 동안 매 호출마다 speed_index 를
    올리고 내리기 때문에, 최소 간격을 두고 한 번씩만 통과시킨다.
    """

    def __init__(self, min_interval_s: float = SPEED_KEY_INTERVAL_S):
        self.min_interval_s = min_interval_s
        self._last_applied = 0.0

    def filter(self, keys: dict) -> dict:
        if not any(k in keys for k in SPEED_KEYS):
            return keys
        now = time.perf_counter()
        if now - self._last_applied >= self.min_interval_s:
            self._last_applied = now
            return keys
        return {k: v for k, v in keys.items() if k not in SPEED_KEYS}


def make_keyboard(cfg: LeKiwiTeleopConfig):
    """종료키 리스너와 베이스 주행용 keyboard teleop 을 만든다.

    반환: (listener, state, keyboard_teleop) — state["quit"] 가 True 가 되면 루프 종료.

    터미널 백엔드에서는 stdin 을 읽는 주체가 하나뿐이어야 하므로, 하나의
    TerminalKeyListener 가 종료키와 주행키를 모두 받아 나눠준다.
    """
    state = {"quit": False}
    backend = cfg.base_control
    if backend == "auto":
        backend = "pynput" if pynput_can_capture() else "terminal"

    key_state = _KeyState(cfg.base_hold_s)

    def dispatch(name: str) -> None:
        key = name.lower()
        if key == "esc":
            print("\n종료합니다...")
            state["quit"] = True
        elif key == "space":
            key_state.clear()
        elif key in BASE_KEYS:
            key_state.press(key)

    if backend == "pynput":
        # 베이스 주행은 진짜 press/release 를 쓰는 lerobot 기본 KeyboardTeleop 이 담당하고,
        # 종료키만 별도 리스너로 받는다.
        from pynput import keyboard as pynput_keyboard

        def on_press(key):
            name = getattr(key, "char", None)
            if name is None and key == pynput_keyboard.Key.esc:
                name = "esc"
            if name:
                dispatch(name)

        listener = pynput_keyboard.Listener(on_press=on_press)
        listener.start()
        logging.info("베이스 주행: pynput (X11). 키를 누르고 있는 동안 이동합니다.")
        return listener, state, KeyboardTeleop(KeyboardTeleopConfig(id="lekiwi_base"))

    listener = TerminalKeyListener(dispatch)
    listener.start()
    if backend == "none":
        logging.info("베이스 주행 비활성화 (--base_control=none). 팔만 조작합니다.")
        return listener, state, TerminalKeyboardTeleop(_KeyState(cfg.base_hold_s))

    logging.info(
        "베이스 주행: 터미널 입력 (Wayland/헤드리스 호환). 이 터미널 창이 포커스를 갖고 있어야 합니다."
    )
    return listener, state, TerminalKeyboardTeleop(key_state)


def probe_cameras(robot: LeKiwiClient, timeout_s: float = 3.0) -> None:
    """호스트에서 실제로 오는 카메라 이름/shape 을 한 번 확인해 로그로 남긴다 (실패해도 계속)."""
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
        logging.warning(
            "카메라 %s 프레임을 받지 못했습니다. 호스트의 --robot.cameras 이름과 맞는지 확인하세요. "
            "(텔레오퍼레이션 자체는 계속 진행합니다)",
            missing,
        )
    for name, frame in frames.items():
        if tuple(frame.shape) != tuple(expected[name]):
            logging.warning(
                "카메라 '%s' shape %s (설정값 %s). 표시에는 문제 없지만 녹화 시에는 맞춰야 합니다.",
                name,
                tuple(frame.shape),
                tuple(expected[name]),
            )
    if frames:
        logging.info("수신 중인 카메라: %s", {k: tuple(v.shape) for k, v in frames.items()})


def teleop_loop(
    robot: LeKiwiClient,
    leader_arm: SOLeader,
    keyboard: KeyboardTeleop,
    state: dict,
    cfg: LeKiwiTeleopConfig,
) -> None:
    throttle = SpeedKeyThrottle()
    start = time.perf_counter()
    control_interval = 1.0 / cfg.fps

    while not state["quit"]:
        loop_start = time.perf_counter()

        observation = robot.get_observation()

        arm_action = {f"arm_{k}": v for k, v in leader_arm.get_action().items()}
        base_action = robot._from_keyboard_to_base_action(throttle.filter(keyboard.get_action()))
        action = {**arm_action, **base_action}

        robot.send_action(action)

        if cfg.display_data:
            log_visualization_data(
                cfg.display_mode,
                observation=observation,
                action=action,
                compress_images=cfg.display_compressed_images,
            )

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(control_interval - dt_s, 0.0))

        if cfg.print_status:
            loop_s = time.perf_counter() - loop_start
            speed = robot.speed_levels[robot.speed_index]
            print(
                f"\rx={base_action['x.vel']:+.2f} y={base_action['y.vel']:+.2f} "
                f"theta={base_action['theta.vel']:+6.1f} | speed={speed['xy']} m/s "
                f"| {1 / loop_s:5.1f} Hz   ",
                end="",
                flush=True,
            )

        if cfg.teleop_time_s is not None and time.perf_counter() - start >= cfg.teleop_time_s:
            print(f"\n--teleop_time_s={cfg.teleop_time_s} 경과, 종료합니다.")
            return


@parser.wrap()
def teleoperate(cfg: LeKiwiTeleopConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    leader_arm = SOLeader(cfg.teleop.to_config())

    listener = None
    keyboard = None

    try:
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
            probe_cameras(robot)

        if cfg.display_data:
            init_visualization(
                cfg.display_mode, session_name="lekiwi_teleop", ip=cfg.display_ip, port=cfg.display_port
            )

        listener, state, keyboard = make_keyboard(cfg)
        keyboard.connect()

        print(
            "\n조작키 | 베이스: W/S 전후, A/D 좌우, Z/X 회전, R/F 속도, SPACE 정지"
            "\n       | 종료  : ESC\n"
        )

        teleop_loop(robot, leader_arm, keyboard, state, cfg)
    except KeyboardInterrupt:
        print("\nCtrl+C, 종료합니다...")
    finally:
        # 베이스를 정지시키고, 팔은 현재 자세를 그대로 목표로 줘서 튀지 않게 한 뒤 연결을 끊는다.
        if robot.is_connected:
            try:
                hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
                robot.send_action({**hold, "x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
                time.sleep(0.2)
            except Exception as e:
                logging.warning("정지 명령 전송 실패: %s", e)
            robot.disconnect()
        if leader_arm.is_connected:
            leader_arm.disconnect()
        if keyboard is not None and keyboard.is_connected:
            keyboard.disconnect()
        if listener is not None:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
        print()


if __name__ == "__main__":
    teleoperate()
