#!/usr/bin/env python
r"""LeKiwi 에피소드 리플레이 (녹화된 action 을 그대로 다시 실행, 리더암 불필요).

`lerobot-replay` 는 `lekiwi_client` 를 import 하지 않아 `--robot.type=lekiwi_client` 를
알아보지 못하고, 중간에 멈출 방법도 없다. 바퀴가 달린 로봇에서 그건 위험하므로 이
스크립트가 ESC 중단 + 시작 자세로의 부드러운 이동 + 종료 시 베이스 정지를 채워 준다.

먼저 라즈베리파이(LeKiwi) 쪽에서 호스트를 띄워 둘 것:

    python -m lerobot.robots.lekiwi.lekiwi_host \
        --robot.id=lekiwi01 \
        --robot.cameras='{
          front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
          wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
        }' \
        --host.connection_time_s=14400

그 다음 PC 에서:

    HF_USER=your_hf_id
    TASK_NAME=lekiwi_pick_cube

    python lekiwi-replay.py \
        --dataset.repo_id=${HF_USER}/${TASK_NAME} \
        --dataset.episode=0

--robot.* 기본값이 이미 위 장비에 맞춰져 있으므로 실제로는 위 두 줄이면 된다.
전부 명시하고 싶다면:

    python lekiwi-replay.py \
        --robot.remote_ip=192.168.0.201 \
        --robot.id=lekiwi01 \
        --dataset.repo_id=${HF_USER}/${TASK_NAME} \
        --dataset.episode=0 \
        --dataset.root=/path/to/local/dataset \
        --warmup_s=2.0 \
        --display_data=true

전체 옵션은 `python lekiwi-replay.py --help`.

카메라 주의: 리플레이는 데이터셋의 action 만 재생하므로 카메라 설정은 화면 표시에만 쓰인다.
호스트와 이름이 달라도 동작하지만, 이름이 같아야 rerun 에 프레임이 뜬다
(`--robot.cameras` 의 index_or_path 는 클라이언트에서 쓰이지 않는다).

안전 주의
    - 리플레이는 녹화 당시의 절대 관절 각도와 베이스 속도를 그대로 재생한다. 로봇 주변
      환경이 녹화 때와 다르면 그대로 충돌한다. 반드시 주변을 비우고 실행할 것.
    - 시작 시 팔이 첫 프레임 자세로 튀는 것을 막기 위해 `--warmup_s` 초 동안 현재
      자세에서 첫 자세까지 선형 보간으로 이동한다 (이 구간에서 베이스는 정지).
    - ESC(또는 Ctrl+C) 로 언제든 중단할 수 있고, 중단 시 베이스를 정지시킨다.

조작키
    ESC : 즉시 중단 (베이스 정지 후 연결 해제)
"""

import getpass
import logging
import os
import time
from dataclasses import dataclass, field
from pprint import pformat

from lerobot.cameras import CameraConfig  # noqa: F401  (draccus 서브클래스 등록용)
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs import parser
from lerobot.datasets import LeRobotDataset
from lerobot.processor import make_default_robot_action_processor
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.constants import ACTION
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.keyboard_input import TerminalKeyListener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import (
    init_visualization,
    log_visualization_data,
    shutdown_visualization,
)

# 호스트의 send_action 은 x/y/theta.vel 이 반드시 있다고 가정한다 (robots/lekiwi/lekiwi.py:394).
# 데이터셋에 베이스 채널이 없으면 0 으로 채워 넣는다.
BASE_VEL_KEYS = ("x.vel", "y.vel", "theta.vel")
STOP_BASE = dict.fromkeys(BASE_VEL_KEYS, 0.0)


def default_cameras() -> dict[str, CameraConfig]:
    """클라이언트 쪽 카메라 설정 (호스트가 보내오는 프레임의 이름과 크기 선언).

    LeKiwiClient 는 카메라를 직접 열지 않는다. 리플레이에서는 화면 표시에만 쓰이므로
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
class DatasetReplayArgs:
    """재생할 데이터셋과 에피소드."""

    repo_id: str = ""
    episode: int = 0
    # 로컬 데이터셋 경로. None 이면 $HF_LEROBOT_HOME/repo_id 에서 찾는다.
    root: str | None = None
    # 재생 주기. None 이면 데이터셋에 기록된 fps 를 그대로 쓴다 (권장).
    fps: int | None = None


@dataclass
class LeKiwiReplayConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    dataset: DatasetReplayArgs = field(default_factory=DatasetReplayArgs)

    # 첫 프레임 자세까지 부드럽게 이동하는 데 쓸 시간(초). 0 이면 곧바로 첫 action 을 보낸다.
    warmup_s: float = 2.0
    # 워밍업이 끝나고 재생을 시작하기 전에 기다리는 시간(초). 손을 뺄 여유.
    countdown_s: float = 2.0

    display_data: bool = True
    display_mode: str = "rerun"
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    # 한 줄짜리 진행 상태 표시 (프레임 번호, 베이스 속도, 루프 주파수)
    print_status: bool = True

    def validate(self) -> None:
        """필수 인자 확인 및 repo_id 보정. 실패 시 짧은 메시지와 함께 종료한다."""
        errors = []
        if not self.dataset.repo_id:
            errors.append(
                "--dataset.repo_id 가 필요합니다. 예: --dataset.repo_id=my_user/lekiwi_pick_cube "
                "(HF_USER 환경변수를 설정했다면 --dataset.repo_id=lekiwi_pick_cube 만 써도 됩니다)"
            )
        if self.dataset.episode < 0:
            errors.append(f"--dataset.episode 는 0 이상이어야 합니다 (받은 값: {self.dataset.episode})")
        if self.dataset.fps is not None and self.dataset.fps <= 0:
            errors.append(f"--dataset.fps 는 1 이상이어야 합니다 (받은 값: {self.dataset.fps})")
        if errors:
            raise SystemExit("\n".join(f"error: {e}" for e in errors))

        if "/" not in self.dataset.repo_id:
            user = os.environ.get("HF_USER") or getpass.getuser()
            self.dataset.repo_id = f"{user}/{self.dataset.repo_id}"

        namespace, _, name = self.dataset.repo_id.partition("/")
        if not namespace or not name:
            raise SystemExit(
                f"error: --dataset.repo_id 형식이 이상합니다: '{self.dataset.repo_id}'. "
                "'계정/데이터셋이름' 이어야 합니다."
            )


def make_abort_listener():
    """ESC 를 누르면 state["quit"] 를 True 로 만드는 리스너를 띄운다.

    반환: (listener, state)

    리플레이는 팔을 잡고 하는 작업이 아니라 discrete 한 중단키만 필요하므로,
    pynput 이 없어도 되는 TerminalKeyListener 하나만 쓴다 (이 터미널 창이 포커스를
    갖고 있어야 키가 들어온다).
    """
    state = {"quit": False}

    def dispatch(name: str) -> None:
        if name.lower() == "esc":
            print("\nESC, 중단합니다...")
            state["quit"] = True

    listener = TerminalKeyListener(dispatch)
    listener.start()
    return listener, state


def load_episode(cfg: LeKiwiReplayConfig) -> tuple[LeRobotDataset, list[str]]:
    """에피소드 하나만 담은 데이터셋과 action 이름 목록을 돌려준다."""
    try:
        dataset = LeRobotDataset(
            cfg.dataset.repo_id, root=cfg.dataset.root, episodes=[cfg.dataset.episode]
        )
    except Exception as e:
        raise SystemExit(
            f"error: 데이터셋 '{cfg.dataset.repo_id}' 의 에피소드 {cfg.dataset.episode} 를 열지 못했습니다: {e}\n"
            "  - repo_id 와 --dataset.episode 번호가 맞는지 확인하세요.\n"
            "  - 로컬 폴더에 있다면 --dataset.root=/경로 를 지정하세요."
        ) from e

    if dataset.num_frames == 0:
        raise SystemExit(f"error: 에피소드 {cfg.dataset.episode} 에 프레임이 없습니다.")

    action_names = list(dataset.features[ACTION]["names"])
    return dataset, action_names


def check_action_names(action_names: list[str], robot: LeKiwiClient) -> None:
    """데이터셋의 action 채널이 LeKiwi 가 받을 수 있는 것인지 확인한다.

    팔 관절(.pos)이 하나라도 빠져 있으면 0 으로 채울 수 없다 (관절을 0 도로 보내는 건
    위험하다). 반면 베이스 속도(.vel)는 빠져 있으면 0 = 정지로 채워도 안전하다.
    """
    required = set(robot.action_features)
    missing = required - set(action_names)
    missing_pos = sorted(k for k in missing if k.endswith(".pos"))
    missing_vel = sorted(k for k in missing if k.endswith(".vel"))
    extra = sorted(set(action_names) - required)

    if missing_pos:
        raise SystemExit(
            f"error: 데이터셋의 action 에 팔 관절 {missing_pos} 이 없습니다.\n"
            f"  데이터셋 action: {action_names}\n"
            f"  LeKiwi 가 필요로 하는 action: {sorted(required)}\n"
            "  이 데이터셋은 LeKiwi 로 녹화된 것이 아닌 것 같습니다."
        )
    if missing_vel:
        logging.warning(
            "데이터셋에 베이스 속도 %s 가 없어 0(정지)으로 채웁니다. 팔만 재생됩니다.", missing_vel
        )
    if extra:
        logging.warning("LeKiwi 가 쓰지 않는 action 채널 %s 는 무시됩니다.", extra)


def move_to_start(
    robot: LeKiwiClient, target: dict[str, float], warmup_s: float, fps: int, state: dict
) -> None:
    """현재 팔 자세에서 첫 프레임 자세까지 선형 보간으로 천천히 이동한다 (베이스는 정지).

    이걸 건너뛰면 첫 send_action 에서 팔이 목표 자세로 최대 속도로 튄다.
    """
    if warmup_s <= 0:
        return

    obs = robot.get_observation()
    start = {k: float(obs[k]) for k in target if k.endswith(".pos") and k in obs}
    if not start:
        logging.warning("현재 팔 자세를 읽지 못해 워밍업을 건너뜁니다.")
        return

    steps = max(int(warmup_s * fps), 1)
    logging.info("첫 프레임 자세로 %.1f 초 동안 이동합니다...", warmup_s)
    for step in range(1, steps + 1):
        if state["quit"]:
            return
        loop_start = time.perf_counter()
        alpha = step / steps
        action = {k: (1.0 - alpha) * v + alpha * float(target[k]) for k, v in start.items()}
        robot.send_action({**action, **STOP_BASE})
        precise_sleep(max(1 / fps - (time.perf_counter() - loop_start), 0.0))


def replay_loop(
    robot: LeKiwiClient,
    dataset: LeRobotDataset,
    action_names: list[str],
    robot_action_processor,
    cfg: LeKiwiReplayConfig,
    fps: int,
    state: dict,
) -> int:
    """데이터셋의 action 을 한 프레임씩 재생한다. 실제로 재생한 프레임 수를 돌려준다."""
    actions = dataset.select_columns(ACTION)
    control_interval = 1.0 / fps
    replayed = 0

    for idx in range(dataset.num_frames):
        if state["quit"]:
            break
        loop_start = time.perf_counter()

        action_array = actions[idx][ACTION]
        action = {name: float(action_array[i]) for i, name in enumerate(action_names)}
        # 베이스 채널이 없는 데이터셋이어도 호스트가 KeyError 를 내지 않도록 정지값을 깔아 둔다.
        action = {**STOP_BASE, **action}

        obs = robot.get_observation()
        robot.send_action(robot_action_processor((action, obs)))
        replayed += 1

        if cfg.display_data:
            log_visualization_data(
                cfg.display_mode,
                observation=obs,
                action=action,
                compress_images=cfg.display_compressed_images,
            )

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(control_interval - dt_s, 0.0))

        if cfg.print_status:
            loop_s = time.perf_counter() - loop_start
            print(
                f"\r[{idx + 1:5d}/{dataset.num_frames}] "
                f"x={action['x.vel']:+.2f} y={action['y.vel']:+.2f} "
                f"theta={action['theta.vel']:+6.1f} | {1 / loop_s:5.1f} Hz   ",
                end="",
                flush=True,
            )

    return replayed


@parser.wrap()
def replay(cfg: LeKiwiReplayConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    robot_action_processor = make_default_robot_action_processor()

    dataset, action_names = load_episode(cfg)
    fps = cfg.dataset.fps or dataset.fps
    logging.info(
        "에피소드 %d: %d 프레임, %.1f 초 @ %d fps (데이터셋 fps=%s)",
        cfg.dataset.episode,
        dataset.num_frames,
        dataset.num_frames / fps,
        fps,
        dataset.fps,
    )

    listener = None

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

        check_action_names(action_names, robot)

        if cfg.display_data:
            init_visualization(
                cfg.display_mode, session_name="lekiwi_replay", ip=cfg.display_ip, port=cfg.display_port
            )

        listener, state = make_abort_listener()
        print("\n중단하려면 ESC (이 터미널 창이 포커스를 갖고 있어야 합니다). Ctrl+C 도 됩니다.\n")

        first_action_array = dataset.select_columns(ACTION)[0][ACTION]
        first_action = {name: float(first_action_array[i]) for i, name in enumerate(action_names)}
        move_to_start(robot, first_action, cfg.warmup_s, fps, state)

        if cfg.countdown_s > 0 and not state["quit"]:
            logging.info("%.1f 초 뒤에 재생을 시작합니다. 로봇에서 손을 떼세요.", cfg.countdown_s)
            time.sleep(cfg.countdown_s)

        if not state["quit"]:
            log_say("Replaying episode", cfg.play_sounds, blocking=True)
            replayed = replay_loop(robot, dataset, action_names, robot_action_processor, cfg, fps, state)
            print()
            logging.info("재생 완료: %d / %d 프레임", replayed, dataset.num_frames)
    except KeyboardInterrupt:
        print("\nCtrl+C, 중단합니다...")
    finally:
        # 베이스를 정지시키고, 팔은 현재 자세를 그대로 목표로 줘서 튀지 않게 한 뒤 연결을 끊는다.
        if robot.is_connected:
            try:
                hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
                robot.send_action({**hold, **STOP_BASE})
                time.sleep(0.2)
            except Exception as e:
                logging.warning("정지 명령 전송 실패: %s", e)
            robot.disconnect()
        if listener is not None:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
        print()


if __name__ == "__main__":
    replay()
