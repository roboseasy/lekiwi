#!/usr/bin/env python
r"""LeKiwi 정책 평가 (실물 로봇에서 N 에피소드 돌리고 성공률 리포트, 리더암 불필요).

`lerobot-eval` 은 gym 시뮬레이션 환경(`--env.type=pusht` 등)에서만 동작한다. 실물
LeKiwi 에는 환경도 리워드도 없으므로, 여기서는 사람이 에피소드마다 성공/실패를 눌러
주고 그걸 모아 성공률과 추론 지연 통계를 내는 방식으로 평가한다.

`lekiwi-rollout.py` 와의 차이:
    rollout   - 정책을 계속 돌린다. 사람이 보고 판단. 리포트 없음.
    evaluate  - 에피소드/리셋 구간을 나눠 N 번 돌리고, 성공률과 지연 통계를 JSON 으로 남긴다.

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

    python lekiwi-evaluate.py \
        --policy.path=outputs/train/${TASK_NAME}/checkpoints/last/pretrained_model \
        --task="Pick up the cube and place it in the box" \
        --eval.n_episodes=10 \
        --eval.episode_time_s=30 \
        --eval.reset_time_s=10

두 체크포인트를 비교하고 싶으면 같은 조건으로 두 번 돌리고 리포트를 비교하면 된다:

    python lekiwi-evaluate.py --policy.path=.../040000/pretrained_model --eval.tag=step040k ...
    python lekiwi-evaluate.py --policy.path=.../080000/pretrained_model --eval.tag=step080k ...

전체 옵션은 `python lekiwi-evaluate.py --help`.

조작키
    에피소드 진행 중 : S 성공으로 종료, F 실패로 종료, →(N) 판정 보류하고 종료,
                       SPACE 일시정지/재개, ESC 평가 전체 중단
    판정 대기 중     : S 성공, F 실패, ESC 중단
    리셋 구간        : →(N) 리셋 끝내고 다음 에피소드로, ESC 중단

리셋 구간에는 로봇이 현재 자세를 유지한 채 멈춰 있으므로, 그동안 물체를 제자리에
돌려놓으면 된다. 판정을 보류한 채 끝난 에피소드는 바로 뒤에서 S/F 를 물어본다.

안전 주의
    - 정책은 학습 때 본 적 없는 상황에서 무엇이든 할 수 있다. 주변을 비우고, 손은
      SPACE/ESC 위에 두고, 처음에는 `--base_speed_scale` 을 낮춰서 시작할 것.
    - 에피소드마다 시작 전에 `--warmup_s` 초 동안 현재 자세에서 첫 목표 자세까지
      선형 보간으로 이동한다 (이 구간에서 베이스는 정지).
"""

import json
import logging
import time
import unicodedata
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from pprint import pformat

import numpy as np
import torch

from lerobot.cameras import CameraConfig  # noqa: F401  (draccus 서브클래스 등록용)
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs import PreTrainedConfig, parser
from lerobot.datasets import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.policies import get_policy_class, make_pre_post_processors
from lerobot.policies.utils import build_inference_frame, make_robot_action
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.constants import ACTION, OBS_STATE
from lerobot.utils.device_utils import auto_select_torch_device, is_torch_device_available
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import TerminalKeyListener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import (
    init_visualization,
    log_visualization_data,
    shutdown_visualization,
)

# 호스트의 send_action 은 x/y/theta.vel 이 반드시 있다고 가정한다 (robots/lekiwi/lekiwi.py:394).
BASE_VEL_KEYS = ("x.vel", "y.vel", "theta.vel")
STOP_BASE = dict.fromkeys(BASE_VEL_KEYS, 0.0)


def default_cameras() -> dict[str, CameraConfig]:
    """클라이언트 쪽 카메라 설정 (호스트가 보내오는 프레임의 이름과 크기 선언).

    LeKiwiClient 는 카메라를 직접 열지 않는다. 여기 적힌 이름이 곧 정책에 들어가는
    `observation.images.<이름>` 키가 되므로, 호스트 및 학습 데이터셋과 이름이 같아야 한다
    (index_or_path 는 클라이언트에서 쓰이지 않는다).
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
class EvalArgs:
    """평가 세션 설정."""

    n_episodes: int = 10
    # 에피소드 한 번의 최대 길이(초). 이 시간이 지나면 판정을 물어본다.
    episode_time_s: float = 30
    # 에피소드 사이에 환경을 되돌릴 시간(초). 로봇은 그 자리에 멈춰 있는다.
    reset_time_s: float = 10
    # 리포트를 저장할 폴더. None 이면 outputs/eval 에 저장한다.
    output_dir: str | None = None
    # 리포트 파일 이름에 붙일 꼬리표 (체크포인트 비교용). 예: --eval.tag=step080k
    tag: str = ""
    # 판정 없이 끝난 에피소드를 리포트에서 제외할지 여부. False 면 실패로 센다.
    drop_unjudged: bool = False


@dataclass
class LeKiwiEvaluateConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    eval: EvalArgs = field(default_factory=EvalArgs)
    # `--policy.path=<허브 repo 또는 로컬 체크포인트>` 로 지정한다.
    policy: PreTrainedConfig | None = None

    # 정책에 넘길 태스크 문자열. 언어 조건부 정책에서는 반드시 필요하다.
    task: str = ""

    fps: int = 30
    # 추론 장치. None 이면 체크포인트 설정 → 자동 선택 순으로 정한다.
    device: str | None = None

    # 에피소드 시작 시 첫 목표 자세까지 부드럽게 이동하는 데 쓸 시간(초).
    warmup_s: float = 2.0
    # 베이스 속도(.vel)에 곱할 계수. 처음 돌려보는 정책은 0.3 정도로 낮춰서 시작할 것.
    base_speed_scale: float = 1.0

    # 로봇의 관측 키를 정책이 기대하는 키로 바꾼다.
    rename_map: dict[str, str] = field(default_factory=dict)

    display_data: bool = True
    display_mode: str = "rerun"
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    check_cameras: bool = True
    auto_camera_shape: bool = True
    print_status: bool = True

    def __post_init__(self):
        """`--policy.path` 로 지정한 체크포인트의 설정을 읽어 온다.

        draccus 는 `.path` 인자를 파싱 대상에서 빼 두므로 (`configs/parser.py` 의 wrap),
        여기서 직접 읽어 `PreTrainedConfig` 를 만들어야 한다.
        """
        policy_path = parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = parser.get_cli_overrides("policy")
            self.policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return ["policy"]

    def validate(self) -> None:
        errors = []
        if self.policy is None:
            errors.append(
                "--policy.path 가 필요합니다. 예: "
                "--policy.path=outputs/train/my_task/checkpoints/last/pretrained_model "
                "(허브 repo_id 도 됩니다)"
            )
        if self.fps <= 0:
            errors.append(f"--fps 는 1 이상이어야 합니다 (받은 값: {self.fps})")
        if self.eval.n_episodes <= 0:
            errors.append(f"--eval.n_episodes 는 1 이상이어야 합니다 (받은 값: {self.eval.n_episodes})")
        if self.eval.episode_time_s <= 0:
            errors.append(
                f"--eval.episode_time_s 는 0 보다 커야 합니다 (받은 값: {self.eval.episode_time_s})"
            )
        if self.base_speed_scale < 0:
            errors.append(f"--base_speed_scale 은 0 이상이어야 합니다 (받은 값: {self.base_speed_scale})")
        if errors:
            raise SystemExit("\n".join(f"error: {e}" for e in errors))

        if self.device is None or not is_torch_device_available(self.device):
            self.device = self.policy.device or auto_select_torch_device().type
            logging.info("추론 장치: %s", self.device)

    def report_path(self, started_at: datetime) -> Path:
        root = Path(self.eval.output_dir) if self.eval.output_dir else Path("outputs/eval")
        suffix = f"_{self.eval.tag}" if self.eval.tag else ""
        return root / f"lekiwi_eval_{started_at:%Y%m%d_%H%M%S}{suffix}.json"


def sync_cameras(robot: LeKiwiClient, auto_adapt: bool = True, timeout_s: float = 10.0) -> None:
    """호스트가 실제로 보내는 프레임과 --robot.cameras 설정을 맞춘다.

    정책에는 실제로 받은 프레임이 그대로 들어가므로 shape 이 어긋나도 추론 자체는 돌지만,
    그러면 정책이 학습 때와 다른 해상도를 보게 된다. auto_adapt=True 면 실제로 받은
    shape 에 맞춰 설정을 고쳐 주고 경고를 남긴다.
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
                "  - --auto_camera_shape=true 로 두면 받은 프레임에 맞춰 자동으로 보정합니다.\n"
                "  - 또는 라즈베리파이의 lekiwi_host 를 학습 때와 같은 해상도/rotation 으로 다시 띄우세요."
            )
        logging.warning(
            "카메라 '%s' 설정 %s → 호스트가 실제로 보낸 %s 로 자동 보정합니다. "
            "학습 때와 같은 해상도가 아니라면 정책 성능이 떨어집니다.",
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


def build_dataset_features(robot: LeKiwiClient, teleop_action_processor, robot_observation_processor) -> dict:
    """lekiwi-record.py 가 데이터셋을 만들 때 쓴 것과 똑같은 feature dict 을 만든다.

    이 dict 이 관측 → 정책 입력 텐서, 정책 출력 텐서 → action 이름 매핑의 기준이 된다.
    """
    return combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )


def check_policy_matches_robot(policy_cfg: PreTrainedConfig, ds_features: dict, rename_map: dict) -> None:
    """정책의 입출력이 LeKiwi 의 관측/액션과 맞는지 미리 확인한다."""
    action_names = ds_features[ACTION]["names"]
    action_ft = policy_cfg.output_features.get(ACTION) if policy_cfg.output_features else None
    if action_ft is not None and action_ft.shape[0] != len(action_names):
        raise SystemExit(
            f"error: 정책의 action 차원({action_ft.shape[0]}) 이 LeKiwi 의 action 차원"
            f"({len(action_names)}) 과 다릅니다.\n"
            f"  LeKiwi action: {action_names}\n"
            "  - 6차원이라면 팔만 학습한 정책입니다. LeKiwi 데이터셋(팔 6 + 베이스 3)으로 다시 학습하세요."
        )

    state_ft = policy_cfg.input_features.get(OBS_STATE) if policy_cfg.input_features else None
    state_names = ds_features[OBS_STATE]["names"]
    if state_ft is not None and state_ft.shape[0] != len(state_names):
        raise SystemExit(
            f"error: 정책의 observation.state 차원({state_ft.shape[0]}) 이 LeKiwi 의 state 차원"
            f"({len(state_names)}) 과 다릅니다.\n"
            f"  LeKiwi state: {state_names}"
        )

    if rename_map:
        return

    expected = {k for k in (policy_cfg.input_features or {}) if k.startswith("observation.images.")}
    provided = {k for k in ds_features if k.startswith("observation.images.")}
    if expected and not (expected <= provided or provided <= expected):
        raise SystemExit(
            "error: 정책이 기대하는 카메라와 로봇이 주는 카메라가 다릅니다.\n"
            f"  정책이 기대: {sorted(expected)}\n"
            f"  로봇이 제공: {sorted(provided)}\n"
            "  - 호스트/클라이언트의 --robot.cameras 이름을 학습 데이터셋과 맞추거나,\n"
            "  - --rename_map='{\"observation.images.front\": \"observation.images.cam_high\"}' 로 매핑하세요."
        )


class LeKiwiPolicy:
    """체크포인트를 올려 두고 관측 한 장 → action dict 한 개를 만들어 주는 얇은 래퍼.

    lerobot 의 `SyncInferenceEngine` 과 같은 일을 하지만, LeKiwi 의 9채널 action
    (팔 6 x .pos + 베이스 3 x .vel) 을 그대로 유지한다.
    """

    def __init__(self, cfg: LeKiwiEvaluateConfig, robot: LeKiwiClient, ds_features: dict):
        self.cfg = cfg
        self.ds_features = ds_features
        self.device = torch.device(cfg.device)
        self.robot_type = robot.name
        self.action_names = list(ds_features[ACTION]["names"])
        self.last_inference_s = 0.0

        policy_cfg = cfg.policy
        logging.info("정책 로딩 중: %s (type=%s)", policy_cfg.pretrained_path, policy_cfg.type)
        policy_class = get_policy_class(policy_cfg.type)
        self.policy = policy_class.from_pretrained(policy_cfg.pretrained_path, config=policy_cfg)
        self.policy = self.policy.to(self.device)
        self.policy.eval()

        # 정규화 통계는 체크포인트에 함께 저장돼 있으므로 pretrained_path 에서 그대로 불러온다.
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=policy_cfg.pretrained_path,
            preprocessor_overrides={
                "device_processor": {"device": cfg.device},
                "rename_observations_processor": {"rename_map": cfg.rename_map},
            },
        )
        logging.info("정책 준비 완료 (device=%s, action %d채널)", cfg.device, len(self.action_names))

    def reset(self) -> None:
        """정책과 전후처리 파이프라인의 내부 상태(액션 청크 큐 등)를 비운다.

        에피소드마다 반드시 호출한다. 안 하면 앞 에피소드에서 만들어 둔 청크가 흘러나온다.
        """
        self.policy.reset()
        self.preprocessor.reset()
        self.postprocessor.reset()

    def select_action(self, observation: dict) -> dict[str, float]:
        """관측 한 장으로 추론해 LeKiwi action dict 을 만든다."""
        start = time.perf_counter()
        with torch.inference_mode():
            frame = build_inference_frame(
                observation, self.device, self.ds_features, self.cfg.task, self.robot_type
            )
            frame = self.preprocessor(frame)
            action_tensor = self.policy.select_action(frame)
            action_tensor = self.postprocessor(action_tensor)
        self.last_inference_s = time.perf_counter() - start

        action = make_robot_action(action_tensor, self.ds_features)
        action = {**STOP_BASE, **action}
        if self.cfg.base_speed_scale != 1.0:
            for key in BASE_VEL_KEYS:
                action[key] *= self.cfg.base_speed_scale
        return action


def make_control_listener():
    """평가용 키를 받는 리스너를 띄운다.

    반환: (listener, state)
        state["quit"]        - ESC. 평가 전체 중단
        state["end_episode"] - S/F/→. 현재 구간 종료
        state["verdict"]     - "success" | "failure" | None
        state["paused"]      - SPACE 토글
        state["resumed"]     - 재개 직후 한 번만 True (정책 상태를 비우는 신호)
    """
    state = {"quit": False, "end_episode": False, "verdict": None, "paused": False, "resumed": False}

    def dispatch(name: str) -> None:
        key = name.lower()
        if key == "esc":
            print("\nESC, 평가를 중단합니다...")
            state["quit"] = True
            state["end_episode"] = True
        elif key == "s":
            state["verdict"] = "success"
            state["end_episode"] = True
        elif key == "f":
            state["verdict"] = "failure"
            state["end_episode"] = True
        elif key in ("right", "n"):
            state["end_episode"] = True
        elif key == "space":
            state["paused"] = not state["paused"]
            if state["paused"]:
                print("\n[일시정지] SPACE 로 재개")
            else:
                print("\n[재개]")
                state["resumed"] = True

    listener = TerminalKeyListener(dispatch)
    listener.start()
    return listener, state


def hold_still(robot: LeKiwiClient, observation: dict) -> dict[str, float]:
    """현재 팔 자세를 그대로 목표로 주고 베이스를 세우는 action 을 만들어 보낸다."""
    action = {k: float(v) for k, v in observation.items() if k.endswith(".pos")}
    action = {**action, **STOP_BASE}
    robot.send_action(action)
    return action


def move_to_start(
    robot: LeKiwiClient, target: dict[str, float], warmup_s: float, fps: int, state: dict
) -> None:
    """현재 팔 자세에서 첫 목표 자세까지 선형 보간으로 천천히 이동한다 (베이스는 정지)."""
    if warmup_s <= 0:
        return

    obs = robot.get_observation()
    start = {k: float(obs[k]) for k in target if k.endswith(".pos") and k in obs}
    if not start:
        logging.warning("현재 팔 자세를 읽지 못해 워밍업을 건너뜁니다.")
        return

    steps = max(int(warmup_s * fps), 1)
    for step in range(1, steps + 1):
        if state["quit"]:
            return
        loop_start = time.perf_counter()
        alpha = step / steps
        action = {k: (1.0 - alpha) * v + alpha * float(target[k]) for k, v in start.items()}
        robot.send_action({**action, **STOP_BASE})
        precise_sleep(max(1 / fps - (time.perf_counter() - loop_start), 0.0))


def run_episode(
    robot: LeKiwiClient, policy: LeKiwiPolicy, state: dict, cfg: LeKiwiEvaluateConfig, index: int
) -> dict:
    """한 에피소드를 끝까지 돌리고 측정치를 돌려준다 (성공/실패 판정은 호출자가 붙인다)."""
    control_interval = 1.0 / cfg.fps
    inference_ms: list[float] = []
    loop_hz: list[float] = []
    steps = 0
    start = time.perf_counter()

    while not state["end_episode"]:
        loop_start = time.perf_counter()

        observation = robot.get_observation()

        if state["paused"]:
            action = hold_still(robot, observation)
        else:
            if state["resumed"]:
                # 정지 중에 쌓인 액션 청크를 버리고 지금 관측부터 다시 시작한다.
                policy.reset()
                state["resumed"] = False
            action = policy.select_action(observation)
            robot.send_action(action)
            inference_ms.append(policy.last_inference_s * 1000)
            steps += 1

        if cfg.display_data:
            log_visualization_data(
                cfg.display_mode,
                observation=observation,
                action=action,
                compress_images=cfg.display_compressed_images,
            )

        dt_s = time.perf_counter() - loop_start
        precise_sleep(max(control_interval - dt_s, 0.0))

        loop_s = time.perf_counter() - loop_start
        if not state["paused"]:
            loop_hz.append(1 / loop_s)

        if cfg.print_status:
            elapsed = time.perf_counter() - start
            tag = "PAUSED" if state["paused"] else "  RUN "
            print(
                f"\r[ep {index + 1}/{cfg.eval.n_episodes}][{tag}] {elapsed:5.1f}/{cfg.eval.episode_time_s:.0f}s "
                f"x={action['x.vel']:+.2f} y={action['y.vel']:+.2f} theta={action['theta.vel']:+6.1f} "
                f"| 추론 {policy.last_inference_s * 1000:5.1f} ms | {1 / loop_s:5.1f} Hz   ",
                end="",
                flush=True,
            )

        # 일시정지 중에는 시간이 흐르지 않게 해서, 손댄 시간이 에피소드 길이를 깎지 않도록 한다.
        if state["paused"]:
            start += loop_s
        elif time.perf_counter() - start >= cfg.eval.episode_time_s:
            break

    print()
    return {
        "index": index,
        "steps": steps,
        "duration_s": round(time.perf_counter() - start, 2),
        "inference_ms_mean": round(float(np.mean(inference_ms)), 2) if inference_ms else None,
        "inference_ms_p95": round(float(np.percentile(inference_ms, 95)), 2) if inference_ms else None,
        "loop_hz_mean": round(float(np.mean(loop_hz)), 2) if loop_hz else None,
    }


def wait_for_verdict(robot: LeKiwiClient, state: dict, cfg: LeKiwiEvaluateConfig, index: int) -> str | None:
    """S/F 를 누를 때까지 로봇을 그 자리에 세워 둔 채 기다린다.

    반환: "success" | "failure" | None (ESC 로 중단한 경우)
    """
    if state["verdict"] is not None or state["quit"]:
        return state["verdict"]

    log_say("Success or failure", cfg.play_sounds)
    print(f"에피소드 {index + 1} 판정: S(성공) / F(실패) / ESC(중단)", flush=True)
    while state["verdict"] is None and not state["quit"]:
        loop_start = time.perf_counter()
        try:
            hold_still(robot, robot.get_observation())
        except Exception as e:
            logging.warning("정지 자세 유지 실패: %s", e)
        precise_sleep(max(0.1 - (time.perf_counter() - loop_start), 0.0))
    return state["verdict"]


def reset_phase(robot: LeKiwiClient, state: dict, cfg: LeKiwiEvaluateConfig) -> None:
    """다음 에피소드를 위해 환경을 되돌릴 시간을 준다. 로봇은 그 자리에 멈춰 있는다."""
    if cfg.eval.reset_time_s <= 0 or state["quit"]:
        return

    log_say("Reset the environment", cfg.play_sounds)
    state["end_episode"] = False
    start = time.perf_counter()
    while not state["end_episode"] and not state["quit"]:
        loop_start = time.perf_counter()
        try:
            hold_still(robot, robot.get_observation())
        except Exception as e:
            logging.warning("정지 자세 유지 실패: %s", e)
        remaining = cfg.eval.reset_time_s - (time.perf_counter() - start)
        if remaining <= 0:
            break
        if cfg.print_status:
            print(f"\r[리셋] {remaining:4.1f}s 남음 (→ 로 건너뛰기)   ", end="", flush=True)
        precise_sleep(max(0.1 - (time.perf_counter() - loop_start), 0.0))
    print()


def summarize(episodes: list[dict], cfg: LeKiwiEvaluateConfig) -> dict:
    """에피소드 기록에서 성공률과 지연 통계를 뽑는다."""
    counted = [ep for ep in episodes if not (cfg.eval.drop_unjudged and ep["verdict"] is None)]
    successes = sum(1 for ep in counted if ep["verdict"] == "success")
    inference = [ep["inference_ms_mean"] for ep in episodes if ep["inference_ms_mean"] is not None]
    hz = [ep["loop_hz_mean"] for ep in episodes if ep["loop_hz_mean"] is not None]
    return {
        "n_episodes_run": len(episodes),
        "n_episodes_counted": len(counted),
        "n_unjudged": sum(1 for ep in episodes if ep["verdict"] is None),
        "n_success": successes,
        "success_rate": round(successes / len(counted), 4) if counted else None,
        "inference_ms_mean": round(float(np.mean(inference)), 2) if inference else None,
        "loop_hz_mean": round(float(np.mean(hz)), 2) if hz else None,
    }


def _display_width(text: str) -> int:
    """한글/한자처럼 두 칸을 차지하는 글자를 감안한 표시 폭. 표를 맞추는 데만 쓴다."""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def _pad(text: str, width: int, align: str = ">") -> str:
    """f-string 의 `:<8` 은 글자 수로 세기 때문에 한글이 섞이면 표가 어긋난다. 표시 폭으로 채운다."""
    fill = " " * max(width - _display_width(text), 0)
    return text + fill if align == "<" else fill + text


REPORT_COLUMNS = (("ep", 4), ("판정", 10), ("길이(s)", 10), ("스텝", 8), ("추론(ms)", 11), ("루프(Hz)", 11))
REPORT_WIDTH = sum(width for _, width in REPORT_COLUMNS)
VERDICT_LABELS = {"success": "성공", "failure": "실패", None: "판정없음"}


def print_report(episodes: list[dict], summary: dict) -> None:
    print("\n" + "=" * REPORT_WIDTH)
    print("".join(_pad(name, width) for name, width in REPORT_COLUMNS))
    print("-" * REPORT_WIDTH)
    for ep in episodes:
        cells = (
            str(ep["index"] + 1),
            VERDICT_LABELS[ep["verdict"]],
            f"{ep['duration_s']:.1f}",
            str(ep["steps"]),
            f"{ep['inference_ms_mean']:.1f}" if ep["inference_ms_mean"] is not None else "-",
            f"{ep['loop_hz_mean']:.1f}" if ep["loop_hz_mean"] is not None else "-",
        )
        print("".join(_pad(cell, width) for cell, (_, width) in zip(cells, REPORT_COLUMNS, strict=True)))
    print("-" * REPORT_WIDTH)
    rate = summary["success_rate"]
    if rate is None:
        print("성공률: 셀 수 있는 에피소드가 없습니다")
    else:
        print(
            f"성공률: {summary['n_success']}/{summary['n_episodes_counted']} = {rate * 100:.1f}%"
        )
    if summary["n_unjudged"]:
        print(f"판정 없이 끝난 에피소드: {summary['n_unjudged']}개")
    inference = summary["inference_ms_mean"]
    hz = summary["loop_hz_mean"]
    if inference is not None:
        print(f"평균 추론 {inference} ms | 평균 루프 {hz} Hz")
    print("=" * REPORT_WIDTH + "\n")


@parser.wrap()
def evaluate(cfg: LeKiwiEvaluateConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    teleop_action_processor, _, robot_observation_processor = make_default_processors()

    started_at = datetime.now()
    listener = None
    episodes: list[dict] = []

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

        if cfg.check_cameras:
            sync_cameras(robot, auto_adapt=cfg.auto_camera_shape)

        # 정책 feature 는 카메라 shape 이 확정된 뒤에 만든다.
        ds_features = build_dataset_features(robot, teleop_action_processor, robot_observation_processor)
        logging.info("정책 입출력 feature: %s", list(ds_features))
        check_policy_matches_robot(cfg.policy, ds_features, cfg.rename_map)

        policy = LeKiwiPolicy(cfg, robot, ds_features)

        if cfg.display_data:
            init_visualization(
                cfg.display_mode, session_name="lekiwi_evaluate", ip=cfg.display_ip, port=cfg.display_port
            )

        listener, state = make_control_listener()
        print(
            "\n조작키 | 진행 중  : S 성공 종료, F 실패 종료, →(N) 판정 보류 종료, SPACE 일시정지, ESC 중단"
            "\n       | 판정 대기: S 성공, F 실패, ESC 중단"
            "\n       | 리셋 구간: →(N) 건너뛰기, ESC 중단"
            "\n       | (이 터미널 창이 포커스를 갖고 있어야 키가 들어옵니다)\n"
        )

        for index in range(cfg.eval.n_episodes):
            if state["quit"]:
                break

            state["end_episode"] = False
            state["verdict"] = None
            state["paused"] = False
            state["resumed"] = False

            log_say(f"Episode {index + 1}", cfg.play_sounds)
            policy.reset()
            # 첫 추론 결과로 팔이 튀지 않도록, 한 번 추론해서 목표를 보고 거기까지 천천히 간다.
            first_action = policy.select_action(robot.get_observation())
            move_to_start(robot, first_action, cfg.warmup_s, cfg.fps, state)
            policy.reset()
            if state["quit"]:
                break

            metrics = run_episode(robot, policy, state, cfg, index)
            metrics["verdict"] = wait_for_verdict(robot, state, cfg, index)
            episodes.append(metrics)
            logging.info(
                "에피소드 %d: %s (%.1f초, %d스텝)",
                index + 1,
                metrics["verdict"] or "판정없음",
                metrics["duration_s"],
                metrics["steps"],
            )

            if index < cfg.eval.n_episodes - 1:
                reset_phase(robot, state, cfg)
    except KeyboardInterrupt:
        print("\nCtrl+C, 중단합니다...")
    finally:
        log_say("Evaluation finished", cfg.play_sounds, blocking=True)

        # 베이스를 정지시키고, 팔은 현재 자세를 그대로 목표로 줘서 튀지 않게 한 뒤 연결을 끊는다.
        if robot.is_connected:
            try:
                hold_still(robot, robot.get_observation())
                time.sleep(0.2)
            except Exception as e:
                logging.warning("정지 명령 전송 실패: %s", e)
            robot.disconnect()
        if listener is not None:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)

        if episodes:
            summary = summarize(episodes, cfg)
            print_report(episodes, summary)
            report = {
                "policy_path": str(cfg.policy.pretrained_path) if cfg.policy else None,
                "policy_type": cfg.policy.type if cfg.policy else None,
                "task": cfg.task,
                "tag": cfg.eval.tag,
                "robot_id": cfg.robot.id,
                "remote_ip": cfg.robot.remote_ip,
                "fps": cfg.fps,
                "device": cfg.device,
                "base_speed_scale": cfg.base_speed_scale,
                "eval": asdict(cfg.eval),
                "started_at": started_at.isoformat(timespec="seconds"),
                "finished_at": datetime.now().isoformat(timespec="seconds"),
                "summary": summary,
                "episodes": episodes,
            }
            path = cfg.report_path(started_at)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
            logging.info("리포트 저장: %s", path)
        else:
            logging.warning("완료된 에피소드가 없어 리포트를 저장하지 않습니다.")


if __name__ == "__main__":
    evaluate()
