r"""LeKiwi 무제한 추론 (학습된 policy 를 사람이 멈출 때까지 계속 돌린다).

`lekiwi-rollout.py` 와 같은 추론 루프지만, **끝나는 시점을 시간이 아니라 사람이 정한다**.
시연이나 수업처럼 "될 때까지 계속 돌려 두는" 용도다.

    rollout    - `--duration_s` 로 정해진 시간만 돌린다 (실험/디버깅용).
    inference  - ESC 를 누를 때까지 무제한. 끝나면 시작 자세로 복귀한다.
    evaluate   - N 에피소드를 돌리고 성공률 리포트를 남긴다.

설계는 physical-labs 앱의 추론 워커(`services/inference_worker.py`)를 따랐다.
특히 종료 후 시작 자세 복귀는 그쪽 `_roll_back_to_start` 와 같은 규칙이다.

먼저 라즈베리파이(LeKiwi) 쪽에서 호스트를 띄워 둘 것:

    python -m lerobot.robots.lekiwi.lekiwi_host \
        --robot.id=lekiwi01 \
        --robot.cameras='{
          front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
          wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
        }' \
        --host.connection_time_s=14400

그 다음 PC 에서:

    python lekiwi-inference.py \
        --policy.path=outputs/train/my_task/checkpoints/last/pretrained_model \
        --task="Pick up the cube and place it in the box" \
        --base_speed_scale=0.3 \
        --display_data=true

카메라 이름이 학습 데이터셋과 다르면 --rename_map 으로 맞춘다 (SmolVLA 베이스 모델은
camera1/camera2/camera3 을 기대한다):

    --rename_map='{"observation.images.front": "observation.images.camera1",
                   "observation.images.wrist": "observation.images.camera2"}'

전체 옵션은 `python lekiwi-inference.py --help`.

조작키
    SPACE : 일시정지 / 재개 (정지 중에는 팔을 현재 자세로 고정하고 베이스를 멈춘다)
    ESC   : 종료. 베이스를 세우고 시작 자세로 되돌아간 뒤 연결을 끊는다.
            복귀 중에 ESC 를 한 번 더 누르면 복귀를 즉시 중단한다.
    Ctrl+C: 비상 정지. 복귀하지 않고 그 자리에 멈춘 채 끝낸다.

시작/종료 자세
    - 연결 직후(아무것도 움직이기 전) 팔 자세를 기억해 두고, ESC 로 끝낼 때 그 자세로
      되돌아간다. 복귀 시간은 이동량에 맞춰 정해진다
      (`--return_home_deg_per_s` 기본 45도/초, 0.5~5초 사이로 제한).
    - 복귀도 사람이 루프 밖에 있는 자율 동작이다. 그래서 정책 추종보다 느리게 가고,
      매 스텝 ESC 를 확인하며, 최대 시간에 상한을 둔다.
    - **오류로 끝난 경우와 Ctrl+C 는 복귀하지 않는다** — 로봇이 어떤 상태인지 모르거나,
      사람이 위험을 느껴 멈춘 것일 수 있기 때문이다.
    - 복귀 자체를 끄려면 `--return_home=false`.

안전 주의
    - 정책은 학습 때 본 적 없는 상황에서 무엇이든 할 수 있다. 주변을 비우고, 손은
      SPACE/ESC 위에 두고, 처음에는 `--base_speed_scale` 을 낮춰서 시작할 것.
    - 팔만 쓰고 베이스를 아예 굴리지 않으려면 `--base_speed_scale=0`.
    - 시작 시 팔이 첫 추론 결과로 튀는 것을 막기 위해 `--warmup_s` 초 동안 현재 자세에서
      첫 목표 자세까지 선형 보간으로 이동한다 (이 구간에서 베이스는 정지).
"""

import logging
import time
from dataclasses import dataclass, field
from pprint import pformat

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

# 시작 자세 복귀에 쓰는 시간의 하한/상한(초). 상한은 "정지를 눌렀는데 안 멈춘다" 를 막는 안전장치다.
RETURN_MIN_S = 0.5
RETURN_MAX_S = 5.0
# 이보다 적게 벗어나 있으면 이미 시작 자세로 보고 움직이지 않는다 (도).
RETURN_SKIP_DEG = 0.5


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
class LeKiwiInferenceConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    # `--policy.path=<허브 repo 또는 로컬 체크포인트>` 로 지정한다. `--policy.<필드>=값` 으로
    # 체크포인트의 설정을 덮어쓸 수도 있다 (예: --policy.n_action_steps=50).
    policy: PreTrainedConfig | None = None

    # 정책에 넘길 태스크 문자열. 언어 조건부 정책(SmolVLA, pi0 등)에서는 반드시 필요하고,
    # ACT/Diffusion 처럼 태스크를 안 보는 정책에서는 무시된다.
    task: str = ""

    fps: int = 30
    # 추론 장치. None 이면 체크포인트 설정 → 자동 선택 순으로 정한다.
    device: str | None = None

    # 첫 목표 자세까지 부드럽게 이동하는 데 쓸 시간(초). 0 이면 곧바로 첫 추론 결과를 보낸다.
    warmup_s: float = 2.0
    # ESC 로 끝낼 때 시작 자세로 되돌아갈지 여부.
    return_home: bool = True
    # 복귀 속도(도/초). 사람이 보고 반응할 수 있어야 하므로 정책 추종보다 느리게 잡는다.
    # 실제 복귀 시간은 이동량 / 이 값이며 RETURN_MIN_S ~ RETURN_MAX_S 로 제한된다.
    return_home_deg_per_s: float = 45.0
    # 베이스 속도(.vel)에 곱할 계수. 처음 돌려보는 정책은 0.3 정도로 낮춰서 시작할 것.
    base_speed_scale: float = 1.0

    # 데이터셋/로봇의 관측 키를 정책이 기대하는 키로 바꾼다.
    # 예: --rename_map='{"observation.images.front": "observation.images.cam_high"}'
    rename_map: dict[str, str] = field(default_factory=dict)

    display_data: bool = True
    display_mode: str = "rerun"
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    # 첫 관측을 받아 카메라 이름/해상도가 호스트와 맞는지 검사
    check_cameras: bool = True
    # 호스트가 보내는 프레임 shape 이 --robot.cameras 와 다르면 자동으로 설정을 맞춘다.
    auto_camera_shape: bool = True
    # 한 줄짜리 상태 표시 (베이스 속도, 추론 지연, 루프 주파수)
    print_status: bool = True

    def __post_init__(self):
        """`--policy.path` 로 지정한 체크포인트의 설정을 읽어 온다.

        draccus 는 `.path` 인자를 파싱 대상에서 빼 두므로 (`configs/parser.py` 의 wrap),
        여기서 직접 읽어 `PreTrainedConfig` 를 만들어야 한다. `--policy.<필드>=값` 으로 준
        나머지 인자는 체크포인트 설정을 덮어쓰는 override 로 넘어간다.
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
        if self.base_speed_scale < 0:
            errors.append(f"--base_speed_scale 은 0 이상이어야 합니다 (받은 값: {self.base_speed_scale})")
        if self.return_home_deg_per_s <= 0:
            errors.append(
                f"--return_home_deg_per_s 는 0 보다 커야 합니다 (받은 값: {self.return_home_deg_per_s})"
            )
        if errors:
            raise SystemExit("\n".join(f"error: {e}" for e in errors))

        # 장치 결정: --device → 체크포인트 설정 → 자동 선택
        if self.device is None or not is_torch_device_available(self.device):
            self.device = self.policy.device or auto_select_torch_device().type
            logging.info("추론 장치: %s", self.device)


def sync_cameras(robot: LeKiwiClient, auto_adapt: bool = True, timeout_s: float = 10.0) -> None:
    """호스트가 실제로 보내는 프레임과 --robot.cameras 설정을 맞춘다.

    LeKiwiClient 는 카메라를 직접 열지 않으므로, 설정된 width/height 는 "호스트가 이런
    모양으로 보내온다"는 선언일 뿐이다. 정책에는 실제로 받은 프레임이 그대로 들어가므로
    shape 이 어긋나도 추론 자체는 돌지만, 그러면 정책이 학습 때와 다른 해상도를 보게 된다.
    auto_adapt=True 면 실제로 받은 shape 에 맞춰 설정을 고쳐 주고 경고를 남긴다.
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
    녹화 때와 동일한 경로로 만들어야 채널 순서가 어긋나지 않는다.
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
    """정책의 입출력이 LeKiwi 의 관측/액션과 맞는지 미리 확인한다.

    가장 흔한 사고가 "팔만 있는 정책(6채널)을 LeKiwi(9채널)에 올리는 것"이라, 로봇을
    움직이기 전에 여기서 막는다.
    """
    action_names = ds_features[ACTION]["names"]
    action_ft = policy_cfg.output_features.get(ACTION) if policy_cfg.output_features else None
    if action_ft is not None and action_ft.shape[0] != len(action_names):
        raise SystemExit(
            f"error: 정책의 action 차원({action_ft.shape[0]}) 이 LeKiwi 의 action 차원"
            f"({len(action_names)}) 과 다릅니다.\n"
            f"  LeKiwi action: {action_names}\n"
            "  - 6차원이라면 팔만 학습한 정책입니다. LeKiwi 데이터셋(팔 6 + 베이스 3)으로 다시 학습하세요.\n"
            "  - lerobot-rollout 로 만든 체크포인트라면 베이스 채널이 빠졌을 수 있습니다 "
            "(rollout/context.py:284 가 .vel 을 버립니다)."
        )

    # state 차원은 경고만 한다. 사전학습 체크포인트에서 이어 학습한 정책(SmolVLA 등)은
    # config 의 input_features 가 베이스 모델 값 그대로 남아 있는 경우가 있는데
    # (lerobot 의 factory.py 는 input_features 가 비어 있을 때만 데이터셋 값으로 덮어쓴다),
    # 실제로는 state 를 내부에서 패딩해 쓰므로 차원이 달라도 정상 동작한다.
    # 반면 action 차원은 그대로 로봇에 나가므로 위에서 에러로 막는다.
    state_ft = policy_cfg.input_features.get(OBS_STATE) if policy_cfg.input_features else None
    state_names = ds_features[OBS_STATE]["names"]
    if state_ft is not None and state_ft.shape[0] != len(state_names):
        logging.warning(
            "정책 config 의 observation.state 차원(%d) 이 LeKiwi 의 state 차원(%d) 과 다릅니다. "
            "사전학습 체크포인트의 값이 남아 있는 경우라면 정상입니다 (정책이 내부에서 패딩). "
            "LeKiwi state: %s",
            state_ft.shape[0],
            len(state_names),
            state_names,
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

    def __init__(self, cfg: LeKiwiInferenceConfig, robot: LeKiwiClient, ds_features: dict):
        self.cfg = cfg
        self.ds_features = ds_features
        self.device = torch.device(cfg.device)
        self.robot_type = robot.name
        self.action_names = list(ds_features[ACTION]["names"])
        # 마지막 추론에 걸린 시간(초). 상태 표시와 evaluate 리포트에 쓴다.
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

        일시정지 후 재개하거나 새 에피소드를 시작할 때 호출한다. 안 하면 정지 전에
        만들어 둔 청크가 그대로 흘러나온다.
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
        # 베이스 채널이 없는 정책이어도 호스트가 KeyError 를 내지 않도록 정지값을 깔아 둔다.
        action = {**STOP_BASE, **action}
        if self.cfg.base_speed_scale != 1.0:
            for key in BASE_VEL_KEYS:
                action[key] *= self.cfg.base_speed_scale
        return action


def make_control_listener():
    """SPACE(일시정지/재개), ESC(종료/복귀 중단) 를 받는 리스너를 띄운다.

    반환: (listener, state)
        state["quit"]         - ESC 한 번. 추론 루프 종료
        state["abort_return"] - 종료 후 복귀 중에 누른 ESC. 복귀 즉시 중단
        state["paused"]       - SPACE 토글
        state["resumed"]      - 재개 직후 한 번만 True (정책 상태를 비우는 신호)

    무제한 추론에는 연속 입력(주행키)이 필요 없고 discrete 한 제어키만 있으면 되므로,
    pynput 없이도 되는 TerminalKeyListener 하나만 쓴다 (이 터미널 창이 포커스 필요).
    """
    state = {"quit": False, "abort_return": False, "paused": False, "resumed": False}

    def dispatch(name: str) -> None:
        key = name.lower()
        if key == "esc":
            if state["quit"]:
                # 이미 종료 중이다 = 지금은 복귀 구간. 한 번 더 누르면 복귀를 멈춘다.
                state["abort_return"] = True
                print("\n복귀를 중단합니다...")
            else:
                print("\nESC, 종료합니다...")
                state["quit"] = True
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
    logging.info("첫 목표 자세로 %.1f 초 동안 이동합니다...", warmup_s)
    for step in range(1, steps + 1):
        if state["quit"]:
            return
        loop_start = time.perf_counter()
        alpha = step / steps
        action = {k: (1.0 - alpha) * v + alpha * float(target[k]) for k, v in start.items()}
        robot.send_action({**action, **STOP_BASE})
        precise_sleep(max(1 / fps - (time.perf_counter() - loop_start), 0.0))


def read_arm_pose(robot: LeKiwiClient) -> dict[str, float]:
    """현재 팔 관절 자세만 뽑아온다 (베이스 속도는 제외)."""
    return {k: float(v) for k, v in robot.get_observation().items() if k.endswith(".pos")}


def return_home(
    robot: LeKiwiClient, home: dict[str, float], cfg: LeKiwiInferenceConfig, state: dict | None
) -> None:
    """시작할 때 기억해 둔 자세로 천천히 되돌아간다 (베이스는 정지).

    복귀도 사람이 루프 밖에 있는 자율 동작이라, physical-labs 앱의 `_roll_back_to_start`
    와 같은 안전 규칙을 따른다:

      - 이동량에 맞춰 시간을 정하되(`--return_home_deg_per_s`), 정책 추종보다 느리게 간다.
      - `RETURN_MAX_S` 상한을 둔다 ("종료를 눌렀는데 안 멈춘다" 방지).
      - 매 스텝 ESC(state["abort_return"]) 를 확인해 즉시 멈출 수 있게 한다.
      - 이미 시작 자세 근처면(`RETURN_SKIP_DEG`) 아무것도 하지 않는다.

    종료 경로에서 불리므로 state["quit"] 는 보지 않는다 (ESC 로 끝낸 경우가 곧 복귀 대상이다).
    """
    if not cfg.return_home or not home:
        return

    current = {k: v for k, v in read_arm_pose(robot).items() if k in home}
    if not current:
        logging.warning("현재 팔 자세를 읽지 못해 시작 자세 복귀를 건너뜁니다.")
        return

    max_delta = max(abs(home[k] - v) for k, v in current.items())
    if max_delta < RETURN_SKIP_DEG:
        logging.info("이미 시작 자세입니다 (최대 편차 %.2f도). 복귀를 건너뜁니다.", max_delta)
        return

    duration_s = min(RETURN_MAX_S, max(RETURN_MIN_S, max_delta / cfg.return_home_deg_per_s))
    steps = max(int(duration_s * cfg.fps), 1)
    logging.info(
        "시작 자세로 되돌아갑니다 (최대 편차 %.1f도, %.1f초). 중단하려면 ESC.", max_delta, duration_s
    )
    for step in range(1, steps + 1):
        if state is not None and state["abort_return"]:
            logging.info("복귀 중단 (%d/%d 스텝).", step, steps)
            return
        loop_start = time.perf_counter()
        alpha = step / steps
        action = {k: (1.0 - alpha) * v + alpha * float(home[k]) for k, v in current.items()}
        robot.send_action({**action, **STOP_BASE})
        precise_sleep(max(1 / cfg.fps - (time.perf_counter() - loop_start), 0.0))
    logging.info("시작 자세 복귀 완료.")


def inference_loop(
    robot: LeKiwiClient, policy: LeKiwiPolicy, state: dict, cfg: LeKiwiInferenceConfig
) -> None:
    """ESC 를 누를 때까지 계속 추론한다. 끝나는 조건은 시간이 아니라 사람이다."""
    control_interval = 1.0 / cfg.fps
    start = time.perf_counter()
    steps = 0

    while not state["quit"]:
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

        if cfg.print_status:
            loop_s = time.perf_counter() - loop_start
            elapsed = time.perf_counter() - start
            tag = "PAUSED" if state["paused"] else "  RUN "
            print(
                f"\r[{tag}] {int(elapsed) // 60:02d}:{int(elapsed) % 60:02d} "
                f"x={action['x.vel']:+.2f} y={action['y.vel']:+.2f} "
                f"theta={action['theta.vel']:+6.1f} | 추론 {policy.last_inference_s * 1000:5.1f} ms "
                f"| {1 / loop_s:5.1f} Hz   ",
                end="",
                flush=True,
            )

    elapsed = time.perf_counter() - start
    print()
    logging.info("정책이 %d 스텝, %.1f 초 동안 실행되었습니다.", steps, elapsed)


@parser.wrap()
def inference(cfg: LeKiwiInferenceConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    teleop_action_processor, _, robot_observation_processor = make_default_processors()

    listener = None
    state = None
    # 아무것도 움직이기 전의 팔 자세. ESC 로 끝낼 때 여기로 되돌아간다.
    home_pose: dict[str, float] = {}
    # 정상 종료(ESC)로 빠져나왔는지. 오류/Ctrl+C 경로에서는 복귀하지 않는다.
    clean_stop = False

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

        # 워밍업으로 팔을 움직이기 전에 지금 자세를 기억해 둔다 (종료 시 복귀 목표).
        home_pose = read_arm_pose(robot)
        logging.info("시작 자세를 기억했습니다: %s", {k: round(v, 1) for k, v in home_pose.items()})

        if cfg.check_cameras:
            sync_cameras(robot, auto_adapt=cfg.auto_camera_shape)

        # 정책 feature 는 카메라 shape 이 확정된 뒤에 만든다.
        ds_features = build_dataset_features(robot, teleop_action_processor, robot_observation_processor)
        logging.info("정책 입출력 feature: %s", list(ds_features))
        check_policy_matches_robot(cfg.policy, ds_features, cfg.rename_map)

        policy = LeKiwiPolicy(cfg, robot, ds_features)

        if cfg.display_data:
            init_visualization(
                cfg.display_mode, session_name="lekiwi_inference", ip=cfg.display_ip, port=cfg.display_port
            )

        listener, state = make_control_listener()
        print(
            "\n조작키 | SPACE 일시정지/재개, ESC 종료(→ 시작 자세로 복귀)"
            "\n       | 복귀 중 ESC 한 번 더 = 복귀 중단, Ctrl+C = 비상 정지(복귀 없음)"
            "\n       | (이 터미널 창이 포커스를 갖고 있어야 키가 들어옵니다)\n"
        )

        # 첫 추론 결과로 팔이 튀지 않도록, 한 번 추론해서 목표를 보고 거기까지 천천히 간다.
        first_action = policy.select_action(robot.get_observation())
        move_to_start(robot, first_action, cfg.warmup_s, cfg.fps, state)
        policy.reset()

        if not state["quit"]:
            log_say("Running policy", cfg.play_sounds, blocking=True)
            inference_loop(robot, policy, state, cfg)
        # 여기까지 왔으면 ESC 로 끝난 정상 경로다 (오류는 예외로 빠진다).
        clean_stop = True
    except KeyboardInterrupt:
        print("\nCtrl+C, 비상 정지합니다 (시작 자세 복귀 없음)...")
    finally:
        # 먼저 베이스를 세우고(즉시 안전 확보), 팔을 시작 자세로 되돌린 뒤 연결을 끊는다.
        if robot.is_connected:
            try:
                hold_still(robot, robot.get_observation())
                time.sleep(0.2)
            except Exception as e:
                logging.warning("정지 명령 전송 실패: %s", e)
            if clean_stop:
                try:
                    return_home(robot, home_pose, cfg, state)
                    # 복귀 직후 팔이 튀지 않게 마지막 자세를 그대로 목표로 한 번 더 보낸다.
                    hold_still(robot, robot.get_observation())
                    time.sleep(0.2)
                except KeyboardInterrupt:
                    print("\n시작 자세 복귀를 중단했습니다.")
                except Exception as e:
                    logging.warning("시작 자세 복귀 실패: %s", e)
            robot.disconnect()
        if listener is not None:
            listener.stop()
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
        print()


if __name__ == "__main__":
    inference()
