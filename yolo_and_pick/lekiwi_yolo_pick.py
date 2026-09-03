#!/usr/bin/env python
r"""YOLO 로 잡은 큐브 쪽으로 LeKiwi 를 몰아가서, 큐브가 화면 가운데 세로선에 오고 정해진 크기로 보일 때 멈춘다.

동작
    1. front 카메라에서 YOLO 추론. 박스 중심이 화면 **가운데 가로선 아래**에 있는 검출만 후보로 삼고
       그중 **가장 큰** 박스를 목표로 한다. 검출이 없거나 전부 가로선 위(멀리 있는 것)라면 무시하고
       움직이지 않는다 (상태 SEARCHING / ABOVE_LINE). 가로선 위치는 --approach.line_ratio (기본 0.5),
       이 필터를 끄려면 --approach.only_below_line=false.
    2. 전진(x.vel): 박스 **폭**이 --approach.target_size_px (기본 117px) 가 될 때까지 앞으로 간다.
           박스가 목표보다 작다 → 멀다 → 전진
           박스 폭 >= 목표 - tolerance     → 정지
           박스 폭 >  목표 + tolerance     → 너무 가까움 → 후진 (--approach.allow_backward, 기본 true;
                                               false 면 그냥 정지)
       크기는 폭을 쓴다: 가까워질수록 큐브 아랫부분이 화면 아래로 잘려 높이는 믿을 수 없다.
    2-1. 너무 가까워서 잘린 경우 (TOO_CLOSE, 최우선): 박스가 화면 **바닥에 붙어 있고** 다음 중 하나면
       큐브가 카메라 아래로 잘려 나간 것이므로 **후진**한다.
           (a) 박스가 납작하다: 높이/폭 < --approach.clip_aspect_min (기본 0.8).
               큐브는 온전히 보이면 대략 정사각형(폭 117 × 높이 128 ≈ 1.1)인데, 아주 가까우면
               윗면 한 줌만 보여서 폭은 넓고 높이는 얇은 띠가 된다 (예: 188 × 30).
           (b) 크기가 목표 - tolerance 보다 작다 (멀리 있는 작은 큐브는 화면 위쪽에 뜨므로 바닥에 붙을 수 없다).
       후진하다가 박스가 바닥에서 떨어지거나 온전한 모양이 되면 일반 로직(2.)으로 돌아가고,
       거기서 폭이 목표 ± tolerance 안에 들면 멈춘다 (커진 채로면 --approach.allow_backward 로 더 물러난다).
       바닥 판정 여유는 --approach.bottom_margin_px (기본 3px), 끄려면 --approach.backward_when_clipped=false.
    3. 좌우 정렬은 **제자리 회전이 최우선** (--approach.lateral_mode=rotate, 기본):
           박스 중심 x 가 세로 중앙선에서 ±--approach.center_tolerance_px 밖  → 전진을 멈추고 제자리 회전만
           (화면 오른쪽에 있으면 우회전, 왼쪽이면 좌회전)
           ±tolerance 안                                                       → 회전을 멈추고 2. 의 전진
       전진 중에 중심이 tolerance + --approach.center_hysteresis_px 이상 벗어나면 다시 회전 단계로 돌아간다
       (경계에서 회전/전진이 번갈아 떨리는 것을 막는 여유).
       --approach.lateral_mode=strafe 면 회전 대신 옆 이동(y.vel)으로, 전진과 동시에 맞춘다.
    4. 두 조건이 연속 --approach.settle_frames 프레임 동안 만족되면 ALIGNED.
       기본으로는 그 뒤에도 계속 감시하며 벗어나면 다시 맞춘다 (--approach.stop_when_done=true 면 종료).
    5. 검출이 --approach.lost_timeout_s 이상 끊기면 바퀴를 멈춘다. 접근 중 팔은 시작 시 자세로 고정한다.
    6. 팔 (--pick.*): ALIGNED 가 되면 바퀴를 세운 채 팔을 **pick 직전 자세**로 --pick.move_time_s 초 동안
       선형 보간으로 움직인다 (ARM_TO_PICK → PICK_READY). 자세는 lekiwi_save_pose.py 로 미리 저장한
       --pick.pose_file (기본 poses/pre_pick.json). PICK_READY 상태에서 큐브가 --pick.drift_frames 프레임
       이상 정렬에서 벗어나거나 사라지면 팔을 시작 자세로 되돌린 뒤(ARM_TO_HOME) 다시 접근한다.
       팔이 움직이는 동안과 PICK_READY 에서는 바퀴를 절대 움직이지 않는다.
       --pick.exit_when_ready=true 면 PICK_READY 에서 스크립트를 끝낸다 (팔은 그 자세로 남는다).

pick 자세 저장 (리더암/손으로 팔을 원하는 자세로 만든 뒤):

    python lekiwi_save_pose.py --name pre_pick

목표 크기 재는 법: lekiwi_yolo_view.py 로 원하는 거리에 큐브를 두고 화면의 박스 폭(px)을 읽어
`--approach.target_size_px` 로 넘긴다. 기본값 117 은 640x480 front 뷰에서 큐브가 화면 아래쪽에
꽉 차게 보이는 거리 기준.

준비:

    pip install ultralytics
    python download_hf_model.py            # weights/best.pt

라즈베리파이(LeKiwi)에서 lekiwi_host 를 먼저 띄운다 (lekiwi_yolo_view.py 의 설명과 동일). 그 다음 PC 에서:

    python lekiwi_yolo_pick.py

먼저 움직이지 않고 어떻게 판단하는지만 보려면:

    python lekiwi_yolo_pick.py --dry_run=true

전부 인자로 바꿀 수 있다:

    python lekiwi_yolo_pick.py \
        --robot.remote_ip=192.168.0.201 \
        --robot.id=lekiwi01 \
        --yolo.path=weights/best.pt \
        --yolo.conf=0.5 \
        --yolo.device=0 \
        --approach.target_size_px=117 \
        --approach.size_tolerance_px=10 \
        --approach.center_tolerance_px=10 \
        --approach.max_speed=0.1 \
        --views='[front, wrist]' \
        --fps=30

창 조작키
    SPACE : 바퀴 제어 일시정지/재개 (일시정지 중에는 정지 명령만 보낸다)
    Q/ESC : 종료 (바퀴 정지 후 연결 해제)

주의
    * 호스트에 클라이언트는 하나만 붙는다. teleoperate/record 가 켜져 있으면 먼저 끌 것.
    * 실제로 바퀴가 돈다. 처음엔 --dry_run=true 로 방향이 맞는지 확인하고,
      --approach.max_speed 를 낮게(0.05~0.1 m/s) 두고 시작할 것.
    * 호스트에는 워치독이 있어 명령이 끊기면 스스로 바퀴를 멈춘다. 이 스크립트도 종료/검출 실패 시
      정지 명령을 보낸다.

전체 옵션은 `python lekiwi_yolo_pick.py --help`.
"""

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from pprint import pformat

import cv2
import numpy as np

from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

from lekiwi_yolo_view import (
    Detection,
    LeKiwiRobotArgs,
    YoloArgs,
    camera_views,
    draw,
    hstack_views,
    infer,
    load_model,
    wait_for_frames,
)

from lekiwi_save_pose import load_pose

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_PICK_POSE = SCRIPT_DIR / "poses" / "pre_pick.json"

LINE_COLOR = (0, 255, 0)  # 가운데 세로선 (초록, BGR) — lekiwi_yolo_view 의 십자선과 같은 색
BAND_COLOR = (0, 180, 0)  # 좌우 허용 범위
TARGET_COLOR = (255, 0, 255)  # 목표(가장 큰) 박스 강조
SIZE_REF_COLOR = (0, 255, 255)  # 목표 크기 참조 박스 (노랑)
IGNORED_COLOR = (140, 140, 140)  # 가로선 위라서 무시한 검출 (회색)
STATE_COLORS = {
    "SEARCHING": (200, 200, 200),
    "ABOVE_LINE": (200, 200, 200),
    "ROTATING": (255, 200, 0),
    "TOO_CLOSE": (0, 0, 255),
    "ARM_TO_PICK": (255, 255, 0),
    "PICK_READY": (0, 255, 0),
    "ARM_TO_HOME": (255, 255, 0),
    "ALIGNING": (0, 165, 255),
    "ALIGNED": (0, 220, 0),
    "PAUSED": (255, 120, 0),
    "DRY_RUN": (255, 200, 0),
}


@dataclass
class ApproachArgs:
    """접근(전진) + 좌우 정렬 제어 설정."""

    # 추론/정렬에 쓸 카메라 (호스트 --robot.cameras 이름)
    view: str = "front"

    # --- 후보 필터: 가운데 가로선 아래에 있는 검출만 ---

    # 박스 중심이 이 가로선(이미지 높이 × line_ratio) 아래에 있어야 목표가 된다. 0.5 = 정중앙
    line_ratio: float = 0.5
    # false 면 가로선 위/아래 상관없이 가장 큰 박스를 따라간다
    only_below_line: bool = True

    # --- 전진: 박스 크기 기준 ---

    # 이 크기(px)로 보일 때까지 전진한다. lekiwi_yolo_view.py 로 원하는 거리에서 박스 폭을 읽어 넣는다.
    target_size_px: int = 117
    # 크기 판정 기준: width(폭, 기본) / height(높이) / max(둘 중 큰 값)
    size_metric: str = "width"
    # 크기가 목표 - 이 값 이상이면 "도착"으로 본다
    size_tolerance_px: int = 10
    # 박스가 화면 바닥에 붙어 있으면서 납작하거나(아래 clip_aspect_min) 목표보다 작으면
    # "너무 가까워 잘린 것"으로 보고 후진한다
    backward_when_clipped: bool = True
    # 박스 아래 변이 (이미지 높이 - 이 값) 이상이면 "바닥에 붙었다"고 본다
    bottom_margin_px: int = 5
    # 바닥에 붙은 박스의 높이/폭 이 이 값보다 작으면 잘린 것. 온전한 큐브는 ≈1.1 (117x128)
    clip_aspect_min: float = 0.8
    # 크기 오차(px) → 전진 속도 게인 (m/s per px). 오차 50px → 0.1 m/s
    kp_forward: float = 0.002
    # 목표보다 (tolerance 이상) 커졌을 때 뒤로 물러나 크기를 맞춘다. false 면 그냥 정지
    allow_backward: bool = True

    # --- 좌우: 가운데 세로선 기준 ---

    # 박스 중심 x 가 세로 중앙선에서 이 픽셀 안에 들면 좌우 정렬된 것으로 본다
    center_tolerance_px: int = 100
    # rotate (기본) : 제자리 회전(theta.vel)으로 먼저 맞추고, 맞은 뒤에만 전진한다
    # strafe        : 옆 이동(y.vel)으로 맞추며 전진과 동시에 한다
    lateral_mode: str = "rotate"
    # [rotate] 전진 중 중심이 tolerance + 이 값 이상 벗어나야 다시 회전 단계로 돌아간다 (떨림 방지)
    center_hysteresis_px: int = 10
    # [rotate] 좌우 오차(px) → 회전 속도 게인 (deg/s per px). 오차 100px → 30 deg/s
    kp_rotate: float = 0.3
    # [rotate] 회전 속도 상/하한 (deg/s). LeKiwi 텔레옵 저속 단계가 30 deg/s
    max_theta_speed: float = 30.0
    min_theta_speed: float = 8.0
    # [strafe] 좌우 오차(px) → 옆 이동 속도 게인 (m/s per px)
    kp_lateral: float = 0.002

    # --- 공통 ---

    # 속도 상/하한 (m/s). min 은 바퀴가 실제로 굴러가는 최소치, max 는 안전 상한
    max_speed: float = 0.1
    min_speed: float = 0.03
    # 연속 N 프레임 두 조건 모두 만족이면 ALIGNED
    settle_frames: int = 10
    # 검출이 이 시간(초) 이상 없으면 바퀴 정지
    lost_timeout_s: float = 0.5
    # true 면 ALIGNED 되는 순간 정지하고 스크립트를 끝낸다. false 면 계속 감시/재정렬
    stop_when_done: bool = False


@dataclass
class PickArgs:
    """접근 완료(ALIGNED) 후 팔을 pick 직전 자세로 보내는 설정."""

    # false 면 팔은 시작 자세를 유지하고 접근/정렬만 한다
    enabled: bool = True
    # lekiwi_save_pose.py 로 저장한 자세 파일
    pose_file: str = str(DEFAULT_PICK_POSE)
    # 현재 자세 → pick 자세로 보내는 데 걸리는 시간(초). 선형 보간으로 천천히 움직인다
    move_time_s: float = 2.0
    # pick 자세에 도달하면 스크립트를 끝낸다 (팔은 그 자세로 남는다). false 면 자세를 유지하며 계속 감시
    exit_when_ready: bool = False
    # pick 자세로 있는 동안 큐브가 정렬에서 이 프레임 수 이상 연속 벗어나면(또는 사라지면)
    # 팔을 시작 자세로 되돌린 뒤 다시 접근한다. 0 이면 되돌리지 않고 계속 유지
    drift_frames: int = 30


@dataclass
class LeKiwiPickConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    yolo: YoloArgs = field(default_factory=YoloArgs)
    approach: ApproachArgs = field(default_factory=ApproachArgs)
    pick: PickArgs = field(default_factory=PickArgs)

    # 화면에 표시할 카메라 (front 왼쪽, wrist 오른쪽 — lekiwi_yolo_view.py 와 동일). approach.view 는 자동으로 포함된다.
    # 추론은 모든 뷰에 대해 한 배치로 돌리고, 정렬 제어에는 approach.view 만 쓴다.
    views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    # 제어/표시 루프 주기
    fps: int = 30
    # 이 시간(초)이 지나면 자동 종료. None 이면 Q/ESC/Ctrl+C 까지 계속
    run_time_s: float | None = None
    # true 면 계산만 하고 바퀴 명령은 보내지 않는다 (항상 정지 명령)
    dry_run: bool = False
    # 시작할 때 일시정지 상태로 시작 (SPACE 로 시작)
    start_paused: bool = False

    # cv2 - OpenCV 창 표시 (기본) / none - 표시 없이 터미널 상태만
    display: str = "cv2"
    view_height: int = 480
    # 가상의 중앙 가로선/세로선(초록)을 그릴 뷰 (lekiwi_yolo_view.py 와 동일). 끄려면 --crosshair_views='[]'
    crosshair_views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    print_status: bool = True

    def validate(self) -> None:
        a = self.approach
        if self.display not in ("cv2", "none"):
            raise SystemExit(f"error: --display 는 cv2/none 중 하나여야 합니다 (받은 값: {self.display})")
        if self.fps <= 0:
            raise SystemExit(f"error: --fps 는 1 이상이어야 합니다 (받은 값: {self.fps})")
        if a.size_metric not in ("width", "height", "max"):
            raise SystemExit(
                f"error: --approach.size_metric 은 width/height/max 중 하나여야 합니다 (받은 값: {a.size_metric})"
            )
        if not 0.0 < a.line_ratio < 1.0:
            raise SystemExit(f"error: --approach.line_ratio 는 0~1 사이여야 합니다 (받은 값: {a.line_ratio})")
        if a.target_size_px <= 0:
            raise SystemExit(f"error: --approach.target_size_px 는 1 이상이어야 합니다 (받은 값: {a.target_size_px})")
        if a.size_tolerance_px < 0 or a.center_tolerance_px < 0 or a.center_hysteresis_px < 0:
            raise SystemExit(
                "error: --approach.size_tolerance_px / center_tolerance_px / center_hysteresis_px 는 0 이상이어야 합니다"
            )
        if a.clip_aspect_min <= 0:
            raise SystemExit(f"error: --approach.clip_aspect_min 은 0 보다 커야 합니다 (받은 값: {a.clip_aspect_min})")
        if a.bottom_margin_px < 0:
            raise SystemExit(f"error: --approach.bottom_margin_px 는 0 이상이어야 합니다 (받은 값: {a.bottom_margin_px})")
        if a.lateral_mode not in ("rotate", "strafe"):
            raise SystemExit(
                f"error: --approach.lateral_mode 는 rotate/strafe 중 하나여야 합니다 (받은 값: {a.lateral_mode})"
            )
        if a.max_theta_speed <= 0 or a.min_theta_speed < 0 or a.min_theta_speed > a.max_theta_speed:
            raise SystemExit(
                "error: 0 <= --approach.min_theta_speed <= --approach.max_theta_speed, max_theta_speed > 0 이어야 합니다 "
                f"(받은 값: min={a.min_theta_speed}, max={a.max_theta_speed})"
            )
        if a.max_speed <= 0 or a.min_speed < 0 or a.min_speed > a.max_speed:
            raise SystemExit(
                "error: 0 <= --approach.min_speed <= --approach.max_speed, --approach.max_speed > 0 이어야 합니다 "
                f"(받은 값: min={a.min_speed}, max={a.max_speed})"
            )
        if a.view not in self.views:
            self.views = [a.view, *self.views]
        if self.pick.enabled:
            if self.pick.move_time_s <= 0:
                raise SystemExit(f"error: --pick.move_time_s 는 0 보다 커야 합니다 (받은 값: {self.pick.move_time_s})")
            if self.pick.drift_frames < 0:
                raise SystemExit(f"error: --pick.drift_frames 는 0 이상이어야 합니다 (받은 값: {self.pick.drift_frames})")
            if not Path(self.pick.pose_file).expanduser().exists():
                raise SystemExit(
                    f"error: pick 자세 파일이 없습니다: {self.pick.pose_file}\n"
                    "  팔을 pick 직전 자세로 만든 뒤 저장하세요:\n"
                    f"      python {SCRIPT_DIR / 'lekiwi_save_pose.py'} --name pre_pick\n"
                    "  팔을 움직이지 않으려면 --pick.enabled=false"
                )


STOP = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}


def box_size(det: Detection, metric: str) -> int:
    x1, y1, x2, y2 = det.xyxy
    w, h = x2 - x1, y2 - y1
    if metric == "width":
        return w
    if metric == "height":
        return h
    return max(w, h)


def largest(dets: list[Detection]) -> Detection | None:
    """면적이 가장 큰 검출 (없으면 None)."""
    if not dets:
        return None
    return max(dets, key=lambda d: (d.xyxy[2] - d.xyxy[0]) * (d.xyxy[3] - d.xyxy[1]))


def p_speed(error_px: float, kp: float, tolerance_px: int, cfg: ApproachArgs) -> float:
    """픽셀 오차 크기 → 속도 크기 (허용치 안이면 0, 아니면 [min, max] 로 클램프한 P 제어)."""
    if abs(error_px) <= tolerance_px:
        return 0.0
    return float(np.clip(kp * abs(error_px), cfg.min_speed, cfg.max_speed))


class Approacher:
    """가장 큰 박스를 향해 (크기 → 전진, 중심 x → 좌우) 맞추는 P 제어기 + 상태 추적."""

    def __init__(self, cfg: ApproachArgs):
        self.cfg = cfg
        self.settled = 0
        self.last_seen = -float("inf")
        self.target: Detection | None = None
        self.ignored: list[Detection] = []  # 가로선 위라서 무시한 검출 (표시용)
        self.size = 0  # 현재 박스 크기(px)
        self.size_error = 0  # 목표 - 현재 (+ : 아직 작다 = 멀다)
        self.center_error = 0  # 중심 x - 화면 중앙 (+ : 화면 오른쪽)
        self.size_ok = False
        self.center_ok = False
        self.aspect = 0.0  # 박스 높이/폭 (온전한 큐브 ≈ 1.1, 잘리면 작아진다)
        self.touching_bottom = False  # 박스 아래 변이 화면 바닥에 붙어 있음
        self.too_close = False  # 바닥에 붙었는데 목표보다 작다 → 잘려 나감 → 후진
        # [rotate 모드] ROTATE: 제자리 회전으로 중심 맞추는 중 / FORWARD: 맞았으니 전진
        self.phase = "ROTATE"
        self.state = "SEARCHING"

    def line_y(self, frame_h: int) -> int:
        return int(round(frame_h * self.cfg.line_ratio))

    def update(self, dets: list[Detection], frame_shape: tuple[int, ...], now: float) -> dict[str, float]:
        """검출 결과로 베이스 속도 명령을 계산한다. 반환값은 항상 x/y/theta.vel 세 개."""
        h, w = frame_shape[:2]

        # 가운데 가로선 아래(가까운 쪽)에 중심이 있는 검출만 후보. 위쪽은 멀리 있는 것으로 보고 무시.
        if self.cfg.only_below_line:
            line_y = self.line_y(h)
            candidates = [d for d in dets if d.center[1] > line_y]
            self.ignored = [d for d in dets if d.center[1] <= line_y]
        else:
            candidates, self.ignored = list(dets), []
        self.target = largest(candidates)

        if self.target is None:
            if self.ignored:
                # 보이긴 하지만 전부 가로선 위 → 확실한 신호이므로 바로 정지/대기
                self.settled = 0
                self.state = "ABOVE_LINE"
            elif now - self.last_seen > self.cfg.lost_timeout_s:
                self.settled = 0
                self.state = "SEARCHING"
            # 그 외(lost_timeout 이전)는 잠깐 놓친 것으로 보고 정지만 (상태 유지)
            return dict(STOP)

        self.last_seen = now
        cx, _ = self.target.center
        self.size = box_size(self.target, self.cfg.size_metric)
        self.size_error = self.cfg.target_size_px - self.size
        self.center_error = cx - w // 2

        # 도착 판정: 크기는 목표 - tol 이상 (allow_backward 면 목표 + tol 이하도), 좌우는 ±tol
        self.size_ok = self.size >= self.cfg.target_size_px - self.cfg.size_tolerance_px
        if self.cfg.allow_backward:
            self.size_ok = self.size_ok and self.size <= self.cfg.target_size_px + self.cfg.size_tolerance_px
        self.center_ok = abs(self.center_error) <= self.cfg.center_tolerance_px

        # 너무 가까워서 잘렸는지: 바닥에 붙어 있으면서
        #   (a) 납작하다 (높이/폭 < clip_aspect_min) — 윗면 한 줌만 보이는 상태, 폭은 커도 잘린 것
        #   (b) 크기가 목표 - tol 보다 작다
        x1, y1, x2, y2 = self.target.xyxy
        self.aspect = (y2 - y1) / max(1, x2 - x1)
        self.touching_bottom = y2 >= h - self.cfg.bottom_margin_px
        too_small = self.size < self.cfg.target_size_px - self.cfg.size_tolerance_px
        self.too_close = (
            self.cfg.backward_when_clipped
            and self.touching_bottom
            and (self.aspect < self.cfg.clip_aspect_min or too_small)
        )
        if self.too_close:
            self.size_ok = False  # 잘린 폭은 믿을 수 없다 → 도착으로 치지 않는다

        x_vel = y_vel = theta_vel = 0.0
        rotating = False

        if self.too_close:
            # 0순위: 후진. 잘린 상태에서는 폭/중심 모두 믿을 수 없으니 회전/전진은 하지 않는다.
            #   얼마나 잘렸는지는 모르므로 일정 속도(max_speed 의 절반, min_speed 이상)로 물러난다.
            x_vel = -max(self.cfg.min_speed, self.cfg.max_speed * 0.5)
            self.phase = "ROTATE"  # 후진이 끝나면 중심부터 다시 맞춘다
        elif self.cfg.lateral_mode == "rotate":
            # 1순위: 제자리 회전으로 중심 맞추기. 회전 중에는 전진하지 않는다.
            #   ROTATE  → 중심이 ±tol 안에 들면 FORWARD 로
            #   FORWARD → 중심이 ±(tol + hysteresis) 밖으로 벗어나면 다시 ROTATE 로
            if self.phase == "FORWARD" and abs(self.center_error) > (
                self.cfg.center_tolerance_px + self.cfg.center_hysteresis_px
            ):
                self.phase = "ROTATE"
            if self.phase == "ROTATE" and self.center_ok:
                self.phase = "FORWARD"

            if self.phase == "ROTATE":
                # 화면 오른쪽(+) → 우회전. LeKiwi 는 theta + 가 좌회전(CCW) 이므로 -theta.
                theta_vel = -np.sign(self.center_error) * self._theta_speed(self.center_error)
                rotating = True
            else:
                x_vel = self._forward_speed()
        else:
            # strafe: 옆 이동과 전진을 동시에. 화면 오른쪽(+) → 오른쪽 이동 = 몸체 y 는 왼쪽이 + 이므로 -y.
            y_vel = -np.sign(self.center_error) * p_speed(
                self.center_error, self.cfg.kp_lateral, self.cfg.center_tolerance_px, self.cfg
            )
            x_vel = self._forward_speed()

        self.settled = self.settled + 1 if (self.size_ok and self.center_ok) else 0
        if self.settled >= self.cfg.settle_frames:
            self.state = "ALIGNED"
        elif self.too_close:
            self.state = "TOO_CLOSE"
        elif rotating:
            self.state = "ROTATING"
        else:
            self.state = "ALIGNING"

        return {"x.vel": float(x_vel), "y.vel": float(y_vel), "theta.vel": float(theta_vel)}

    def _forward_speed(self) -> float:
        """크기 오차 → 전진 속도. 아직 작다(+) → 앞으로. 커졌으면 기본은 정지, allow_backward 면 후진."""
        if self.size_error > 0:
            return p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        if self.cfg.allow_backward:
            return -p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        return 0.0

    def _theta_speed(self, error_px: float) -> float:
        """좌우 픽셀 오차 크기 → 회전 속도 크기 (deg/s), [min, max] 로 클램프."""
        if abs(error_px) <= self.cfg.center_tolerance_px:
            return 0.0
        return float(np.clip(self.cfg.kp_rotate * abs(error_px), self.cfg.min_theta_speed, self.cfg.max_theta_speed))

    @property
    def done(self) -> bool:
        return self.state == "ALIGNED"


def draw_alignment(
    frame_bgr: np.ndarray, ap: Approacher, cmd: dict[str, float], state_label: str
) -> np.ndarray:
    """오버레이: 가운데 세로선 + 좌우 허용 띠 + 목표 박스 강조 + 목표 크기 참조 박스 + 오차/명령 텍스트."""
    canvas = frame_bgr
    h, w = canvas.shape[:2]
    tol = ap.cfg.center_tolerance_px
    mx = w // 2

    # 좌우 허용 범위 띠(반투명) + 가운데 세로선
    band = canvas.copy()
    cv2.rectangle(band, (mx - tol, 0), (mx + tol, h), BAND_COLOR, -1)
    cv2.addWeighted(band, 0.25, canvas, 0.75, 0, canvas)
    cv2.line(canvas, (mx, 0), (mx, h), LINE_COLOR, 1, cv2.LINE_AA)

    # 가운데 가로선: 이 선 아래의 검출만 목표가 된다
    if ap.cfg.only_below_line:
        ly = ap.line_y(h)
        cv2.line(canvas, (0, ly), (w, ly), LINE_COLOR, 1, cv2.LINE_AA)
        cv2.putText(canvas, "target zone", (6, ly + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, LINE_COLOR, 1, cv2.LINE_AA)

    # 가로선 위라서 무시한 검출은 회색으로 덮어 표시
    for d in ap.ignored:
        x1, y1, x2, y2 = d.xyxy
        cv2.rectangle(canvas, (x1, y1), (x2, y2), IGNORED_COLOR, 2)
        cv2.line(canvas, (x1, y1), (x2, y2), IGNORED_COLOR, 1)
        cv2.line(canvas, (x1, y2), (x2, y1), IGNORED_COLOR, 1)
        cv2.putText(canvas, "ignored", (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, IGNORED_COLOR, 1, cv2.LINE_AA)

    t = ap.target
    if t is not None:
        x1, y1, x2, y2 = t.xyxy
        cx, cy = t.center
        cv2.rectangle(canvas, (x1, y1), (x2, y2), TARGET_COLOR, 3)
        cv2.circle(canvas, (cx, cy), 6, TARGET_COLOR, 2)
        # 중심 → 세로선까지의 좌우 오차를 가로 화살표로
        if abs(ap.center_error) > tol:
            cv2.arrowedLine(canvas, (cx, cy), (mx, cy), TARGET_COLOR, 2, tipLength=0.3)
        # 목표 크기 참조: 박스 중심에 목표 폭(높이는 현재 비율 유지)의 노란 사각형
        ts = ap.cfg.target_size_px
        ref_w = ts if ap.cfg.size_metric != "height" else max(1, int(round((x2 - x1) * ts / max(1, y2 - y1))))
        ref_h = ts if ap.cfg.size_metric != "width" else max(1, int(round((y2 - y1) * ts / max(1, x2 - x1))))
        cv2.rectangle(canvas, (cx - ref_w // 2, cy - ref_h // 2), (cx + ref_w // 2, cy + ref_h // 2), SIZE_REF_COLOR, 1, cv2.LINE_AA)

        if ap.touching_bottom:
            # 바닥에 붙은 박스: 아래 변을 두껍게 표시. 너무 가까우면 빨간 경고까지
            edge_color = STATE_COLORS["TOO_CLOSE"] if ap.too_close else TARGET_COLOR
            cv2.line(canvas, (x1, h - 2), (x2, h - 2), edge_color, 4)
            if ap.too_close:
                cv2.putText(canvas, f"TOO CLOSE (h/w={ap.aspect:.2f}) - backing up", (x1, max(y1 - 52, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, edge_color, 2, cv2.LINE_AA)

        size_txt = f"{ap.cfg.size_metric}={ap.size}/{ts}px" + (" OK" if ap.size_ok else "")
        dx_txt = f"dx={ap.center_error:+d}px" + (" OK" if ap.center_ok else "")
        cv2.putText(canvas, f"{size_txt}  {dx_txt}", (x1, max(y1 - 30, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_COLOR, 2, cv2.LINE_AA)

    color = STATE_COLORS.get(state_label, (255, 255, 255))
    cmd_txt = f"x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} m/s  th={cmd['theta.vel']:+5.1f} deg/s"
    cv2.rectangle(canvas, (0, h - 26), (w, h), (0, 0, 0), -1)
    cv2.putText(canvas, f"{state_label}  {cmd_txt}", (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return canvas


def status_line(ap: Approacher, cmd: dict[str, float], state_label: str, hz: float) -> str:
    if ap.target is None:
        target = f"target: - (가로선 위 {len(ap.ignored)}개 무시)" if ap.ignored else "target: -"
    else:
        cx, cy = ap.target.center
        target = (
            f"target {ap.target.name} {ap.target.conf:.2f} @({cx},{cy}) "
            f"{ap.cfg.size_metric}={ap.size}/{ap.cfg.target_size_px}{'✓' if ap.size_ok else ''} "
            f"dx={ap.center_error:+d}{'✓' if ap.center_ok else ''}"
            + (f" [바닥·잘림 h/w={ap.aspect:.2f}→후진]" if ap.too_close else f" [바닥 h/w={ap.aspect:.2f}]" if ap.touching_bottom else "")
        )
    return (
        f"\r[{state_label:10s}] {target} | x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} th={cmd['theta.vel']:+5.1f} "
        f"| settled {ap.settled}/{ap.cfg.settle_frames} | {hz:5.1f} Hz   "
    )


class ArmSequencer:
    """접근 완료 후 팔을 pick 자세로, 벗어나면 다시 시작 자세로 보내는 상태기.

    상태: HOME(시작 자세 유지) → TO_PICK(보간 이동) → PICK(pick 자세 유지) → TO_HOME(보간 복귀) → HOME
    팔이 HOME 이 아닐 때는 바퀴를 움직이면 안 된다 (`base_locked`).
    """

    def __init__(self, home: dict[str, float], pick: dict[str, float] | None, cfg: PickArgs):
        self.cfg = cfg
        self.home = dict(home)
        self.pick = dict(pick) if pick else None
        self.state = "HOME"
        self.current = dict(home)  # 지금 보내고 있는 자세
        self._from: dict[str, float] = {}
        self._to: dict[str, float] = {}
        self._t0 = 0.0
        self.progress = 0.0
        self.drift = 0
        self.just_ready = False  # 이번 프레임에 PICK 에 도달했는지

    @property
    def enabled(self) -> bool:
        return self.cfg.enabled and self.pick is not None

    @property
    def base_locked(self) -> bool:
        return self.state != "HOME"

    def _start_move(self, target: dict[str, float], state: str, now: float) -> None:
        self._from = dict(self.current)
        self._to = {k: target.get(k, self.current[k]) for k in self.current}
        self._t0 = now
        self.progress = 0.0
        self.state = state

    def update(self, aligned: bool, tracking_ok: bool, allow_motion: bool, now: float) -> dict[str, float]:
        """이번 프레임에 보낼 팔 목표 자세를 돌려준다.

        aligned      : Approacher 가 ALIGNED 인지
        tracking_ok  : 큐브가 여전히 정렬 허용치 안에 있는지 (size_ok and center_ok)
        allow_motion : 일시정지/dry-run 이 아닌지
        """
        self.just_ready = False
        if not self.enabled:
            return self.current

        if self.state == "HOME":
            if aligned and allow_motion:
                self._start_move(self.pick, "TO_PICK", now)
        elif self.state in ("TO_PICK", "TO_HOME"):
            a = min(1.0, (now - self._t0) / self.cfg.move_time_s)
            self.progress = a
            self.current = {k: self._from[k] + (self._to[k] - self._from[k]) * a for k in self.current}
            if a >= 1.0:
                if self.state == "TO_PICK":
                    self.state = "PICK"
                    self.drift = 0
                    self.just_ready = True
                else:
                    self.state = "HOME"
        elif self.state == "PICK":
            if self.cfg.drift_frames > 0:
                self.drift = 0 if tracking_ok else self.drift + 1
                if self.drift >= self.cfg.drift_frames and allow_motion:
                    self._start_move(self.home, "TO_HOME", now)
        return self.current

    @property
    def label(self) -> str | None:
        return {"TO_PICK": "ARM_TO_PICK", "PICK": "PICK_READY", "TO_HOME": "ARM_TO_HOME"}.get(self.state)


def control_loop(
    robot: LeKiwiClient,
    model,
    views: list[str],
    arm_hold: dict,
    pick_pose: dict[str, float] | None,
    cfg: LeKiwiPickConfig,
) -> None:
    window = "LeKiwi YOLO pick (SPACE 일시정지, Q/ESC 종료)"
    ap = Approacher(cfg.approach)
    arm = ArmSequencer(arm_hold, pick_pose, cfg.pick)
    interval = 1.0 / cfg.fps
    start = time.perf_counter()
    paused = cfg.start_paused
    hz = 0.0
    cmd = dict(STOP)

    while True:
        loop_start = time.perf_counter()

        obs = robot.get_observation()
        # LeKiwiClient 프레임은 RGB → YOLO/cv2 용 BGR
        frames_bgr = {
            v: cv2.cvtColor(obs[v], cv2.COLOR_RGB2BGR) for v in views if isinstance(obs.get(v), np.ndarray)
        }
        if cfg.approach.view not in frames_bgr:
            robot.send_action({**arm.current, **STOP})
            time.sleep(0.05)
            continue

        dets_by_view = infer(model, cfg.yolo, frames_bgr)
        cmd = ap.update(dets_by_view[cfg.approach.view], frames_bgr[cfg.approach.view].shape, loop_start)

        allow_motion = not paused and not cfg.dry_run
        tracking_ok = ap.target is not None and ap.size_ok and ap.center_ok
        arm_pose = arm.update(ap.done, tracking_ok, allow_motion, loop_start)

        if paused:
            state_label = "PAUSED"
            sent = dict(STOP)
        elif cfg.dry_run:
            state_label = "DRY_RUN"
            sent = dict(STOP)
        elif arm.base_locked:
            # 팔이 움직이는 중이거나 pick 자세 → 바퀴 고정
            state_label = arm.label or ap.state
            sent = dict(STOP)
        else:
            state_label = ap.state
            sent = cmd
        # 팔: 시작 자세 / 보간 중 / pick 자세, 바퀴: 계산한 속도 또는 정지
        robot.send_action({**arm_pose, **sent})

        if arm.just_ready:
            print(f"\npick 직전 자세 도달 ({cfg.approach.size_metric}={ap.size}px, dx={ap.center_error:+d}px).")
            if cfg.pick.exit_when_ready:
                print("--pick.exit_when_ready=true 이므로 종료합니다. 팔은 pick 자세로 남습니다.")
                return

        if cfg.display == "cv2":
            panels = []
            for v in views:
                if v not in frames_bgr:
                    continue
                img = draw(frames_bgr[v], v, dets_by_view.get(v, []), hz, crosshair=v in cfg.crosshair_views)
                if v == cfg.approach.view:
                    img = draw_alignment(img, ap, cmd, state_label)
                panels.append(img)
            cv2.imshow(window, hstack_views(panels, cfg.view_height))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("\n종료합니다...")
                return
            if key == ord(" "):
                paused = not paused
                print("\n일시정지" if paused else "\n재개")
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                print("\n창이 닫혔습니다. 종료합니다...")
                return

        if cfg.approach.stop_when_done and ap.done and not paused and not cfg.dry_run:
            print(
                f"\n정렬 완료 ({cfg.approach.size_metric}={ap.size}px, dx={ap.center_error:+d}px). "
                "--approach.stop_when_done=true 이므로 종료합니다."
            )
            return

        dt = time.perf_counter() - loop_start
        precise_sleep(max(interval - dt, 0.0))
        hz = 1.0 / max(time.perf_counter() - loop_start, 1e-6)

        if cfg.print_status:
            line = status_line(ap, sent, state_label, hz)
            if arm.state in ("TO_PICK", "TO_HOME"):
                line += f"| arm {arm.state} {arm.progress * 100:3.0f}% "
            elif arm.state == "PICK":
                line += f"| arm PICK drift {arm.drift}/{cfg.pick.drift_frames} "
            print(line, end="", flush=True)

        if cfg.run_time_s is not None and time.perf_counter() - start >= cfg.run_time_s:
            print(f"\n--run_time_s={cfg.run_time_s} 경과, 종료합니다.")
            return


def latch_arm_pose(robot: LeKiwiClient) -> dict[str, float]:
    """현재 팔 관절 위치를 읽어 고정 목표로 쓴다 (베이스만 움직이고 팔은 그대로)."""
    obs = robot.get_observation()
    hold = {k: float(v) for k, v in obs.items() if k.endswith(".pos")}
    if not hold:
        logging.warning("팔 관절 위치를 읽지 못했습니다. 팔 목표 없이 바퀴만 제어합니다.")
    return hold


@parser.wrap()
def main(cfg: LeKiwiPickConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    robot = LeKiwiClient(cfg.robot.to_config())
    views = camera_views(robot, cfg.views)
    model = load_model(cfg.yolo)

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
                "  - teleoperate/record 가 이미 붙어 있으면 먼저 종료하세요 (클라이언트는 하나만 붙습니다)."
            ) from e

        wait_for_frames(robot, views)
        arm_hold = latch_arm_pose(robot)

        pick_pose = None
        if cfg.pick.enabled:
            pick_pose = load_pose(Path(cfg.pick.pose_file).expanduser())
            missing = sorted(set(arm_hold) - set(pick_pose))
            extra = sorted(set(pick_pose) - set(arm_hold))
            if missing:
                logging.warning("pick 자세 파일에 없는 관절 %s 은 시작 자세를 유지합니다.", missing)
            if extra:
                logging.warning("pick 자세 파일의 %s 은 로봇에 없는 관절이라 무시합니다.", extra)
                pick_pose = {k: v for k, v in pick_pose.items() if k in arm_hold}
            logging.info("pick 자세 (%s): %s", cfg.pick.pose_file, {k: round(v, 1) for k, v in pick_pose.items()})

        a = cfg.approach
        mode = "DRY RUN (바퀴 명령 안 보냄)" if cfg.dry_run else "실제 주행"
        print(
            f"\n모드: {mode} | 뷰: {a.view} | 목표 {a.size_metric} {a.target_size_px}px (-{a.size_tolerance_px}) "
            f"| 좌우 ±{a.center_tolerance_px}px | 속도 {a.min_speed}~{a.max_speed} m/s"
            + (
                f"\n팔: ALIGNED 후 {cfg.pick.move_time_s}s 동안 pick 자세로 이동 ({Path(cfg.pick.pose_file).name})"
                if cfg.pick.enabled
                else "\n팔: 시작 자세 유지 (--pick.enabled=false)"
            )
            + "\n조작: SPACE 일시정지/재개, Q/ESC 종료, Ctrl+C 종료\n"
        )
        if cfg.start_paused:
            print("일시정지 상태로 시작합니다. SPACE 를 누르면 움직입니다.")

        try:
            control_loop(robot, model, views, arm_hold, pick_pose, cfg)
        except cv2.error as e:
            raise SystemExit(
                f"error: OpenCV 창을 띄우지 못했습니다: {e}\n  헤드리스/SSH 환경이면 --display=none 을 쓰세요."
            ) from e
    except KeyboardInterrupt:
        print("\nCtrl+C, 종료합니다...")
    finally:
        # 어떤 경로로 끝나든 바퀴를 멈추고 연결을 끊는다.
        if robot.is_connected:
            try:
                hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
                robot.send_action({**hold, **STOP})
                time.sleep(0.2)
            except Exception as e:
                logging.warning("정지 명령 전송 실패: %s", e)
            robot.disconnect()
        if cfg.display == "cv2":
            cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()
