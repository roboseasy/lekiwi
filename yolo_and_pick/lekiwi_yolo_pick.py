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
       박스가 목표보다 훨씬 커져도(너무 가까움) 기본으로는 그냥 멈춘다.
       --approach.allow_backward=true 면 뒤로 물러나 크기를 맞춘다.
       크기는 폭을 쓴다: 가까워질수록 큐브 아랫부분이 화면 아래로 잘려 높이는 믿을 수 없다.
    3. 좌우 정렬은 **제자리 회전이 최우선** (--approach.lateral_mode=rotate, 기본):
           박스 중심 x 가 세로 중앙선에서 ±--approach.center_tolerance_px 밖  → 전진을 멈추고 제자리 회전만
           (화면 오른쪽에 있으면 우회전, 왼쪽이면 좌회전)
           ±tolerance 안                                                       → 회전을 멈추고 2. 의 전진
       전진 중에 중심이 tolerance + --approach.center_hysteresis_px 이상 벗어나면 다시 회전 단계로 돌아간다
       (경계에서 회전/전진이 번갈아 떨리는 것을 막는 여유).
       --approach.lateral_mode=strafe 면 회전 대신 옆 이동(y.vel)으로, 전진과 동시에 맞춘다.
    4. 두 조건이 연속 --approach.settle_frames 프레임 동안 만족되면 ALIGNED.
       기본으로는 그 뒤에도 계속 감시하며 벗어나면 다시 맞춘다 (--approach.stop_when_done=true 면 종료).
    5. 검출이 --approach.lost_timeout_s 이상 끊기면 바퀴를 멈춘다. 팔은 시작 시 자세로 고정한다.

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

LINE_COLOR = (0, 255, 0)  # 가운데 세로선 (초록, BGR) — lekiwi_yolo_view 의 십자선과 같은 색
BAND_COLOR = (0, 180, 0)  # 좌우 허용 범위
TARGET_COLOR = (255, 0, 255)  # 목표(가장 큰) 박스 강조
SIZE_REF_COLOR = (0, 255, 255)  # 목표 크기 참조 박스 (노랑)
IGNORED_COLOR = (140, 140, 140)  # 가로선 위라서 무시한 검출 (회색)
STATE_COLORS = {
    "SEARCHING": (200, 200, 200),
    "ABOVE_LINE": (200, 200, 200),
    "ROTATING": (255, 200, 0),
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
    # 크기 오차(px) → 전진 속도 게인 (m/s per px). 오차 50px → 0.1 m/s
    kp_forward: float = 0.002
    # 목표보다 (tolerance 이상) 커졌을 때 뒤로 물러날지. false 면 그냥 정지
    allow_backward: bool = False

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
class LeKiwiPickConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    yolo: YoloArgs = field(default_factory=YoloArgs)
    approach: ApproachArgs = field(default_factory=ApproachArgs)

    # 화면에 표시할 카메라. approach.view 는 자동으로 포함된다.
    views: list[str] = field(default_factory=lambda: ["front"])
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

        # 전진: 아직 작다(+) → 앞으로. 목표보다 커졌으면 기본은 정지, allow_backward 면 후진.
        if self.size_error > 0:
            x_vel = p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        elif self.cfg.allow_backward:
            x_vel = -p_speed(self.size_error, self.cfg.kp_forward, self.cfg.size_tolerance_px, self.cfg)
        else:
            x_vel = 0.0

        # 좌우: 화면 오른쪽(+) → 오른쪽으로 이동. 몸체 y 는 왼쪽이 + 이므로 -y.
        y_vel = -np.sign(self.center_error) * p_speed(
            self.center_error, self.cfg.kp_lateral, self.cfg.center_tolerance_px, self.cfg
        )

        # 도착 판정: 크기는 "목표 - tol 이상" (커진 건 OK), 좌우는 ±tol
        self.size_ok = self.size >= self.cfg.target_size_px - self.cfg.size_tolerance_px
        if self.cfg.allow_backward:
            self.size_ok = self.size_ok and self.size <= self.cfg.target_size_px + self.cfg.size_tolerance_px
        self.center_ok = abs(self.center_error) <= self.cfg.center_tolerance_px

        self.settled = self.settled + 1 if (self.size_ok and self.center_ok) else 0
        self.state = "ALIGNED" if self.settled >= self.cfg.settle_frames else "ALIGNING"

        return {"x.vel": float(x_vel), "y.vel": float(y_vel), "theta.vel": 0.0}

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

        size_txt = f"{ap.cfg.size_metric}={ap.size}/{ts}px" + (" OK" if ap.size_ok else "")
        dx_txt = f"dx={ap.center_error:+d}px" + (" OK" if ap.center_ok else "")
        cv2.putText(canvas, f"{size_txt}  {dx_txt}", (x1, max(y1 - 30, 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_COLOR, 2, cv2.LINE_AA)

    color = STATE_COLORS.get(state_label, (255, 255, 255))
    cmd_txt = f"x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} m/s"
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
        )
    return (
        f"\r[{state_label:9s}] {target} | x={cmd['x.vel']:+.2f} y={cmd['y.vel']:+.2f} "
        f"| settled {ap.settled}/{ap.cfg.settle_frames} | {hz:5.1f} Hz   "
    )


def control_loop(robot: LeKiwiClient, model, views: list[str], arm_hold: dict, cfg: LeKiwiPickConfig) -> None:
    window = "LeKiwi YOLO pick (SPACE 일시정지, Q/ESC 종료)"
    ap = Approacher(cfg.approach)
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
            robot.send_action({**arm_hold, **STOP})
            time.sleep(0.05)
            continue

        dets_by_view = infer(model, cfg.yolo, frames_bgr)
        cmd = ap.update(dets_by_view[cfg.approach.view], frames_bgr[cfg.approach.view].shape, loop_start)

        if paused:
            state_label = "PAUSED"
            sent = dict(STOP)
        elif cfg.dry_run:
            state_label = "DRY_RUN"
            sent = dict(STOP)
        else:
            state_label = ap.state
            sent = cmd
        # 팔은 시작 자세로 고정, 바퀴는 계산한 속도 (또는 정지)
        robot.send_action({**arm_hold, **sent})

        if cfg.display == "cv2":
            panels = []
            for v in views:
                if v not in frames_bgr:
                    continue
                img = draw(frames_bgr[v], v, dets_by_view.get(v, []), hz)
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
            print(status_line(ap, sent, state_label, hz), end="", flush=True)

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

        a = cfg.approach
        mode = "DRY RUN (바퀴 명령 안 보냄)" if cfg.dry_run else "실제 주행"
        print(
            f"\n모드: {mode} | 뷰: {a.view} | 목표 {a.size_metric} {a.target_size_px}px (-{a.size_tolerance_px}) "
            f"| 좌우 ±{a.center_tolerance_px}px | 속도 {a.min_speed}~{a.max_speed} m/s"
            f"\n조작: SPACE 일시정지/재개, Q/ESC 종료, Ctrl+C 종료\n"
        )
        if cfg.start_paused:
            print("일시정지 상태로 시작합니다. SPACE 를 누르면 움직입니다.")

        try:
            control_loop(robot, model, views, arm_hold, cfg)
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
