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
    7. 집기 준비 (--grasp.*, wrist_servo.py): PICK_READY 에서 --grasp.pick_dwell_s 뒤
         7-0. 손목 뷰 검출은 정리해서 쓴다: 화면 왼쪽 절반(--grasp.left_region_ratio)에 중심이 있는 박스는
              conf ≥ --grasp.left_min_conf(0.85) 만 인정 (보라색 그리퍼 손가락 오검출 방지), 그 뒤
              가장 큰 박스 **하나만** 남긴다 (--grasp.single_box). 화면에도 그 하나만 그린다.
         7-1. 그리퍼를 최대로 연다 (--grasp.gripper_open_pct=100 = 캘리브레이션 range_max)      [GRIPPER_OPEN]
         7-2. 손목 뷰에서 큐브 박스의 **왼쪽 변**이 화면 가운데 세로선에 오고(±--grasp.x_tolerance_px),
              화면 중앙 높이가 박스 위/아래 변 사이에 들도록 shoulder_pan(가로)/wrist_flex(세로)를
              조정한다. 손목 카메라가 그리퍼 왼쪽에 있어 큐브가 화면 오른쪽에 있어야 집히기 때문
              (--grasp.x_anchor=left|center|right)                                              [WRIST_CENTERING]
         7-3. 중앙점이 박스 안에 있는 동안만 팔을 뻗어 박스가 점점 커지게 한다                    [WRIST_APPROACH]
              기본은 미리 저장한 grasp 자세(poses/grasp.json) 방향으로 보간 (--grasp.approach_mode=pose),
              또는 관절별 속도로 뻗기 (--grasp.approach_mode=joints --grasp.reach_joints='{...}')
         7-4. 손목 뷰 박스 폭이 --grasp.target_size_px 이상이고 가로/세로 기준이 맞으면 정지·유지   [GRASP_READY]
              크기는 됐는데 기준이 안 맞으면 더 뻗지 않고 보정만                                    [WRIST_REFINE]
       목표 구성은 poses/grasp_ref.json (--grasp.ref_file) 에서 읽는다: 집기 직전 '완벽한' 화면에서의
       박스 왼쪽 변 dx / 위 변 dy / 폭. 서보 화면에서 **S** 를 누르면 지금 구성이 그 파일로 저장된다.
         7-5. GRASP_READY 에서 --grasp.grasp_dwell_s 뒤 그리퍼를 닫는다 (--grasp.gripper_close_time_s 동안 보간).
              목표는 큐브를 물고 저장한 poses/grasp_closed.json 의 arm_gripper.pos (--grasp.close_pose_file,
              측정값 15.1%), 또는 --grasp.gripper_close_pct 직접 지정. --grasp.gripper_close_extra_pct 만큼
              더 조일 수 있다. 닫히면 유지                                                          [GRIPPER_CLOSE → GRASPED]
              --grasp.exit_when_grasped=true 면 여기서 종료 (팔은 큐브를 문 채 남는다).
    8. 집기 판별 (--check.*, grasp_check.py): GRASPED 뒤 매 프레임
         front 뷰 큐브 박스의 바로 왼쪽/오른쪽 띠(--check.band_px)에 보라색 그리퍼가 있고
         **동시에** wrist 뷰 큐브 박스의 왼쪽/오른쪽 변에도 보라색이 있으면 그 프레임 OK.
         OK 가 --check.confirm_frames 연속이면 GRASP_OK, --check.timeout_s 안에 못 채우면 GRASP_FAIL.
         보라색은 HSV 범위(--check.hue_min/hue_max/sat_min/val_min, 기본 그리퍼 색 측정값)로 잡는다.
         결과에 따라 종료하려면 --check.exit_on_result=true.
         GRASP_FAIL 이면 집기 재시도 (--grasp.max_retries, 기본 5): 그리퍼를 다시 벌리고 손목 뷰 목표 폭을
         --grasp.retry_size_step_px 키워(더 가까이) 다시 내려간 뒤 닫고 다시 판별한다. 맞을 때까지 반복.
         5번 다 실패하면 그리퍼를 벌리고 시작 자세로 돌아가(--grasp.pick_retry_wait_s=3초 대기) 접근부터
         다시 한다. pick 시도는 --grasp.max_pick_attempts(기본 5)까지, 그것도 다 쓰면 멈춘다 [GIVE_UP].
    10. 마무리: GRASP_OK 면 큐브를 문 채 --grasp.carry_time_s(3초) 동안 시작 자세로 돌아간다 [CARRY_HOME → DONE].
       이것이 task 의 끝. 기본은 그 자세로 유지하며 계속 돌고, --grasp.exit_when_done=true 면 종료한다.
    9. 높이 힌트 (--grasp.front_hint): 손목 서보 중 front 뷰 **정중앙** 창(--grasp.front_hint_win_px)에 보라색
       그리퍼가 보이면(비율 ≥ --grasp.front_hint_min_ratio) 딱 알맞게 내려온 높이 → 가로만 맞으면 바로 READY.
       첫 시도에서만 쓰고, 재시도에서는 무시하고 키운 목표 폭까지 더 내려간다.
       큐브를 놓치면 그 자리에서 멈춘다 [WRIST_LOST]. 이 단계 전체를 끄려면 --grasp.enabled=false.

자세 저장 (리더암/손으로 팔을 원하는 자세로 만든 뒤):

    python lekiwi_save_pose.py --name pre_pick     # 6. 의 pick 직전 자세
    python lekiwi_save_pose.py --name grasp        # 7-3. 의 뻗는 방향 끝점 (큐브를 집는 순간의 자세)

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
    S     : 지금 손목 뷰의 박스-화면중심 관계를 집기 참조(poses/grasp_ref.json)로 저장
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
from grasp_check import GraspChecker, GraspCheckArgs, center_purple_ratio, draw_grasp_check
from wrist_servo import GRIPPER_JOINT, GraspArgs, WristServo, filter_wrist_dets, save_reference
from wrist_servo import largest as wrist_largest

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
    "GRIPPER_OPEN": (255, 255, 0),
    "WRIST_CENTERING": (0, 165, 255),
    "WRIST_APPROACH": (255, 200, 0),
    "WRIST_LOST": (0, 0, 255),
    "WRIST_REFINE": (0, 200, 255),
    "GRASP_READY": (0, 255, 0),
    "GRIPPER_CLOSE": (255, 255, 0),
    "GRASPED": (0, 255, 0),
    "GRASP_CHECK": (0, 200, 255),
    "HOME_WAIT": (200, 200, 200),
    "GIVE_UP": (0, 0, 255),
    "CARRY_HOME": (0, 255, 0),
    "DONE": (0, 255, 0),
    "GRASP_OK": (0, 255, 0),
    "GRASP_FAIL": (0, 0, 255),
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
    grasp: GraspArgs = field(default_factory=GraspArgs)
    check: GraspCheckArgs = field(default_factory=GraspCheckArgs)

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
        if self.grasp.enabled and self.pick.enabled:
            self.grasp.validate()
            if self.grasp.view not in self.views:
                self.views = [*self.views, self.grasp.view]
            if self.check.enabled:
                self.check.validate()
                for v in (self.check.front_view, self.check.wrist_view):
                    if v not in self.views:
                        self.views = [*self.views, v]
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
    """접근 완료 후 팔을 pick 자세로 보내고, 이어서 그리퍼를 열고 손목 뷰 서보로 큐브에 다가가는 상태기.

    상태:
        HOME(시작 자세 유지) → TO_PICK(보간) → PICK(pick 자세, 잠깐 대기)
            → OPEN_GRIPPER(그리퍼 최대 열기) → SERVO(손목 뷰 보며 뻗기, WristServo) → GRASP_READY(정지·유지)
        PICK 에서 큐브가 벗어나면 TO_HOME(보간 복귀) → HOME 으로 돌아가 다시 접근한다.
        --grasp.enabled=false 면 PICK 에서 멈춘다.
    팔이 HOME 이 아닐 때는 바퀴를 움직이면 안 된다 (`base_locked`).
    """

    def __init__(
        self,
        home: dict[str, float],
        pick: dict[str, float] | None,
        cfg: PickArgs,
        grasp_cfg: GraspArgs,
        grasp_pose: dict[str, float] | None = None,
    ):
        self.cfg = cfg
        self.grasp_cfg = grasp_cfg
        self.grasp_pose = dict(grasp_pose) if grasp_pose else None
        self.home = dict(home)
        self.pick = dict(pick) if pick else None
        self.state = "HOME"
        self.current = dict(home)  # 지금 보내고 있는 자세
        self._from: dict[str, float] = {}
        self._to: dict[str, float] = {}
        self._t0 = 0.0
        self._dur = 1.0
        self._pick_t = 0.0
        self.progress = 0.0
        self.drift = 0
        self.servo: WristServo | None = None
        self.just_ready = False  # 이번 프레임에 PICK 에 도달했는지
        self.grasp_just_ready = False  # 이번 프레임에 GRASP_READY 가 됐는지
        self.just_grasped = False  # 이번 프레임에 그리퍼가 다 닫혔는지 (GRASPED)
        self._ready_t = 0.0
        self.retries = 0  # 이번 pick 시도 안에서 GRIP FAIL 뒤 집기를 재시도한 횟수
        self.pick_attempts = 0  # pick 자세로 내려간 횟수 (첫 시도 포함)
        self._base_target_size = grasp_cfg.target_size_px  # 재시도로 키운 목표 크기를 새 pick 시도에서 되돌리기 위해
        self._after_open = "SERVO"  # OPEN_GRIPPER 가 끝나면 갈 곳: SERVO(집기) / HOME(포기하고 복귀)
        self._home_wait_t0 = 0.0
        self.gave_up = False  # pick 시도를 다 써서 포기했는지
        self.just_done = False  # 이번 프레임에 큐브를 문 채 시작 자세에 도착했는지

    @property
    def can_retry(self) -> bool:
        return self.state == "GRASPED" and self.retries < self.grasp_cfg.max_retries

    @property
    def can_restart_pick(self) -> bool:
        return self.state == "GRASPED" and self.pick_attempts < self.grasp_cfg.max_pick_attempts

    def retry(self, now: float) -> None:
        """GRIP FAIL → 그리퍼를 다시 벌리고(OPEN_GRIPPER), 끝나면 서보를 더 깊이 이어간다."""
        self.retries += 1
        self._after_open = "SERVO"
        target = dict(self.current)
        target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
        self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)

    def restart_pick(self, now: float) -> None:
        """집기 재시도를 다 썼다 → 그리퍼를 벌리고 시작 자세로 돌아가 잠깐 기다린 뒤 접근부터 다시."""
        self._after_open = "HOME"
        target = dict(self.current)
        target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
        self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)

    def give_up(self) -> None:
        """pick 시도까지 다 썼다 → 그 자리에서 멈춘다 (더는 움직이지 않음)."""
        self.gave_up = True

    def carry_home(self, now: float) -> None:
        """집기 성공 → 큐브를 문 채(그리퍼 그대로) 시작 자세로 천천히 돌아간다. 도착하면 DONE."""
        target = dict(self.home)
        target[GRIPPER_JOINT] = self.current.get(GRIPPER_JOINT, target.get(GRIPPER_JOINT, 0.0))  # 그리퍼는 닫힌 채
        self._start_move(target, "CARRY_HOME", now, self.grasp_cfg.carry_time_s)

    @property
    def done(self) -> bool:
        return self.state == "DONE"

    @property
    def enabled(self) -> bool:
        return self.cfg.enabled and self.pick is not None

    @property
    def grasp_enabled(self) -> bool:
        return self.enabled and self.grasp_cfg.enabled

    @property
    def base_locked(self) -> bool:
        return self.state != "HOME"

    def _start_move(self, target: dict[str, float], state: str, now: float, duration_s: float) -> None:
        self._from = dict(self.current)
        self._to = {k: target.get(k, self.current[k]) for k in self.current}
        self._t0 = now
        self._dur = max(duration_s, 1e-3)
        self.progress = 0.0
        self.state = state

    def _interpolate(self, now: float) -> float:
        a = min(1.0, (now - self._t0) / self._dur)
        self.progress = a
        self.current = {k: self._from[k] + (self._to[k] - self._from[k]) * a for k in self.current}
        return a

    def update(
        self,
        aligned: bool,
        tracking_ok: bool,
        allow_motion: bool,
        now: float,
        wrist_dets: list[Detection] | None = None,
        wrist_shape: tuple[int, ...] | None = None,
        dt: float = 1 / 30,
        descend_hint: bool = False,
    ) -> dict[str, float]:
        """이번 프레임에 보낼 팔 목표 자세를 돌려준다.

        aligned      : Approacher 가 ALIGNED 인지
        tracking_ok  : 큐브가 여전히 정렬 허용치 안에 있는지 (size_ok and center_ok)
        allow_motion : 일시정지/dry-run 이 아닌지
        wrist_dets   : 손목 뷰 검출 (SERVO 단계에서 사용)
        """
        self.just_ready = False
        self.grasp_just_ready = False
        self.just_grasped = False
        self.just_done = False
        if not self.enabled:
            return self.current

        if self.gave_up or self.state == "DONE":
            return self.current

        if self.state == "CARRY_HOME":
            if self._interpolate(now) >= 1.0:
                self.state = "DONE"
                self.just_done = True
            return self.current

        if self.state == "HOME":
            if aligned and allow_motion:
                # 새 pick 시도: 집기 재시도 카운터와 키워 둔 목표 크기를 원래대로
                self.pick_attempts += 1
                self.retries = 0
                self.servo = None
                self.grasp_cfg.target_size_px = self._base_target_size
                self._start_move(self.pick, "TO_PICK", now, self.cfg.move_time_s)
        elif self.state in ("TO_PICK", "TO_HOME"):
            if self._interpolate(now) >= 1.0:
                if self.state == "TO_PICK":
                    self.state = "PICK"
                    self.drift = 0
                    self.just_ready = True
                    self._pick_t = now
                elif self._after_open == "HOME":
                    # 포기 후 복귀: 잠깐 기다렸다가 다시 접근한다
                    self._after_open = "SERVO"
                    self._home_wait_t0 = now
                    self.state = "HOME_WAIT"
                else:
                    self.state = "HOME"
        elif self.state == "HOME_WAIT":
            if now - self._home_wait_t0 >= self.grasp_cfg.pick_retry_wait_s:
                self.state = "HOME"
        elif self.state == "PICK":
            if self.grasp_enabled and allow_motion and now - self._pick_t >= self.grasp_cfg.pick_dwell_s:
                # 다음 단계: 그리퍼를 최대로 연다 (정규화 100 = 캘리브레이션 range_max)
                target = dict(self.current)
                target[GRIPPER_JOINT] = self.grasp_cfg.gripper_open_pct
                self._start_move(target, "OPEN_GRIPPER", now, self.grasp_cfg.gripper_open_time_s)
            elif self.cfg.drift_frames > 0:
                self.drift = 0 if tracking_ok else self.drift + 1
                if self.drift >= self.cfg.drift_frames and allow_motion:
                    self._start_move(self.home, "TO_HOME", now, self.cfg.move_time_s)
        elif self.state == "OPEN_GRIPPER":
            if self._interpolate(now) >= 1.0:
                if self._after_open == "HOME":
                    self._start_move(self.home, "TO_HOME", now, self.cfg.move_time_s)
                elif self.servo is None:
                    self.servo = WristServo(self.grasp_cfg, self.current, self.grasp_pose)
                    self.state = "SERVO"
                else:
                    self.servo.resume()  # 재시도: 같은 자리에서 목표를 키워 이어간다
                    self.state = "SERVO"
        elif self.state == "SERVO":
            if wrist_dets is not None and wrist_shape is not None:
                self.current = self.servo.update(wrist_dets, wrist_shape, dt, now, allow_motion, descend_hint)
                if self.servo.just_ready:
                    self.state = "GRASP_READY"
                    self.grasp_just_ready = True
                    self._ready_t = now
        elif self.state == "GRASP_READY":
            # 집기: 잠깐 멈춘 뒤 그리퍼를 측정해 둔 값까지 닫는다
            g = self.grasp_cfg
            if g.close_after_ready and allow_motion and now - self._ready_t >= g.grasp_dwell_s:
                target = dict(self.current)
                target[GRIPPER_JOINT] = g.gripper_close_pct if g.gripper_close_pct is not None else g.resolve_close_pct()
                self._start_move(target, "CLOSE_GRIPPER", now, g.gripper_close_time_s)
        elif self.state == "CLOSE_GRIPPER":
            if self._interpolate(now) >= 1.0:
                self.state = "GRASPED"
                self.just_grasped = True
        # GRASPED: 그대로 유지 (큐브를 물고 있음)
        return self.current

    @property
    def label(self) -> str | None:
        if self.gave_up:
            return "GIVE_UP"
        if self.state == "SERVO" and self.servo is not None:
            return {
                "CENTERING": "WRIST_CENTERING",
                "APPROACHING": "WRIST_APPROACH",
                "LOST": "WRIST_LOST",
                "REFINING": "WRIST_REFINE",
                "READY": "GRASP_READY",
            }.get(self.servo.state)
        return {
            "TO_PICK": "ARM_TO_PICK",
            "PICK": "PICK_READY",
            "TO_HOME": "ARM_TO_HOME",
            "OPEN_GRIPPER": "GRIPPER_OPEN",
            "GRASP_READY": "GRASP_READY",
            "CLOSE_GRIPPER": "GRIPPER_CLOSE",
            "GRASPED": "GRASPED",
            "HOME_WAIT": "HOME_WAIT",
            "CARRY_HOME": "CARRY_HOME",
            "DONE": "DONE",
        }.get(self.state)


def draw_wrist_servo(frame_bgr: np.ndarray, arm: ArmSequencer, state_label: str) -> np.ndarray:
    """손목 뷰 오버레이: 화면 중앙점, 목표 박스, 목표 크기 참조, 서보 상태."""
    canvas = frame_bgr
    h, w = canvas.shape[:2]
    cx0, cy0 = w // 2, h // 2
    servo = arm.servo
    active = arm.state in ("SERVO", "GRASP_READY") and servo is not None

    # 화면 중앙점: 서보 중엔 박스 안/밖에 따라 초록/빨강, 아니면 흰색
    if active and servo.target is not None:
        color = (0, 220, 0) if servo.inside else (0, 0, 255)
    else:
        color = (255, 255, 255)
    cv2.circle(canvas, (cx0, cy0), 7, color, 2, cv2.LINE_AA)
    cv2.circle(canvas, (cx0, cy0), 2, color, -1, cv2.LINE_AA)

    if active and servo.target is not None:
        x1, y1, x2, y2 = servo.target.xyxy
        bx, by = servo.target.center
        cv2.rectangle(canvas, (x1, y1), (x2, y2), TARGET_COLOR, 3)
        # 세로선에 맞추는 기준 변(왼쪽/오른쪽) 또는 중심선을 두껍게 표시하고, 맞지 않으면 화살표
        ax, ay = servo.anchor_x, servo.anchor_y
        edge_color = (0, 220, 0) if servo.x_ok else TARGET_COLOR
        cv2.line(canvas, (ax, y1 - 8), (ax, y2 + 8), edge_color, 4)
        if servo.cfg.y_anchor != "inside":
            # 세로 기준(위 변/중심)도 두껍게
            cv2.line(canvas, (x1 - 8, ay), (x2 + 8, ay), (0, 220, 0) if servo.y_ok else TARGET_COLOR, 4)
        # 참조 목표점(노란 십자): 기준점이 여기 와야 한다
        tx, ty = cx0 + servo.cfg.x_target_dx, cy0 + (servo.cfg.y_target_dy if servo.cfg.y_anchor != "inside" else 0)
        cv2.drawMarker(canvas, (tx, ty), SIZE_REF_COLOR, cv2.MARKER_CROSS, 24, 2, cv2.LINE_AA)
        if not (servo.x_ok and servo.y_ok):
            cv2.arrowedLine(canvas, (tx, ty), (ax, ay if servo.cfg.y_anchor != "inside" else by), TARGET_COLOR, 2, tipLength=0.2)
        # 목표 크기 참조(노랑): 박스 중심에 target_size 정사각형
        ts = servo.cfg.target_size_px
        cv2.rectangle(canvas, (bx - ts // 2, by - ts // 2), (bx + ts // 2, by + ts // 2), SIZE_REF_COLOR, 1, cv2.LINE_AA)
        info = (
            f"{servo.cfg.size_metric}={servo.size}/{ts}px  dx={servo.dx:+d} dy={servo.dy:+d}  "
            f"{servo.cfg.x_anchor}-x {'OK' if servo.x_ok else '..'} y {'OK' if servo.y_ok else '..'}"
        )
        # 검출 라벨(박스 위)과 겹치지 않게 박스 아래에, 화면 아래로 나가면 박스 위 라벨보다 더 위에
        ty = y2 + 22 if y2 + 22 < h - 30 else max(y1 - 30, 40)
        cv2.putText(canvas, info, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TARGET_COLOR, 2, cv2.LINE_AA)

    if arm.state in ("OPEN_GRIPPER", "SERVO", "GRASP_READY", "CLOSE_GRIPPER", "GRASPED", "CARRY_HOME", "DONE"):
        col = STATE_COLORS.get(state_label, (255, 255, 255))
        if arm.state in ("OPEN_GRIPPER", "CLOSE_GRIPPER", "CARRY_HOME"):
            txt = f"{state_label}  {arm.progress * 100:3.0f}%  gripper {arm.current.get(GRIPPER_JOINT, 0):.0f}%"
        elif arm.state in ("GRASPED", "DONE"):
            txt = f"{state_label}  gripper {arm.current.get(GRIPPER_JOINT, 0):.1f}%"
        else:
            txt = (
                f"{state_label}  reach {servo.progress_pct:3.0f}%  "
                f"pan{servo.pan_delta:+.1f} tilt{servo.tilt_delta:+.1f} deg"
            )
        cv2.rectangle(canvas, (0, h - 26), (w, h), (0, 0, 0), -1)
        cv2.putText(canvas, txt, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
    return canvas


def control_loop(
    robot: LeKiwiClient,
    model,
    views: list[str],
    arm_hold: dict,
    pick_pose: dict[str, float] | None,
    grasp_pose: dict[str, float] | None,
    cfg: LeKiwiPickConfig,
) -> None:
    window = "LeKiwi YOLO pick (SPACE 일시정지, Q/ESC 종료)"
    ap = Approacher(cfg.approach)
    arm = ArmSequencer(arm_hold, pick_pose, cfg.pick, cfg.grasp, grasp_pose)
    checker = GraspChecker(cfg.check)
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
        # 손목 뷰: 왼쪽(그리퍼) 쪽은 conf 를 빡빡하게, 그리고 가장 큰 박스 하나만 (표시·서보 모두 이 결과 사용)
        if cfg.grasp.enabled and cfg.grasp.view in frames_bgr and cfg.grasp.view != cfg.approach.view:
            dets_by_view[cfg.grasp.view] = filter_wrist_dets(
                dets_by_view.get(cfg.grasp.view, []), frames_bgr[cfg.grasp.view].shape, cfg.grasp
            )
        cmd = ap.update(dets_by_view[cfg.approach.view], frames_bgr[cfg.approach.view].shape, loop_start)

        allow_motion = not paused and not cfg.dry_run
        tracking_ok = ap.target is not None and ap.size_ok and ap.center_ok
        wrist_frame = frames_bgr.get(cfg.grasp.view)
        # 높이 힌트: front 정중앙 창에 보라색(그리퍼)이 보이면 알맞게 내려온 것 (서보 중에만 계산)
        front_hint_ratio = 0.0
        if cfg.grasp.front_hint and arm.state == "SERVO" and cfg.approach.view in frames_bgr:
            front_hint_ratio = center_purple_ratio(frames_bgr[cfg.approach.view], cfg.check, cfg.grasp.front_hint_win_px)
        descend_hint = front_hint_ratio >= cfg.grasp.front_hint_min_ratio
        arm_pose = arm.update(
            ap.done,
            tracking_ok,
            allow_motion,
            loop_start,
            wrist_dets=dets_by_view.get(cfg.grasp.view) if wrist_frame is not None else None,
            wrist_shape=wrist_frame.shape if wrist_frame is not None else None,
            dt=(1.0 / hz) if hz > 0 else interval,
            descend_hint=descend_hint,
        )

        # 집기 판별: 그리퍼가 다 닫힌(GRASPED) 뒤부터 두 뷰의 박스 좌/우에 보라색 그리퍼가 있는지 본다
        if cfg.check.enabled and arm.grasp_enabled and arm.state == "GRASPED":
            if checker.state == "IDLE":
                checker.start(loop_start)
            checker.update(frames_bgr, dets_by_view, loop_start)
        check_label = {"CHECKING": "GRASP_CHECK", "SUCCESS": "GRASP_OK", "FAIL": "GRASP_FAIL"}.get(checker.state)

        if paused:
            state_label = "PAUSED"
            sent = dict(STOP)
        elif cfg.dry_run:
            state_label = "DRY_RUN"
            sent = dict(STOP)
        elif arm.base_locked:
            # 팔이 움직이는 중이거나 pick 자세 → 바퀴 고정
            state_label = check_label or arm.label or ap.state
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
        if arm.grasp_just_ready and arm.servo is not None:
            print(
                f"\n집기 준비 완료: 손목 뷰 {cfg.grasp.size_metric}={arm.servo.size}px, "
                f"dx={arm.servo.dx:+d} dy={arm.servo.dy:+d}, reach {arm.servo.progress_pct:.0f}%, "
                f"보정 pan {arm.servo.pan_delta:+.1f} / tilt {arm.servo.tilt_delta:+.1f} deg"
                + (
                    f"  (가로/세로가 허용치 밖이지만 {cfg.grasp.refine_timeout_s}s 지나 그대로 진행)"
                    if arm.servo.ready_forced
                    else "  (front 정중앙 보라색 힌트로 높이 확정)"
                    if arm.servo.ready_by_hint
                    else ""
                )
                + (f"  [재시도 {arm.retries}]" if arm.retries else "")
            )
            if cfg.grasp.exit_when_ready:
                print("--grasp.exit_when_ready=true 이므로 종료합니다. 팔은 그 자세로 남습니다.")
                return
        if arm.just_grasped:
            print(f"\n집기 완료: 그리퍼 {arm.current.get(GRIPPER_JOINT, float('nan')):.1f}% 로 닫힘. 자세를 유지합니다.")
            if cfg.grasp.exit_when_grasped:
                print("--grasp.exit_when_grasped=true 이므로 종료합니다. 팔은 큐브를 문 채 남습니다.")
                return
        if checker.just_decided:
            verdict = "제대로 집었습니다 ✓" if checker.state == "SUCCESS" else "집기 실패로 판정 ✗ (양쪽 뷰에서 박스 좌/우에 그리퍼가 안 보임)"
            print(f"\n집기 판별: {verdict}  [{checker.summary()}]")
            if checker.state == "FAIL" and allow_motion and arm.can_retry:
                # 집기 재시도: 그리퍼를 다시 벌리고, 손목 뷰 박스가 더 커지도록(더 가까이) 내려가 다시 집는다
                arm.retry(loop_start)
                checker.reset()
                print(
                    f"집기 재시도 {arm.retries}/{cfg.grasp.max_retries} (pick 시도 {arm.pick_attempts}/{cfg.grasp.max_pick_attempts}): "
                    f"그리퍼를 벌리고 목표 폭을 {cfg.grasp.target_size_px + cfg.grasp.retry_size_step_px}px 로 키워 더 내려갑니다."
                )
            elif checker.state == "FAIL" and allow_motion and arm.can_restart_pick:
                # 집기 재시도를 다 썼다 → 시작 자세로 돌아가 잠깐 기다린 뒤 접근부터 다시
                arm.restart_pick(loop_start)
                checker.reset()
                print(
                    f"집기 재시도 {cfg.grasp.max_retries}회 모두 실패. 그리퍼를 벌리고 시작 자세로 돌아가 "
                    f"{cfg.grasp.pick_retry_wait_s}s 뒤 다시 접근합니다 (pick 시도 {arm.pick_attempts + 1}/{cfg.grasp.max_pick_attempts})."
                )
            elif cfg.check.exit_on_result:
                print("--check.exit_on_result=true 이므로 종료합니다.")
                return
            elif checker.state == "FAIL":
                arm.give_up()
                print(
                    f"pick 시도 {cfg.grasp.max_pick_attempts}회 × 집기 재시도 {cfg.grasp.max_retries}회 모두 실패. "
                    "그 자리에서 멈춥니다 (SPACE/ESC 로 처리하세요)."
                )
        # 집기 성공 → 큐브를 문 채 시작 자세로 (일시정지 중이면 풀릴 때 시작)
        if checker.state == "SUCCESS" and arm.state == "GRASPED" and cfg.grasp.return_home_when_grasped and allow_motion:
            arm.carry_home(loop_start)
            print(f"큐브를 문 채 {cfg.grasp.carry_time_s}s 동안 시작 자세로 돌아갑니다.")
        if arm.just_done:
            print(f"\n=== TASK 완료: 큐브를 집어 시작 자세로 돌아왔습니다 (pick 시도 {arm.pick_attempts}, 집기 재시도 {arm.retries}) ===")
            if cfg.grasp.exit_when_done:
                print("--grasp.exit_when_done=true 이므로 종료합니다. 팔은 큐브를 문 채 시작 자세로 남습니다.")
                return
        if arm.state not in ("GRASPED", "CARRY_HOME", "DONE") and checker.state != "IDLE":
            checker.reset()  # 새 시도로 넘어갔으면 판별기도 초기화

        if cfg.display == "cv2":
            panels = []
            for v in views:
                if v not in frames_bgr:
                    continue
                img = draw(frames_bgr[v], v, dets_by_view.get(v, []), hz, crosshair=v in cfg.crosshair_views)
                if v == cfg.approach.view:
                    img = draw_alignment(img, ap, cmd, state_label)
                    if cfg.grasp.front_hint and arm.state in ("SERVO", "GRASP_READY"):
                        # 높이 힌트 창: 보라색 비율이 문턱 이상이면 초록, 아니면 노랑
                        hh, ww = img.shape[:2]
                        r = cfg.grasp.front_hint_win_px // 2
                        hc = (0, 220, 0) if descend_hint else (0, 200, 255)
                        cv2.rectangle(img, (ww // 2 - r, hh // 2 - r), (ww // 2 + r, hh // 2 + r), hc, 2)
                        cv2.putText(img, f"hint {front_hint_ratio:.2f}", (ww // 2 + r + 4, hh // 2 - r - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, hc, 1, cv2.LINE_AA)
                if v == cfg.grasp.view and arm.grasp_enabled:
                    img = draw_wrist_servo(img, arm, state_label)
                if cfg.check.enabled and v in (cfg.check.front_view, cfg.check.wrist_view):
                    img = draw_grasp_check(img, checker, v)
                panels.append(img)
            cv2.imshow(window, hstack_views(panels, cfg.view_height))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("\n종료합니다...")
                return
            if key == ord(" "):
                paused = not paused
                print("\n일시정지" if paused else "\n재개")
            if key == ord("s") and cfg.grasp.enabled:
                # 지금 손목 뷰의 박스-화면중심 관계를 '집기 직전 참조'로 저장 (다음 실행부터 이 구성을 재현)
                wdets = dets_by_view.get(cfg.grasp.view) or []
                if wrist_frame is None or not wdets:
                    print("\n[S] 손목 뷰에 검출이 없어 참조를 저장하지 않았습니다.")
                else:
                    ref = save_reference(cfg.grasp.ref_file, wrist_largest(wdets), wrist_frame.shape, cfg.grasp)
                    print(
                        f"\n[S] 집기 참조 저장: {cfg.grasp.ref_file}  "
                        f"{ref['x_anchor']}-dx={ref['x_target_dx']:+d} {ref['y_anchor']}-dy={ref['y_target_dy']:+d} "
                        f"{ref['size_metric']}={ref['target_size_px']}px  (다음 실행부터 적용)"
                    )
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
            elif arm.state == "OPEN_GRIPPER":
                line += f"| gripper → {cfg.grasp.gripper_open_pct:.0f}% {arm.progress * 100:3.0f}% "
            elif arm.state == "CLOSE_GRIPPER":
                line += f"| gripper → {cfg.grasp.gripper_close_pct:.1f}% {arm.progress * 100:3.0f}% "
            elif arm.state == "CARRY_HOME":
                line += f"| 큐브 운반 → 시작 자세 {arm.progress * 100:3.0f}% "
            elif arm.state == "DONE":
                line += "| TASK DONE (큐브를 문 채 시작 자세) "
            elif arm.state == "HOME_WAIT":
                line += f"| 시작 자세 대기 {cfg.grasp.pick_retry_wait_s - (loop_start - arm._home_wait_t0):.1f}s 후 재접근 (pick {arm.pick_attempts}/{cfg.grasp.max_pick_attempts}) "
            elif arm.state == "GRASPED":
                line += f"| GRASPED gripper {arm.current.get(GRIPPER_JOINT, 0):.1f}% "
                if checker.state != "IDLE":
                    line += f"| check {checker.state} {checker.summary()} streak {checker.streak}/{cfg.check.confirm_frames} "
            elif arm.state in ("SERVO", "GRASP_READY") and arm.servo is not None:
                s = arm.servo
                tgt = f"{s.cfg.size_metric}={s.size}/{s.cfg.target_size_px} {s.cfg.x_anchor}-dx={s.dx:+d}{'✓' if s.x_ok else ''} dy={s.dy:+d}{'✓' if s.y_ok else ''}" if s.target else "target -"
                line += f"| wrist {s.state} {tgt} reach {s.progress_pct:3.0f}% pan{s.pan_delta:+.1f} tilt{s.tilt_delta:+.1f} "
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

        grasp_pose = None
        if cfg.pick.enabled and cfg.grasp.enabled and cfg.grasp.approach_mode == "pose":
            grasp_pose = {
                k: v for k, v in load_pose(Path(cfg.grasp.grasp_pose_file).expanduser()).items() if k in arm_hold
            }
            logging.info("grasp 자세 (%s): %s", cfg.grasp.grasp_pose_file, {k: round(v, 1) for k, v in grasp_pose.items()})

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
            + (
                f"\n집기: pick 자세 {cfg.grasp.pick_dwell_s}s 후 그리퍼 {cfg.grasp.gripper_open_pct:.0f}% 열기 → "
                f"손목 뷰({cfg.grasp.view}) 박스 {cfg.grasp.x_anchor} 변을 세로선에(±{cfg.grasp.x_tolerance_px}px) 맞추게 "
                f"{cfg.grasp.pan_joint.removesuffix('.pos')}/"
                f"{cfg.grasp.tilt_joint.removesuffix('.pos')} 보정 → "
                + (
                    f"grasp 자세({Path(cfg.grasp.grasp_pose_file).name}) 방향으로 뻗기"
                    if cfg.grasp.approach_mode == "pose"
                    else f"관절 속도 {cfg.grasp.reach_joints} 로 뻗기"
                )
                + f" → {cfg.grasp.size_metric} {cfg.grasp.target_size_px}px 에서 정지"
                + (
                    f" → 그리퍼 {cfg.grasp.gripper_close_pct:.1f}% 로 닫기"
                    if cfg.grasp.close_after_ready and cfg.grasp.gripper_close_pct is not None
                    else " (닫기 없음)"
                )
                if cfg.pick.enabled and cfg.grasp.enabled
                else "\n집기: 없음 (--grasp.enabled=false)"
            )
            + "\n조작: SPACE 일시정지/재개, Q/ESC 종료, Ctrl+C 종료\n"
        )
        if cfg.start_paused:
            print("일시정지 상태로 시작합니다. SPACE 를 누르면 움직입니다.")

        try:
            control_loop(robot, model, views, arm_hold, pick_pose, grasp_pose, cfg)
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
