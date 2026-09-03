r"""손목 카메라로 큐브를 보며 팔을 조금씩 뻗는 비주얼 서보 (lekiwi_yolo_pick.py 의 pick 자세 이후 단계).

흐름 (pick 자세에 도달한 뒤):
    1. 그리퍼를 최대로 연다 (`gripper_open_pct`, 정규화 0~100 의 100 = 캘리브레이션 range_max).
    2. 손목 뷰에서 큐브 박스의 **왼쪽 변**(`x_anchor=left`)이 화면 **가운데 세로선**에 오고,
       화면 중앙 높이가 박스의 위/아래 변 사이에 들도록 관절을 조정한다.
           가로 오차(dx = 기준변 x − 화면 중심 x) → shoulder_pan,  세로 오차(dy) → wrist_flex
           (관절/부호/게인 모두 옵션)
       LeKiwi 손목 카메라는 그리퍼 왼쪽에 달려 있어서, 큐브가 화면 오른쪽 절반에 있어야 그리퍼 사이에
       들어온다. 그래서 박스 중심이 아니라 왼쪽 변을 세로선에 맞춘다. (`x_anchor=center|right` 로 바꿀 수 있다.)
       맞지 않으면 조정만 하고, 맞으면(±`x_tolerance_px`) 조정을 멈춘다.
    3. 중앙점이 박스 안에 있는 동안만 팔을 큐브 쪽으로 **조금씩 뻗는다** (박스가 점점 커진다).
           approach_mode=pose  : pick 자세 → 저장해 둔 grasp 자세(`grasp_pose_file`) 방향으로 보간해 나간다 (권장)
           approach_mode=joints: `reach_joints` 에 적은 관절별 deg/s 만큼 누적한다 (부호를 직접 맞춰야 함)
    4. 박스 폭이 `target_size_px` 이상이고 가로/세로 기준이 모두 맞으면 READY — 팔을 멈추고 유지한다.
       크기는 됐는데 기준이 아직 안 맞으면 REFINING: 더 뻗지 않고 pan/tilt 보정만 계속한다.
    3-1. 높이 힌트 (`front_hint`): front 뷰 정중앙 창에 보라색(그리퍼)이 보이면 딱 알맞게 내려온 높이다.
       첫 시도에서 이 힌트가 켜지고 가로(x)만 맞으면 크기와 무관하게 READY.
    3-2. GRIP FAIL 재시도 (`max_retries`, 기본 5): 그리퍼를 다시 벌리고, 목표 크기를 `retry_size_step_px` 키우고
       경로를 `retry_overreach` 만큼 더 갈 수 있게 한 뒤 같은 자리에서 서보를 이어간다 (힌트는 무시).
       다시 READY → 닫기 → 판별. 판별이 맞을 때까지 반복한다.
    3-3. 재시도를 다 써도 실패하면 (`max_pick_attempts`, 기본 5): 그리퍼를 벌리고 시작 자세로 돌아가
       `pick_retry_wait_s`(3초) 기다린 뒤 접근 → pick 자세 → 집기를 처음부터 다시 한다 (목표 크기도 원래대로).
       pick 시도도 다 쓰면 시작 자세에서 멈춘다.
    5. READY 후 `grasp_dwell_s` 지나면 그리퍼를 닫는다 (`close_after_ready`). 목표는 큐브를 물고 있을 때
       저장한 `close_pose_file`(poses/grasp_closed.json) 의 arm_gripper.pos (측정값 15.1), 또는
       `gripper_close_pct` 로 직접. `gripper_close_extra_pct` 만큼 더 조일 수 있다. 닫히면 GRASPED 로 유지.
    6. 집기 판별(grasp_check.py)이 SUCCESS 면 큐브를 문 채 `carry_time_s` 동안 시작 자세로 돌아간다 → DONE.
       (`return_home_when_grasped`). 이것이 task 의 끝이다. `exit_when_done` 이면 스크립트를 끝낸다.

    참조 구성 (`ref_file`, 기본 poses/grasp_ref.json): 집기 직전 '완벽한' 화면에서 박스와 화면 중심의
    위치 관계(왼쪽 변 dx, 위 변 dy, 폭)를 저장한 파일. 있으면 위 목표들을 이 값으로 덮어쓴다.
    lekiwi_yolo_pick.py 서보 화면에서 **S 키**를 누르면 지금 보이는 구성이 이 파일로 저장된다.

설계 메모
    * 관절 목표 = 접근 경로 위의 점(progress) + 누적 보정(pan/tilt). 보정은 `max_correction_deg` 로 묶고,
      한 프레임에 움직일 수 있는 각도는 `max_joint_speed_dps` 로 묶어 튀지 않게 한다.
    * 큐브를 놓치면(`lost_timeout_s`) 그 자리에서 멈추고 기다린다. 뻗은 팔을 자동으로 되돌리진 않는다
      (돌아가다 부딪힐 수 있으므로). 사람이 SPACE/ESC 로 처리한다.
    * 부호(pan_sign / tilt_sign)가 틀리면 오차가 커지는 쪽으로 움직인다. 처음엔 `max_correction_deg` 를
      5~10 으로 낮게 두고 어느 쪽으로 도는지 확인한 뒤 부호를 맞춘다.
"""

import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from lekiwi_yolo_view import Detection

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_GRASP_POSE = SCRIPT_DIR / "poses" / "grasp.json"
DEFAULT_GRASP_REF = SCRIPT_DIR / "poses" / "grasp_ref.json"

GRIPPER_JOINT = "arm_gripper.pos"


@dataclass
class GraspArgs:
    """pick 자세 이후: 그리퍼 열기 → 손목 뷰 서보로 접근."""

    # false 면 pick 자세에서 멈춘다 (예전 동작)
    enabled: bool = True
    # 서보에 쓸 카메라 (호스트 --robot.cameras 이름)
    view: str = "wrist"

    # --- 0. 손목 뷰 검출 필터 (화면에도 이 결과만 그린다) ---

    # 가장 큰 박스 하나만 남긴다 (손목 뷰에는 항상 박스가 하나만 보이게)
    single_box: bool = True
    # 박스 중심 x 가 (화면 폭 × 이 비율) 보다 왼쪽이면 '그리퍼 쪽' 으로 보고 left_min_conf 이상만 인정한다.
    # 손목 카메라 왼쪽에 보이는 보라색 그리퍼 손가락이 가끔 큐브로 잡히기 때문
    left_region_ratio: float = 0.5
    left_min_conf: float = 0.85

    # --- 1. 그리퍼 ---

    # 그리퍼 목표 (정규화 0~100). 100 = 캘리브레이션에서 기록한 range_max = 최대 열림
    gripper_open_pct: float = 100.0
    # 현재 → 열림까지 보간 시간(초)
    gripper_open_time_s: float = 1.0
    # pick 자세 도달 후 그리퍼를 열기 전 잠깐 멈추는 시간(초)
    pick_dwell_s: float = 0.5

    # --- 2. 중앙 맞추기 (화면 중앙점이 박스 안에) ---

    # 박스를 이 픽셀만큼 안쪽으로 줄여서 "안에 들어왔다"를 판정한다 (0 = 박스 경계 그대로)
    inside_margin_px: int = 5
    # 화면 가운데 세로선에 맞출 박스의 기준: left(왼쪽 변, 기본) / center(중심) / right(오른쪽 변)
    #   손목 카메라가 그리퍼 왼쪽에 있어 큐브가 화면 오른쪽에 있어야 집힌다 → 왼쪽 변을 세로선에
    x_anchor: str = "left"
    # 기준 x 가 목표에서 이 픽셀 안이면 가로 정렬된 것으로 본다 (너무 빡빡하면 끝없이 보정만 하다 못 집는다)
    x_tolerance_px: int = 25
    # 기준 x 의 목표 위치 (화면 중심 x 로부터 px). 0 = 세로선 위. 참조 파일이 있으면 거기 값으로 덮인다
    x_target_dx: int = 0
    # 세로 기준: inside(화면 중앙 높이가 박스 위/아래 사이면 OK) / top(박스 위 변을 y_target_dy 에) /
    #            center(박스 중심을 y_target_dy 에)
    #   큐브가 가까워지면 박스 아래 변이 화면 밖으로 잘리므로 중심보다 위 변이 믿을 만하다
    y_anchor: str = "inside"
    # [top/center] 기준 y 의 목표 (화면 중심 y 로부터 px, 위가 음수)
    y_target_dy: int = 0
    # [top/center] 기준 y 가 목표에서 이 픽셀 안이면 세로 정렬된 것으로 본다
    y_tolerance_px: int = 40
    # 집기 직전 '완벽한' 화면 구성을 저장한 참조 파일. 있으면 x_anchor/x_target_dx/y_anchor/y_target_dy/
    # size_metric/target_size_px 를 이 파일 값으로 덮어쓴다. 서보 화면에서 S 키로 현재 구성을 저장할 수 있다
    ref_file: str = str(DEFAULT_GRASP_REF)
    # 가로 오차(dx = 기준 x - 화면 중심 x) 를 움직일 관절과 게인(deg/s per px), 부호
    pan_joint: str = "arm_shoulder_pan.pos"
    pan_gain: float = 0.15
    pan_sign: float = 1.0
    # 세로 오차(dy) 를 움직일 관절과 게인, 부호
    tilt_joint: str = "arm_wrist_flex.pos"
    tilt_gain: float = 0.15
    tilt_sign: float = 1.0
    # 보정 누적치 한계 (deg). 부호가 틀렸을 때 피해를 막는 안전장치이기도 하다
    max_correction_deg: float = 20.0

    # --- 3. 접근 (박스가 커지도록 뻗기) ---

    # pose  : pick 자세 → grasp 자세 방향으로 보간 (lekiwi_save_pose.py --name grasp 로 저장)
    # joints: reach_joints 의 관절별 deg/s 를 누적
    approach_mode: str = "pose"
    grasp_pose_file: str = str(DEFAULT_GRASP_POSE)
    # [pose] pick → grasp 전체를 가는 데 걸리는 시간(초). 중앙점이 박스 안일 때만 진행된다
    approach_time_s: float = 6.0
    # [joints] 관절별 deg/s. 예: --grasp.reach_joints='{arm_shoulder_lift.pos: 5, arm_elbow_flex.pos: -3}'
    reach_joints: dict[str, float] = field(default_factory=dict)
    # [joints] 최대 누적 시간(초) — 이 이상 뻗지 않는다
    max_reach_s: float = 6.0
    # 이 크기(px)가 되면 READY (손목 뷰 640 기준). lekiwi_yolo_view.py 로 잡기 직전 거리에서 읽어 넣는다
    target_size_px: int = 260
    size_metric: str = "width"
    # 크기가 목표 - 이 값 이상이면 "도달"로 본다
    size_tolerance_px: int = 20
    # 크기는 됐는데 가로/세로가 허용치 밖이라 REFINING 이 이 시간(초) 이상 이어지면 그냥 READY 로 넘어간다
    # (딱 그 순간을 노리다 못 집는 것보다 대충 맞으면 집는 게 낫다). 0 이면 끝까지 맞출 때까지 기다린다
    refine_timeout_s: float = 2.0

    # --- 3-1. 내려간 높이 힌트: front 뷰 정중앙에 보라색(그리퍼)이 보이면 딱 알맞은 높이 ---

    # true 면 (첫 시도에서) front 정중앙 창에 보라색이 보이고 가로가 맞으면 크기와 상관없이 READY
    front_hint: bool = True
    # 정중앙 창 크기(px) 와 그 안 보라색 비율 문턱. 색 범위는 --check.* 의 HSV 를 쓴다
    front_hint_win_px: int = 20
    front_hint_min_ratio: float = 0.5

    # --- 3-2. GRIP FAIL 재시도: 그리퍼를 다시 벌리고 더 깊이 내려가 다시 집는다 ---

    # 한 번의 pick 시도 안에서 집기(닫기→판별)를 다시 해보는 최대 횟수 (0 = 재시도 없음)
    max_retries: int = 5
    # 집기 재시도를 다 써도 실패하면: 그리퍼를 벌리고 시작 자세로 돌아가 pick_retry_wait_s 기다린 뒤
    # 접근부터 다시 한다. 전체 pick 시도는 이 횟수까지 (첫 시도 포함)
    max_pick_attempts: int = 5
    pick_retry_wait_s: float = 3.0
    # 재시도마다 손목 뷰 목표 크기를 이만큼 키운다 (더 가까이)
    retry_size_step_px: int = 30
    # [pose 모드] 재시도마다 pick→grasp 경로를 이 비율만큼 더 넘어갈 수 있게 한다 (0.15 = 15퍼센트 더 뻗음)
    # [joints 모드] max_reach_s 의 이 비율만큼 더 누적할 수 있다
    retry_overreach: float = 0.15

    # --- 4. 집기: READY 후 그리퍼 닫기 ---

    # READY 가 되면 그리퍼를 닫는다. false 면 READY(열린 채)에서 멈춘다
    close_after_ready: bool = True
    # READY 후 닫기 전 잠깐 멈추는 시간(초) — 팔이 완전히 정지하도록
    grasp_dwell_s: float = 0.3
    # 닫을 목표 (0~100). None 이면 close_pose_file 의 arm_gripper.pos 를 쓴다
    gripper_close_pct: float | None = None
    # 큐브를 물고 있는 자세 파일 (lekiwi_save_pose.py --name grasp_closed). 그리퍼 값만 쓴다
    close_pose_file: str = str(SCRIPT_DIR / "poses" / "grasp_closed.json")
    # 파일/지정값보다 이만큼 더 닫는다 (+ 면 더 조임). 측정값이 '물고 있는' 값이라 보통 0~3
    gripper_close_extra_pct: float = 0.0
    # 열림 → 닫힘 보간 시간(초)
    gripper_close_time_s: float = 1.0
    # GRASPED 가 되면 스크립트를 끝낸다 (팔은 그 자세로 남는다)
    exit_when_grasped: bool = False

    # --- 5. 마무리: 집기 성공(GRASP_OK) 이면 큐브를 문 채 시작 자세로 돌아간다 ---

    return_home_when_grasped: bool = True
    # 시작 자세로 돌아가는 데 걸리는 시간(초). 큐브를 물고 있으니 천천히
    carry_time_s: float = 3.0
    # 시작 자세에 도착(DONE)하면 스크립트를 끝낸다. false 면 그 자세로 유지하며 계속 돈다
    exit_when_done: bool = False

    # --- 공통 ---

    # 어떤 관절도 한 프레임에 이 속도(deg/s) 이상 움직이지 않는다
    max_joint_speed_dps: float = 20.0
    # 큐브를 이 시간(초) 이상 놓치면 LOST (그 자리에서 정지)
    lost_timeout_s: float = 0.5
    # READY 가 되면 스크립트를 끝낸다 (팔은 그 자세로 남는다)
    exit_when_ready: bool = False

    def load_reference(self) -> dict | None:
        """참조 파일이 있으면 읽어 목표 구성을 덮어쓴다. 읽은 dict 를 돌려준다 (없으면 None)."""
        path = Path(self.ref_file).expanduser()
        if not self.ref_file or not path.exists():
            return None
        ref = json.loads(path.read_text())
        for key in ("x_anchor", "x_target_dx", "y_anchor", "y_target_dy", "size_metric", "target_size_px"):
            if key in ref:
                setattr(self, key, type(getattr(self, key))(ref[key]))
        logging.info(
            "집기 참조 %s: %s 변→dx %+d, %s→dy %+d, %s %dpx",
            path.name, self.x_anchor, self.x_target_dx, self.y_anchor, self.y_target_dy, self.size_metric, self.target_size_px,
        )
        return ref

    def resolve_close_pct(self) -> float:
        """그리퍼 닫기 목표를 정한다: gripper_close_pct 가 있으면 그 값, 없으면 close_pose_file 의 arm_gripper.pos."""
        if self.gripper_close_pct is not None:
            base = self.gripper_close_pct
        else:
            path = Path(self.close_pose_file).expanduser()
            if not path.exists():
                raise SystemExit(
                    f"error: 닫힘 자세 파일이 없습니다: {self.close_pose_file}\n"
                    "  큐브를 물고 있는 상태에서 저장하세요:\n"
                    f"      python {SCRIPT_DIR / 'lekiwi_save_pose.py'} --name grasp_closed\n"
                    "  또는 값을 직접: --grasp.gripper_close_pct=15\n"
                    "  닫기 단계를 끄려면: --grasp.close_after_ready=false"
                )
            data = json.loads(path.read_text())
            pose = data.get("pose", data)
            if GRIPPER_JOINT not in pose:
                raise SystemExit(f"error: {self.close_pose_file} 에 {GRIPPER_JOINT} 가 없습니다")
            base = float(pose[GRIPPER_JOINT])
        pct = float(np.clip(base - self.gripper_close_extra_pct, 0.0, 100.0))
        logging.info("그리퍼 닫기 목표: %.1f%% (기준 %.1f, 추가 조임 %.1f)", pct, base, self.gripper_close_extra_pct)
        return pct

    def validate(self) -> None:
        if not self.enabled:
            return
        self.load_reference()
        if self.close_after_ready:
            if self.gripper_close_time_s <= 0 or self.grasp_dwell_s < 0:
                raise SystemExit("error: --grasp.gripper_close_time_s 는 0 보다, grasp_dwell_s 는 0 이상이어야 합니다")
            if self.gripper_close_pct is not None and not 0.0 <= self.gripper_close_pct <= 100.0:
                raise SystemExit(f"error: --grasp.gripper_close_pct 는 0~100 이어야 합니다 (받은 값: {self.gripper_close_pct})")
            self.gripper_close_pct = self.resolve_close_pct()
        if not 0.0 <= self.left_region_ratio <= 1.0 or not 0.0 <= self.left_min_conf <= 1.0:
            raise SystemExit("error: --grasp.left_region_ratio / left_min_conf 는 0~1 이어야 합니다")
        if self.y_anchor not in ("inside", "top", "center"):
            raise SystemExit(f"error: --grasp.y_anchor 는 inside/top/center 중 하나여야 합니다 (받은 값: {self.y_anchor})")
        if self.y_tolerance_px < 0 or self.size_tolerance_px < 0 or self.refine_timeout_s < 0:
            raise SystemExit("error: --grasp.y_tolerance_px / size_tolerance_px / refine_timeout_s 는 0 이상이어야 합니다")
        if self.front_hint_win_px <= 0 or not 0.0 < self.front_hint_min_ratio <= 1.0:
            raise SystemExit("error: --grasp.front_hint_win_px 는 0 보다, front_hint_min_ratio 는 0~1 사이여야 합니다")
        if self.max_retries < 0 or self.retry_size_step_px < 0 or self.retry_overreach < 0:
            raise SystemExit("error: --grasp.max_retries / retry_size_step_px / retry_overreach 는 0 이상이어야 합니다")
        if self.max_pick_attempts < 1 or self.pick_retry_wait_s < 0:
            raise SystemExit("error: --grasp.max_pick_attempts 는 1 이상, pick_retry_wait_s 는 0 이상이어야 합니다")
        if self.carry_time_s <= 0:
            raise SystemExit(f"error: --grasp.carry_time_s 는 0 보다 커야 합니다 (받은 값: {self.carry_time_s})")
        if self.approach_mode not in ("pose", "joints"):
            raise SystemExit(
                f"error: --grasp.approach_mode 는 pose/joints 중 하나여야 합니다 (받은 값: {self.approach_mode})"
            )
        if self.approach_mode == "pose" and not Path(self.grasp_pose_file).expanduser().exists():
            raise SystemExit(
                f"error: grasp 자세 파일이 없습니다: {self.grasp_pose_file}\n"
                "  리더암으로 팔을 '큐브를 집는 순간'의 자세로 만든 뒤 저장하세요:\n"
                f"      python {SCRIPT_DIR / 'lekiwi_save_pose.py'} --name grasp\n"
                "  저장 없이 관절 속도로 뻗으려면: --grasp.approach_mode=joints "
                "--grasp.reach_joints='{arm_shoulder_lift.pos: 5}'\n"
                "  이 단계를 끄려면: --grasp.enabled=false"
            )
        if self.approach_mode == "joints" and not self.reach_joints:
            raise SystemExit(
                "error: --grasp.approach_mode=joints 에는 --grasp.reach_joints 가 필요합니다 "
                "(예: --grasp.reach_joints='{arm_shoulder_lift.pos: 5}')"
            )
        if self.size_metric not in ("width", "height", "max"):
            raise SystemExit(f"error: --grasp.size_metric 은 width/height/max 중 하나여야 합니다 (받은 값: {self.size_metric})")
        for name, v in (
            ("gripper_open_time_s", self.gripper_open_time_s),
            ("approach_time_s", self.approach_time_s),
            ("max_reach_s", self.max_reach_s),
            ("max_joint_speed_dps", self.max_joint_speed_dps),
            ("target_size_px", self.target_size_px),
        ):
            if v <= 0:
                raise SystemExit(f"error: --grasp.{name} 은 0 보다 커야 합니다 (받은 값: {v})")
        if not 0.0 <= self.gripper_open_pct <= 100.0:
            raise SystemExit(f"error: --grasp.gripper_open_pct 는 0~100 이어야 합니다 (받은 값: {self.gripper_open_pct})")
        if self.max_correction_deg < 0 or self.inside_margin_px < 0 or self.pick_dwell_s < 0 or self.x_tolerance_px < 0:
            raise SystemExit(
                "error: --grasp.max_correction_deg / inside_margin_px / pick_dwell_s / x_tolerance_px 는 0 이상이어야 합니다"
            )
        if self.x_anchor not in ("left", "center", "right"):
            raise SystemExit(f"error: --grasp.x_anchor 는 left/center/right 중 하나여야 합니다 (받은 값: {self.x_anchor})")
        if abs(self.pan_sign) != 1.0 or abs(self.tilt_sign) != 1.0:
            raise SystemExit("error: --grasp.pan_sign / tilt_sign 은 1 또는 -1 이어야 합니다")


def box_size(det: Detection, metric: str) -> int:
    x1, y1, x2, y2 = det.xyxy
    w, h = x2 - x1, y2 - y1
    return w if metric == "width" else h if metric == "height" else max(w, h)


def largest(dets: list[Detection]) -> Detection | None:
    if not dets:
        return None
    return max(dets, key=lambda d: (d.xyxy[2] - d.xyxy[0]) * (d.xyxy[3] - d.xyxy[1]))


def filter_wrist_dets(dets: list[Detection], frame_shape: tuple[int, ...], cfg: GraspArgs) -> list[Detection]:
    """손목 뷰 검출 정리: 왼쪽(그리퍼 쪽) 박스는 conf 를 더 빡빡하게 보고, 가장 큰 박스 하나만 남긴다.

    화면에 그리는 것도 이 결과를 쓰므로 손목 뷰에는 박스가 최대 하나만 보인다.
    """
    _, w = frame_shape[:2]
    left_edge = w * cfg.left_region_ratio
    kept = [d for d in dets if d.center[0] >= left_edge or d.conf >= cfg.left_min_conf]
    if cfg.single_box:
        best = largest(kept)
        return [best] if best is not None else []
    return kept


class WristServo:
    """손목 뷰 서보 상태기. 매 프레임 `update()` 가 보낼 팔 자세를 돌려준다.

    상태: CENTERING(중앙점이 박스 밖, 보정 중) / APPROACHING(안에 있음, 뻗는 중) / LOST(큐브 없음, 정지)
          / READY(목표 크기 도달, 정지)
    """

    def __init__(self, cfg: GraspArgs, start_pose: dict[str, float], grasp_pose: dict[str, float] | None):
        self.cfg = cfg
        self.start = dict(start_pose)  # 그리퍼를 연 직후의 자세 (progress 0)
        # 접근 방향 벡터 (관절별 deg). pose 모드: grasp - start, joints 모드: deg/s 그대로
        if cfg.approach_mode == "pose":
            if grasp_pose is None:
                raise ValueError("approach_mode=pose 에는 grasp_pose 가 필요합니다")
            self.reach = {k: grasp_pose.get(k, v) - v for k, v in self.start.items()}
            self.reach[GRIPPER_JOINT] = 0.0  # 그리퍼는 서보가 따로 관리
            self.max_progress = 1.0
            self.progress_rate = 1.0 / cfg.approach_time_s
        else:
            self.reach = {k: float(cfg.reach_joints.get(k, 0.0)) for k in self.start}
            self.max_progress = cfg.max_reach_s
            self.progress_rate = 1.0
        self.progress = 0.0
        self.pan_delta = 0.0
        self.tilt_delta = 0.0
        self.current = dict(self.start)
        self.current[GRIPPER_JOINT] = cfg.gripper_open_pct

        self.state = "CENTERING"
        self.target: Detection | None = None
        self.size = 0
        self.anchor_x = 0  # 세로선에 맞추는 기준 x (x_anchor 에 따라 왼쪽 변/중심/오른쪽 변)
        self.anchor_y = 0  # 세로 기준 y (y_anchor 에 따라 위 변/중심)
        self.dx = 0
        self.dy = 0
        self.x_ok = False  # 기준 x 가 세로선 ±x_tolerance_px 안
        self.y_ok = False  # 화면 중앙 높이가 박스 위/아래 변 사이
        self.inside = False  # x_ok and y_ok
        self.last_seen = -float("inf")
        self._refine_since = 0.0
        self.ready_forced = False  # refine_timeout 으로 대충 맞은 채 READY 가 됐는지
        self.ready_by_hint = False  # front 정중앙 보라색 힌트로 READY 가 됐는지
        self.attempt = 0  # 0 = 첫 시도, 재시도마다 +1
        self.just_ready = False

    @property
    def done(self) -> bool:
        return self.state == "READY"

    def _compose(self) -> dict[str, float]:
        pose = {k: self.start[k] + self.reach[k] * self.progress for k in self.start}
        if self.cfg.pan_joint in pose:
            pose[self.cfg.pan_joint] += self.pan_delta
        if self.cfg.tilt_joint in pose:
            pose[self.cfg.tilt_joint] += self.tilt_delta
        pose[GRIPPER_JOINT] = self.cfg.gripper_open_pct
        return pose

    def _limit_speed(self, pose: dict[str, float], dt: float) -> dict[str, float]:
        """한 프레임 관절 이동량을 max_joint_speed_dps*dt 로 묶는다 (그리퍼 제외)."""
        step = self.cfg.max_joint_speed_dps * max(dt, 1e-3)
        out = {}
        for k, v in pose.items():
            if k == GRIPPER_JOINT:
                out[k] = v
                continue
            cur = self.current.get(k, v)
            out[k] = float(np.clip(v, cur - step, cur + step))
        return out

    def resume(self) -> None:
        """GRIP FAIL 뒤 재시도: 목표 크기를 키우고 경로를 더 갈 수 있게 한 뒤 서보를 다시 켠다 (자세는 그대로)."""
        self.attempt += 1
        self.cfg.target_size_px += self.cfg.retry_size_step_px
        if self.cfg.approach_mode == "pose":
            self.max_progress += self.cfg.retry_overreach
        else:
            self.max_progress += self.cfg.retry_overreach * self.cfg.max_reach_s
        self.state = "CENTERING"
        self.ready_forced = False
        self.ready_by_hint = False
        self.just_ready = False
        self._refine_since = 0.0

    def update(
        self,
        dets: list[Detection],
        frame_shape: tuple[int, ...],
        dt: float,
        now: float,
        allow_motion: bool,
        descend_hint: bool = False,
    ) -> dict[str, float]:
        """descend_hint: front 뷰 정중앙에 보라색이 보임(알맞게 내려온 높이). 첫 시도에서만 쓴다 —
        재시도는 '그 높이에서 실패했으니 더 내려가자' 이므로 힌트를 무시하고 키운 목표 크기까지 간다."""
        self.just_ready = False
        if self.state == "READY":
            return self.current

        h, w = frame_shape[:2]
        cx0, cy0 = w // 2, h // 2
        self.target = largest(dets)

        if self.target is None:
            if now - self.last_seen > self.cfg.lost_timeout_s:
                self.state = "LOST"
            return self.current  # 잠깐 놓친 건 그대로 유지

        self.last_seen = now
        x1, y1, x2, y2 = self.target.xyxy
        bx, by = self.target.center
        self.size = box_size(self.target, self.cfg.size_metric)
        # 가로: 기준 x(왼쪽 변/중심/오른쪽 변)를 화면 세로선에 맞춘다
        self.anchor_x = {"left": x1, "center": bx, "right": x2}[self.cfg.x_anchor]
        self.dx = self.anchor_x - cx0 - self.cfg.x_target_dx
        self.x_ok = abs(self.dx) <= self.cfg.x_tolerance_px
        # 세로
        m = self.cfg.inside_margin_px
        inside_y = y1 + m <= cy0 <= y2 - m  # 화면 중앙 높이가 박스 위/아래 변 사이 (느슨한 조건)
        if self.cfg.y_anchor == "inside":
            self.anchor_y = by
            self.dy = by - cy0
            self.y_ok = inside_y
        else:
            self.anchor_y = y1 if self.cfg.y_anchor == "top" else by
            self.dy = (self.anchor_y - cy0) - self.cfg.y_target_dy
            self.y_ok = abs(self.dy) <= self.cfg.y_tolerance_px
        # 뻗기 게이트: 가로는 맞아야 하고, 세로는 정확히 맞거나 최소한 중앙 높이가 박스 안이면 진행
        # (참조 구성은 '최종' 모양이라 접근 초반엔 정확히 맞지 않을 수 있다. 최종 READY 는 엄격하게 본다)
        self.inside = self.x_ok and (self.y_ok or inside_y)

        if not allow_motion:
            # 일시정지/dry-run: 판단만 하고 움직이지 않는다
            self.state = "APPROACHING" if self.inside else "CENTERING"
            return self.current

        # 높이 힌트: front 정중앙에 그리퍼가 보이고 가로만 맞으면 크기와 무관하게 READY (첫 시도만)
        if descend_hint and self.cfg.front_hint and self.attempt == 0 and self.x_ok:
            self.state = "READY"
            self.just_ready = True
            self.ready_by_hint = True
            return self.current

        size_reached = self.size >= self.cfg.target_size_px - self.cfg.size_tolerance_px
        refine_timed_out = (
            self.state == "REFINING"
            and self.cfg.refine_timeout_s > 0
            and now - self._refine_since >= self.cfg.refine_timeout_s
        )
        if size_reached and ((self.x_ok and self.y_ok) or refine_timed_out):
            self.state = "READY"
            self.just_ready = True
            self.ready_forced = refine_timed_out
            return self.current

        if size_reached:
            # 크기는 됐는데 참조 구성(가로/세로)이 아직 안 맞음 → 더 뻗지 않고 보정만 (refine_timeout_s 까지)
            if self.state != "REFINING":
                self._refine_since = now
            lim = self.cfg.max_correction_deg
            self.pan_delta = float(np.clip(self.pan_delta + self.cfg.pan_sign * self.cfg.pan_gain * self.dx * dt, -lim, lim))
            self.tilt_delta = float(
                np.clip(self.tilt_delta + self.cfg.tilt_sign * self.cfg.tilt_gain * self.dy * dt, -lim, lim)
            )
            self.state = "REFINING"
        elif not self.inside:
            # 2. 중앙 맞추기: 오차에 비례해 보정을 누적 (관절이 위치 제어라 적분형이 맞다)
            lim = self.cfg.max_correction_deg
            self.pan_delta = float(np.clip(self.pan_delta + self.cfg.pan_sign * self.cfg.pan_gain * self.dx * dt, -lim, lim))
            self.tilt_delta = float(
                np.clip(self.tilt_delta + self.cfg.tilt_sign * self.cfg.tilt_gain * self.dy * dt, -lim, lim)
            )
            self.state = "CENTERING"
        else:
            # 3. 접근: 중앙점이 박스 안에 있을 때만 경로를 따라 전진
            self.progress = min(self.max_progress, self.progress + self.progress_rate * dt)
            self.state = "APPROACHING"

        self.current = self._limit_speed(self._compose(), dt)
        return self.current

    @property
    def progress_pct(self) -> float:
        return 100.0 * self.progress / self.max_progress if self.max_progress else 0.0


def save_reference(
    path: str | Path, det: Detection, frame_shape: tuple[int, ...], cfg: GraspArgs, note: str = ""
) -> dict:
    """지금 손목 뷰에 보이는 박스와 화면 중심의 위치 관계를 참조 파일로 저장한다.

    저장되는 목표: x_anchor 변의 dx, y_anchor(top 기본) 의 dy, 박스 크기. 다음 실행부터 서보가 이 구성을 재현한다.
    """
    h, w = frame_shape[:2]
    cx0, cy0 = w // 2, h // 2
    x1, y1, x2, y2 = det.xyxy
    bx, by = det.center
    x_anchor = cfg.x_anchor
    # inside 모드였다면 위 변 기준으로 저장한다 (박스 아래가 잘려도 재현 가능)
    y_anchor = cfg.y_anchor if cfg.y_anchor != "inside" else "top"
    anchor_x = {"left": x1, "center": bx, "right": x2}[x_anchor]
    anchor_y = y1 if y_anchor == "top" else by
    ref = {
        "name": Path(path).stem,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": note or "lekiwi_yolo_pick.py 서보 화면에서 S 키로 저장",
        "view": cfg.view,
        "frame_size": [w, h],
        "note": "화면 중심 기준 큐브 박스의 상대 위치. 집기 직전 위치. 서보가 이 구성을 재현한다.",
        "x_anchor": x_anchor,
        "x_target_dx": int(anchor_x - cx0),
        "y_anchor": y_anchor,
        "y_target_dy": int(anchor_y - cy0),
        "size_metric": cfg.size_metric,
        "target_size_px": int(box_size(det, cfg.size_metric)),
        "measured_box_xyxy": [int(x1), int(y1), int(x2), int(y2)],
    }
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(ref, indent=2, ensure_ascii=False) + "\n")
    return ref
