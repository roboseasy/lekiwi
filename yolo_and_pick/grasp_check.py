r"""집었는지 판별 (lekiwi_yolo_pick.py 의 GRASPED 이후 단계).

판정 기준 (사용자 정의):
    front 뷰에서 큐브 박스의 **바로 왼쪽/오른쪽**에 보라색 그리퍼가 있고,
    동시에 wrist 뷰에서도 큐브 박스의 **왼쪽/오른쪽 변**에 보라색 그리퍼가 있으면 제대로 집은 것.

구현
    * 보라색 = HSV 범위 (OpenCV H 0~180 기준 H 115~165, S ≥ 70, V ≥ 40). LeKiwi 그리퍼 손가락(자주/보라 PLA)
      은 H≈132, S≈180, V≈110 으로 측정됨. 빨간 큐브(H≈0~5)와 흰 테이블(S≈5)은 이 범위에 들지 않는다.
    * 각 뷰에서 큐브 박스의 왼쪽 변과 오른쪽 변에 걸친 세로 띠(band_px 폭, 박스 안쪽으로 inset_px 만큼 포함)를
      잘라 보라색 픽셀 비율을 구한다. 양쪽 모두 min_purple_ratio 이상이면 그 뷰는 OK.
    * 두 뷰가 모두 OK 인 프레임이 confirm_frames 연속이면 SUCCESS. timeout_s 안에 못 채우면 FAIL.
    * 어느 뷰에서든 큐브가 검출되지 않으면 그 프레임은 NOT OK (집으면서 큐브가 가려질 수 있으니 timeout 을 넉넉히).
"""

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from lekiwi_yolo_view import Detection

SCRIPT_DIR = Path(__file__).resolve().parent


@dataclass
class GraspCheckArgs:
    """집기 성공 판별 설정."""

    # false 면 GRASPED 에서 판별 없이 유지
    enabled: bool = True
    # 판별에 쓸 두 뷰
    front_view: str = "front"
    wrist_view: str = "wrist"

    # 보라색 HSV 범위 (OpenCV: H 0~180, S/V 0~255)
    hue_min: int = 115
    hue_max: int = 165
    sat_min: int = 70
    val_min: int = 40

    # 박스 좌/우 변에 걸친 띠: 변 바깥쪽으로 band_px, 안쪽으로 inset_px
    band_px: int = 40
    inset_px: int = 8
    # 띠 안 보라색 픽셀 비율이 이 값 이상이면 그쪽에 그리퍼가 있다고 본다
    min_purple_ratio: float = 0.15
    # 뷰별로 좌/우 둘 다 있어야 OK. false 면 한쪽만 있어도 OK (큐브가 한쪽 손가락을 가릴 때)
    require_both_sides: bool = True

    # 두 뷰 모두 OK 인 프레임이 이만큼 연속이면 SUCCESS
    confirm_frames: int = 5
    # 이 시간(초) 안에 SUCCESS 가 안 나면 FAIL
    timeout_s: float = 3.0
    # 결과(SUCCESS/FAIL)가 나오면 스크립트를 끝낸다
    exit_on_result: bool = False

    def validate(self) -> None:
        if not self.enabled:
            return
        if not 0 <= self.hue_min <= self.hue_max <= 180:
            raise SystemExit(f"error: --check.hue_min/hue_max 는 0~180 이고 min<=max 여야 합니다 ({self.hue_min}, {self.hue_max})")
        if not 0 <= self.sat_min <= 255 or not 0 <= self.val_min <= 255:
            raise SystemExit("error: --check.sat_min / val_min 은 0~255 여야 합니다")
        if self.band_px <= 0 or self.inset_px < 0:
            raise SystemExit("error: --check.band_px 는 0 보다, inset_px 는 0 이상이어야 합니다")
        if not 0.0 < self.min_purple_ratio <= 1.0:
            raise SystemExit(f"error: --check.min_purple_ratio 는 0~1 사이여야 합니다 (받은 값: {self.min_purple_ratio})")
        if self.confirm_frames < 1 or self.timeout_s <= 0:
            raise SystemExit("error: --check.confirm_frames 는 1 이상, timeout_s 는 0 보다 커야 합니다")


def purple_mask(frame_bgr: np.ndarray, cfg: GraspCheckArgs) -> np.ndarray:
    """보라색 픽셀 마스크 (0/255)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (cfg.hue_min, cfg.sat_min, cfg.val_min), (cfg.hue_max, 255, 255))


def center_purple_ratio(frame_bgr: np.ndarray, cfg: GraspCheckArgs, win_px: int = 20) -> float:
    """화면 정중앙 win_px×win_px 창 안의 보라색 픽셀 비율.

    front 뷰에서 정중앙에 보라색(그리퍼)이 보이면 팔이 딱 알맞게 내려온 높이라는 힌트로 쓴다.
    """
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    r = max(1, win_px // 2)
    roi = frame_bgr[max(0, cy - r) : min(h, cy + r), max(0, cx - r) : min(w, cx + r)]
    if roi.size == 0:
        return 0.0
    return float(purple_mask(roi, cfg).mean() / 255.0)


def edge_strips(xyxy: tuple[int, int, int, int], frame_shape: tuple[int, ...], cfg: GraspCheckArgs):
    """박스 왼쪽/오른쪽 변에 걸친 띠 두 개의 (x1, y1, x2, y2). 화면 밖은 잘라낸다."""
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = xyxy
    y1c, y2c = max(0, y1), min(h, y2)
    left = (max(0, x1 - cfg.band_px), y1c, min(w, x1 + cfg.inset_px), y2c)
    right = (max(0, x2 - cfg.inset_px), y1c, min(w, x2 + cfg.band_px), y2c)
    return left, right


def strip_ratio(mask: np.ndarray, strip: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = strip
    if x2 <= x1 or y2 <= y1:
        return 0.0
    return float(mask[y1:y2, x1:x2].mean() / 255.0)


def largest(dets: list[Detection]) -> Detection | None:
    if not dets:
        return None
    return max(dets, key=lambda d: (d.xyxy[2] - d.xyxy[0]) * (d.xyxy[3] - d.xyxy[1]))


@dataclass
class ViewResult:
    """한 뷰의 판별 결과 (표시용)."""

    box: tuple[int, int, int, int] | None = None
    left_strip: tuple[int, int, int, int] | None = None
    right_strip: tuple[int, int, int, int] | None = None
    left_ratio: float = 0.0
    right_ratio: float = 0.0
    ok: bool = False


class GraspChecker:
    """GRASPED 이후 매 프레임 두 뷰를 보고 SUCCESS / FAIL 을 판정한다.

    상태: IDLE(아직 시작 안 함) → CHECKING → SUCCESS | FAIL
    """

    def __init__(self, cfg: GraspCheckArgs):
        self.cfg = cfg
        self.state = "IDLE"
        self.started_at = 0.0
        self.streak = 0
        self.results: dict[str, ViewResult] = {cfg.front_view: ViewResult(), cfg.wrist_view: ViewResult()}
        self.just_decided = False

    @property
    def done(self) -> bool:
        return self.state in ("SUCCESS", "FAIL")

    def start(self, now: float) -> None:
        self.state = "CHECKING"
        self.started_at = now
        self.streak = 0

    def reset(self) -> None:
        """재시도 전 초기화: 다음 GRASPED 에서 다시 시작한다."""
        self.state = "IDLE"
        self.streak = 0
        self.just_decided = False
        self.results = {k: ViewResult() for k in self.results}

    def _check_view(self, frame_bgr: np.ndarray, dets: list[Detection]) -> ViewResult:
        res = ViewResult()
        det = largest(dets)
        if det is None:
            return res
        res.box = det.xyxy
        mask = purple_mask(frame_bgr, self.cfg)
        res.left_strip, res.right_strip = edge_strips(det.xyxy, frame_bgr.shape, self.cfg)
        res.left_ratio = strip_ratio(mask, res.left_strip)
        res.right_ratio = strip_ratio(mask, res.right_strip)
        lo, ro = res.left_ratio >= self.cfg.min_purple_ratio, res.right_ratio >= self.cfg.min_purple_ratio
        res.ok = (lo and ro) if self.cfg.require_both_sides else (lo or ro)
        return res

    def update(
        self,
        frames_bgr: dict[str, np.ndarray],
        dets_by_view: dict[str, list[Detection]],
        now: float,
    ) -> str:
        """프레임마다 호출. 현재 상태 문자열을 돌려준다."""
        self.just_decided = False
        if self.state != "CHECKING":
            return self.state

        for view in (self.cfg.front_view, self.cfg.wrist_view):
            frame = frames_bgr.get(view)
            self.results[view] = (
                self._check_view(frame, dets_by_view.get(view, [])) if frame is not None else ViewResult()
            )

        both_ok = all(r.ok for r in self.results.values())
        self.streak = self.streak + 1 if both_ok else 0
        if self.streak >= self.cfg.confirm_frames:
            self.state = "SUCCESS"
            self.just_decided = True
        elif now - self.started_at >= self.cfg.timeout_s:
            self.state = "FAIL"
            self.just_decided = True
        return self.state

    def summary(self) -> str:
        parts = []
        for view, r in self.results.items():
            parts.append(f"{view} L{r.left_ratio:.2f}/R{r.right_ratio:.2f}{'✓' if r.ok else '✗'}")
        return " ".join(parts)


def draw_grasp_check(frame_bgr: np.ndarray, checker: GraspChecker, view: str) -> np.ndarray:
    """띠 영역과 보라색 비율을 그린다 (CHECKING/SUCCESS/FAIL 중에만)."""
    if checker.state == "IDLE":
        return frame_bgr
    r = checker.results.get(view)
    if r is None or r.box is None:
        return frame_bgr
    canvas = frame_bgr
    thr = checker.cfg.min_purple_ratio
    for strip, ratio in ((r.left_strip, r.left_ratio), (r.right_strip, r.right_ratio)):
        if strip is None:
            continue
        x1, y1, x2, y2 = strip
        color = (255, 80, 200) if ratio >= thr else (120, 120, 120)  # 보라색 있음 → 자주, 없음 → 회색
        overlay = canvas.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        cv2.addWeighted(overlay, 0.3, canvas, 0.7, 0, canvas)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 1)
        cv2.putText(canvas, f"{ratio:.2f}", (x1 + 2, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    label = {"CHECKING": "GRIP?", "SUCCESS": "GRIP OK", "FAIL": "GRIP FAIL"}[checker.state]
    col = {"CHECKING": (0, 200, 255), "SUCCESS": (0, 220, 0), "FAIL": (0, 0, 255)}[checker.state]
    bx1, by1, _, _ = r.box
    cv2.putText(canvas, label, (bx1, max(by1 - 44, 60)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
    return canvas
