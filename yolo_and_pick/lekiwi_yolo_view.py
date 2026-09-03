#!/usr/bin/env python
r"""LeKiwi 의 front / wrist 카메라를 받아 YOLO 실시간 추론 결과를 화면에 띄운다.

로봇을 움직이지는 않는다 (관측만 받는다). 픽 동작을 붙이기 전에, 모델이 실제 로봇
카메라에서 얼마나 잘 잡는지 눈으로 확인하는 용도.

준비:

    pip install ultralytics
    python download_hf_model.py            # weights/best.pt 내려받기

라즈베리파이(LeKiwi) 쪽에서 호스트를 먼저 띄워 둘 것 (lekiwi-teleoperate.py 와 동일):

    python -m lerobot.robots.lekiwi.lekiwi_host \
        --robot.id=lekiwi01 \
        --robot.cameras='{
          front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
          wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
        }' \
        --host.connection_time_s=14400

그 다음 PC 에서:

    python lekiwi_yolo_view.py

기본값(위 장비 세팅)이 이미 들어 있으므로 보통은 위 한 줄이면 된다. 전부 인자로 바꿀 수 있다:

    python lekiwi_yolo_view.py \
        --robot.remote_ip=192.168.0.201 \
        --robot.id=lekiwi01 \
        --yolo.path=weights/best.pt \
        --yolo.conf=0.5 \
        --yolo.device=0 \
        --views='[front, wrist]' \
        --fps=30 \
        --display=cv2

주의
    * LeKiwi 호스트에는 클라이언트가 하나만 붙는다. lekiwi-teleoperate.py / lekiwi-record.py
      가 돌고 있으면 먼저 끄고 실행할 것.
    * `--views` 는 호스트의 `--robot.cameras` 이름과 같아야 프레임이 온다.
    * 창이 안 뜨는 환경(SSH/헤드리스)이면 `--display=rerun` 을 쓴다.
    * 추론이 무거우면 `--infer_every=2` 처럼 건너뛰며 돌린다 (표시는 계속 30Hz).

종료: 창에서 Q 또는 ESC, 터미널에서 Ctrl+C.

전체 옵션은 `python lekiwi_yolo_view.py --help`.
"""

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from pprint import pformat

import cv2
import numpy as np

from lerobot.cameras import CameraConfig  # noqa: F401  (draccus 서브클래스 등록용)
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = SCRIPT_DIR / "weights" / "best.pt"

# 클래스별 박스 색 (BGR). 클래스 수보다 적으면 순환해서 쓴다.
BOX_COLORS = [(60, 60, 255), (60, 220, 60), (255, 170, 40), (220, 60, 220), (40, 220, 220)]
# 가상 중앙 가로선/세로선 색 (BGR, 초록)
CROSSHAIR_COLOR = (0, 255, 0)


def default_cameras() -> dict[str, CameraConfig]:
    """클라이언트 쪽 카메라 선언 (LeKiwiClient 는 카메라를 직접 열지 않는다).

    이름은 호스트와 같아야 프레임을 받는다. width/height 는 표시/추론에는 영향이 없다.
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
class YoloArgs:
    """ultralytics YOLO 추론 설정."""

    # 가중치 경로. download_hf_model.py 가 받아 두는 위치가 기본값.
    path: str = str(DEFAULT_MODEL_PATH)
    conf: float = 0.4
    iou: float = 0.45
    imgsz: int = 640
    # None 이면 ultralytics 가 알아서 고른다 ("cpu", "0", "cuda:0" 등).
    device: str | None = None
    max_det: int = 10
    # 특정 클래스만 보고 싶을 때: --yolo.classes='[0]'
    classes: list[int] | None = None
    half: bool = False


@dataclass
class LeKiwiYoloConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    yolo: YoloArgs = field(default_factory=YoloArgs)

    # 추론/표시할 카메라. 호스트에서 오는 이름이어야 한다.
    views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    # 표시 루프 주기
    fps: int = 30
    # N 프레임마다 한 번만 추론한다 (1 = 매 프레임). 그 사이에는 직전 결과를 그린다.
    infer_every: int = 1
    # 이 시간(초)이 지나면 자동 종료. None 이면 Q/ESC/Ctrl+C 까지 계속.
    run_time_s: float | None = None

    # cv2  - OpenCV 창 하나에 뷰들을 가로로 붙여서 표시 (기본)
    # rerun- rerun viewer 로 전송 (헤드리스/원격에서 유용)
    # none - 화면 표시 없이 검출 결과만 터미널에 출력
    display: str = "cv2"
    # cv2 창에서 각 뷰의 표시 높이(px). 원본 비율은 유지한다.
    view_height: int = 480
    # 가상의 중앙 가로선/세로선(초록)을 그릴 뷰. 끄려면 --crosshair_views='[]', front 만 보려면 --crosshair_views='[front]'
    crosshair_views: list[str] = field(default_factory=lambda: ["front", "wrist"])
    display_ip: str | None = None
    display_port: int | None = None

    # 한 줄짜리 상태 표시 (뷰별 검출 수 / 최고 conf / 중심 좌표, 루프 주파수)
    print_status: bool = True

    def validate(self) -> None:
        if self.display not in ("cv2", "rerun", "none"):
            raise SystemExit(
                f"error: --display 는 cv2/rerun/none 중 하나여야 합니다 (받은 값: {self.display})"
            )
        if self.fps <= 0:
            raise SystemExit(f"error: --fps 는 1 이상이어야 합니다 (받은 값: {self.fps})")
        if self.infer_every < 1:
            raise SystemExit(f"error: --infer_every 는 1 이상이어야 합니다 (받은 값: {self.infer_every})")
        if not self.views:
            raise SystemExit("error: --views 가 비어 있습니다 (예: --views='[front, wrist]')")


@dataclass
class Detection:
    """한 개의 검출 결과 (픽셀 좌표계)."""

    name: str
    conf: float
    xyxy: tuple[int, int, int, int]
    cls: int

    @property
    def center(self) -> tuple[int, int]:
        x1, y1, x2, y2 = self.xyxy
        return (x1 + x2) // 2, (y1 + y2) // 2


def load_model(cfg: YoloArgs):
    """ultralytics 모델 로드 (없으면 무엇을 해야 하는지 알려주고 종료)."""
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "ultralytics 가 설치돼 있지 않습니다. 먼저 설치할 것:\n    pip install ultralytics"
        ) from None

    path = Path(cfg.path).expanduser()
    if not path.exists():
        raise SystemExit(
            f"error: 모델 파일이 없습니다: {path}\n"
            "  먼저 가중치를 받으세요:\n"
            f"      python {SCRIPT_DIR / 'download_hf_model.py'}\n"
            "  다른 곳에 있다면 --yolo.path=/경로/best.pt 로 넘기세요."
        )

    logging.info("YOLO 모델 로드: %s", path)
    model = YOLO(str(path))
    logging.info("클래스: %s", model.names)
    return model


def infer(model, cfg: YoloArgs, frames_bgr: dict[str, np.ndarray]) -> dict[str, list[Detection]]:
    """뷰 여러 개를 한 번의 배치로 추론한다."""
    names = list(frames_bgr)
    results = model.predict(
        [frames_bgr[n] for n in names],
        conf=cfg.conf,
        iou=cfg.iou,
        imgsz=cfg.imgsz,
        device=cfg.device,
        max_det=cfg.max_det,
        classes=cfg.classes,
        half=cfg.half,
        verbose=False,
    )

    out: dict[str, list[Detection]] = {}
    for view, result in zip(names, results, strict=True):
        dets: list[Detection] = []
        for box in result.boxes:
            cls = int(box.cls.item())
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            dets.append(
                Detection(
                    name=result.names.get(cls, str(cls)),
                    conf=float(box.conf.item()),
                    xyxy=(x1, y1, x2, y2),
                    cls=cls,
                )
            )
        dets.sort(key=lambda d: d.conf, reverse=True)
        out[view] = dets
    return out


def draw_crosshair(canvas: np.ndarray) -> np.ndarray:
    """이미지 정중앙을 지나는 가로선/세로선을 제자리에 그린다 (초록)."""
    h, w = canvas.shape[:2]
    cx, cy = w // 2, h // 2
    cv2.line(canvas, (0, cy), (w, cy), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    cv2.line(canvas, (cx, 0), (cx, h), CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    cv2.circle(canvas, (cx, cy), 5, CROSSHAIR_COLOR, 1, cv2.LINE_AA)
    return canvas


def draw(
    frame_bgr: np.ndarray, view: str, dets: list[Detection], fps: float | None, crosshair: bool = False
) -> np.ndarray:
    """검출 박스 + 중심점 + 뷰 이름(+ 옵션으로 중앙 십자선)을 그린 새 이미지를 돌려준다."""
    canvas = frame_bgr.copy()
    if crosshair:
        draw_crosshair(canvas)  # 박스보다 먼저 그려서 박스/라벨이 선 위에 오도록
    for det in dets:
        x1, y1, x2, y2 = det.xyxy
        color = BOX_COLORS[det.cls % len(BOX_COLORS)]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)

        label = f"{det.name} {det.conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y1, th + 4)
        cv2.rectangle(canvas, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
        cv2.putText(canvas, label, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cx, cy = det.center
        cv2.circle(canvas, (cx, cy), 4, color, -1)
        cv2.putText(
            canvas, f"({cx},{cy})", (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
        )

    header = f"{view}  det={len(dets)}"
    if fps is not None:
        header += f"  {fps:4.1f} Hz"
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(canvas, header, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def hstack_views(images: list[np.ndarray], height: int) -> np.ndarray:
    """높이를 맞춰 가로로 이어 붙인다 (뷰 해상도가 서로 달라도 된다)."""
    resized = []
    for img in images:
        scale = height / img.shape[0]
        resized.append(cv2.resize(img, (max(1, int(round(img.shape[1] * scale))), height)))
    return np.hstack(resized)


def camera_views(robot: LeKiwiClient, wanted: list[str]) -> list[str]:
    """설정에 있는 카메라 이름만 남긴다 (오타를 바로 잡아 준다)."""
    available = [name for name, shape in robot.observation_features.items() if isinstance(shape, tuple)]
    missing = [v for v in wanted if v not in available]
    if missing:
        raise SystemExit(
            f"error: --views 의 {missing} 는 --robot.cameras 에 없습니다.\n"
            f"  설정된 카메라: {available}\n"
            "  호스트의 --robot.cameras 와 이름이 같아야 합니다."
        )
    return [v for v in wanted if v in available]


def wait_for_frames(robot: LeKiwiClient, views: list[str], timeout_s: float = 5.0) -> None:
    """첫 프레임이 올 때까지 기다린다 (호스트 카메라 이름이 틀리면 여기서 잡힌다)."""
    deadline = time.perf_counter() + timeout_s
    while time.perf_counter() < deadline:
        obs = robot.get_observation()
        got = {v: obs[v].shape for v in views if isinstance(obs.get(v), np.ndarray)}
        if len(got) == len(views):
            logging.info("수신 중인 카메라: %s", got)
            return
        time.sleep(0.1)
    raise SystemExit(
        f"error: {timeout_s}초 안에 카메라 {views} 프레임을 받지 못했습니다.\n"
        "  라즈베리파이의 lekiwi_host --robot.cameras 이름이 --views 와 같은지 확인하세요."
    )


def status_line(dets_by_view: dict[str, list[Detection]], hz: float) -> str:
    parts = []
    for view, dets in dets_by_view.items():
        if dets:
            best = dets[0]
            cx, cy = best.center
            parts.append(f"{view}: {len(dets)}개 best {best.name} {best.conf:.2f} @({cx},{cy})")
        else:
            parts.append(f"{view}: -")
    return f"\r{' | '.join(parts)} | {hz:5.1f} Hz   "


def view_loop(robot: LeKiwiClient, model, views: list[str], cfg: LeKiwiYoloConfig) -> None:
    window = "LeKiwi YOLO (Q/ESC 종료)"
    interval = 1.0 / cfg.fps
    start = time.perf_counter()
    step = 0
    hz = 0.0
    dets_by_view: dict[str, list[Detection]] = {v: [] for v in views}

    if cfg.display == "rerun":
        from lerobot.utils.visualization_utils import init_visualization

        init_visualization("rerun", session_name="lekiwi_yolo", ip=cfg.display_ip, port=cfg.display_port)

    while True:
        loop_start = time.perf_counter()

        obs = robot.get_observation()
        # LeKiwiClient 프레임은 RGB (호스트가 RGB 를 JPEG 로 실어 보낸다) → YOLO/cv2 용 BGR 로 변환.
        frames_bgr = {
            v: cv2.cvtColor(obs[v], cv2.COLOR_RGB2BGR) for v in views if isinstance(obs.get(v), np.ndarray)
        }
        if not frames_bgr:
            time.sleep(0.05)
            continue

        if step % cfg.infer_every == 0:
            dets_by_view = infer(model, cfg.yolo, frames_bgr)
        step += 1

        annotated = (
            []
            if cfg.display == "none"
            else [
                draw(frames_bgr[v], v, dets_by_view.get(v, []), hz, crosshair=v in cfg.crosshair_views)
                for v in frames_bgr
            ]
        )

        if cfg.display == "cv2":
            cv2.imshow(window, hstack_views(annotated, cfg.view_height))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):  # Q, ESC
                print("\n종료합니다...")
                return
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                print("\n창이 닫혔습니다. 종료합니다...")
                return
        elif cfg.display == "rerun":
            import rerun as rr

            for view, img in zip(frames_bgr, annotated, strict=True):
                rr.log(f"yolo/{view}", rr.Image(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))

        dt = time.perf_counter() - loop_start
        precise_sleep(max(interval - dt, 0.0))
        hz = 1.0 / max(time.perf_counter() - loop_start, 1e-6)

        if cfg.print_status:
            print(status_line(dets_by_view, hz), end="", flush=True)

        if cfg.run_time_s is not None and time.perf_counter() - start >= cfg.run_time_s:
            print(f"\n--run_time_s={cfg.run_time_s} 경과, 종료합니다.")
            return


@parser.wrap()
def main(cfg: LeKiwiYoloConfig) -> None:
    init_logging()
    cfg.validate()
    logging.info(pformat(cfg))

    # 설정 오류(뷰 이름 오타 등)는 무거운 모델 로드 전에 잡는다.
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
        print(f"\n뷰: {', '.join(views)} | 종료: 창에서 Q/ESC, 터미널에서 Ctrl+C\n")

        try:
            view_loop(robot, model, views, cfg)
        except cv2.error as e:
            raise SystemExit(
                f"error: OpenCV 창을 띄우지 못했습니다: {e}\n"
                "  헤드리스/SSH 환경이면 --display=rerun 또는 --display=none 을 쓰세요."
            ) from e
    except KeyboardInterrupt:
        print("\nCtrl+C, 종료합니다...")
    finally:
        if robot.is_connected:
            robot.disconnect()
        if cfg.display == "cv2":
            cv2.destroyAllWindows()
        if cfg.display == "rerun":
            from lerobot.utils.visualization_utils import shutdown_visualization

            shutdown_visualization("rerun")
        print()


if __name__ == "__main__":
    main()
