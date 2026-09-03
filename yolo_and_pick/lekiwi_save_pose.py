#!/usr/bin/env python
r"""LeKiwi 팔의 현재 관절 위치를 읽어 JSON 으로 저장한다 (pick 직전 자세 등을 기록해 두는 용도).

리더암으로 팔을 원하는 자세로 만들어 둔 뒤 (또는 손으로 잡아 놓은 뒤) 실행:

    python lekiwi_save_pose.py --name pre_pick
    → poses/pre_pick.json

저장된 파일은 lekiwi_yolo_pick.py 가 `--pick.pose_file` 로 읽어, 큐브에 충분히 접근했을 때
그 자세로 팔을 움직인다.

    python lekiwi_save_pose.py --list=true        # 저장된 자세 목록 (draccus 라 =true 필요)
    python lekiwi_save_pose.py --name home        # 다른 이름으로 저장
    python lekiwi_save_pose.py --name pre_pick --samples 10   # 여러 번 읽어 평균 (노이즈 줄이기)

호스트에 클라이언트는 하나만 붙으므로 teleoperate/record/pick 스크립트가 붙어 있으면 먼저 끌 것.
전체 옵션은 `python lekiwi_save_pose.py --help`.
"""

import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from lerobot.configs import parser
from lerobot.robots.lekiwi import LeKiwiClient
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.utils import init_logging

from lekiwi_yolo_view import LeKiwiRobotArgs

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_POSE_DIR = SCRIPT_DIR / "poses"


@dataclass
class SavePoseConfig:
    robot: LeKiwiRobotArgs = field(default_factory=LeKiwiRobotArgs)
    # 저장 이름 → <pose_dir>/<name>.json
    name: str = "pre_pick"
    pose_dir: Path = DEFAULT_POSE_DIR
    # 이 횟수만큼 읽어 평균낸다
    samples: int = 5
    sample_interval_s: float = 0.1
    # 이미 있으면 덮어쓸지
    overwrite: bool = True
    # 저장하지 않고 목록만 출력
    list: bool = False


def load_pose(path: Path) -> dict[str, float]:
    """저장된 자세 파일을 읽는다 ({"arm_shoulder_pan.pos": ..., ...} 형식)."""
    data = json.loads(Path(path).read_text())
    pose = data["pose"] if "pose" in data else data
    if not pose or not all(isinstance(v, (int, float)) for v in pose.values()):
        raise ValueError(f"{path} 는 자세 파일이 아닙니다 (관절→값 dict 또는 {{'pose': {{...}}}} 형식이어야 함)")
    return {k: float(v) for k, v in pose.items()}


def read_arm_pose(robot: LeKiwiClient, samples: int, interval_s: float) -> dict[str, float]:
    """관절 .pos 값을 여러 번 읽어 평균낸다."""
    acc: dict[str, list[float]] = {}
    for _ in range(max(1, samples)):
        obs = robot.get_observation()
        for k, v in obs.items():
            if k.endswith(".pos"):
                acc.setdefault(k, []).append(float(v))
        time.sleep(interval_s)
    if not acc:
        raise SystemExit("error: 팔 관절 위치(.pos)를 하나도 받지 못했습니다. 호스트가 팔에 연결돼 있는지 확인하세요.")
    return {k: float(np.mean(v)) for k, v in acc.items()}


@parser.wrap()
def main(cfg: SavePoseConfig) -> None:
    init_logging()
    cfg.pose_dir.mkdir(parents=True, exist_ok=True)

    if cfg.list:
        files = sorted(cfg.pose_dir.glob("*.json"))
        if not files:
            print(f"{cfg.pose_dir} 에 저장된 자세가 없습니다.")
            return
        for f in files:
            try:
                pose = load_pose(f)
            except ValueError:
                continue  # grasp_ref.json 같은 다른 종류의 파일은 건너뛴다
            print(f"{f.stem:>12s}: " + ", ".join(f"{k.removesuffix('.pos')}={v:.1f}" for k, v in pose.items()))
        return

    out = cfg.pose_dir / f"{cfg.name}.json"
    if out.exists() and not cfg.overwrite:
        raise SystemExit(f"error: {out} 가 이미 있습니다. 덮어쓰려면 --overwrite=true")

    robot = LeKiwiClient(cfg.robot.to_config())
    try:
        logging.info("LeKiwi 호스트(%s)에 접속 중...", cfg.robot.remote_ip)
        try:
            robot.connect()
        except DeviceNotConnectedError as e:
            raise SystemExit(
                f"error: LeKiwi 호스트({cfg.robot.remote_ip})에 연결하지 못했습니다: {e}\n"
                "  라즈베리파이에서 lekiwi_host 가 실행 중인지, 다른 클라이언트가 붙어 있지 않은지 확인하세요."
            ) from e
        pose = read_arm_pose(robot, cfg.samples, cfg.sample_interval_s)
    finally:
        if robot.is_connected:
            robot.disconnect()

    out.write_text(
        json.dumps(
            {"name": cfg.name, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"), "robot_id": cfg.robot.id, "pose": pose},
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )
    print(f"저장: {out}")
    for k, v in pose.items():
        print(f"  {k:<28s} {v:8.2f}")


if __name__ == "__main__":
    main()
