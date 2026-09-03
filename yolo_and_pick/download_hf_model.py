#!/usr/bin/env python
r"""Hugging Face 에 올라간 YOLO 가중치 내려받기 (전부 CLI 인자, 코드 수정 불필요).

기본 대상은 공개 레포 `roboseasylabs/red_cube_yolo` (ultralytics YOLO26 파인튜닝,
red cube 검출용). 레포에는 `best.pt` 와 `last.pt` 가 들어 있다.

    pip install huggingface_hub          # 없으면 먼저 설치

가장 흔한 사용법 — `best.pt` 를 이 스크립트 옆 `weights/` 로 받기:

    python download_hf_model.py

다른 파일 / 다른 위치 / 다른 레포:

    python download_hf_model.py --filename last.pt
    python download_hf_model.py --out-dir ~/models/red_cube
    python download_hf_model.py --repo-id someone/other_yolo --filename best.pt

레포 전체(README 포함)를 받으려면:

    python download_hf_model.py --all

받을 수 있는 파일 목록만 확인:

    python download_hf_model.py --list

비공개(gated) 레포라면 `huggingface-cli login` 을 했거나 `HF_TOKEN` 환경변수가
설정돼 있으면 그대로 동작하고, 아니면 `--token hf_xxx` 로 넘긴다.

받은 가중치는 ultralytics 에서 바로 쓸 수 있다:

    from ultralytics import YOLO
    model = YOLO("weights/best.pt")
    results = model.predict(frame, conf=0.5)

전체 옵션은 `python download_hf_model.py --help`.
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

DEFAULT_REPO_ID = "roboseasylabs/red_cube_yolo"
DEFAULT_FILENAME = "best.pt"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent / "weights"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Hugging Face 레포에서 YOLO 가중치 다운로드",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--repo-id",
        default=DEFAULT_REPO_ID,
        help="Hugging Face 모델 레포 id (예: roboseasylabs/red_cube_yolo)",
    )
    parser.add_argument(
        "--filename",
        default=DEFAULT_FILENAME,
        help="받을 파일 이름. --all 을 쓰면 무시된다",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="파일을 복사해 둘 디렉터리",
    )
    parser.add_argument(
        "--revision",
        default=None,
        help="브랜치/태그/커밋 sha (기본: main)",
    )
    parser.add_argument(
        "--token",
        default=None,
        help="HF 액세스 토큰. 생략하면 HF_TOKEN 환경변수나 로그인 캐시를 쓴다",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="파일 하나가 아니라 레포 전체를 내려받는다",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="다운로드하지 않고 레포 안의 파일 목록만 출력한다",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="out-dir 에 같은 이름의 파일이 있어도 덮어쓴다",
    )
    return parser.parse_args()


def require_hub():
    try:
        import huggingface_hub  # noqa: F401
    except ImportError:
        sys.exit(
            "huggingface_hub 가 설치돼 있지 않다. 먼저 설치할 것:\n"
            "    pip install huggingface_hub"
        )
    return huggingface_hub


def main() -> int:
    args = parse_args()
    hub = require_hub()
    token = args.token or os.environ.get("HF_TOKEN")

    from huggingface_hub.utils import (
        EntryNotFoundError,
        GatedRepoError,
        RepositoryNotFoundError,
    )

    api = hub.HfApi(token=token)

    try:
        files = api.list_repo_files(args.repo_id, revision=args.revision)
    except RepositoryNotFoundError:
        sys.exit(
            f"레포를 찾을 수 없다: {args.repo_id}\n"
            "이름이 맞는지, 비공개라면 --token 이 필요한지 확인할 것."
        )
    except GatedRepoError:
        sys.exit(
            f"gated 레포다: {args.repo_id}\n"
            "웹에서 접근 승인을 받고 --token 또는 huggingface-cli login 을 쓸 것."
        )

    if args.list:
        print(f"{args.repo_id} ({args.revision or 'main'}) 안의 파일:")
        for name in sorted(files):
            print(f"  {name}")
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)

    if args.all:
        local_dir = args.out_dir / args.repo_id.split("/")[-1]
        path = hub.snapshot_download(
            repo_id=args.repo_id,
            revision=args.revision,
            token=token,
            local_dir=local_dir,
        )
        print(f"레포 전체를 받았다: {path}")
        return 0

    if args.filename not in files:
        sys.exit(
            f"'{args.filename}' 은 {args.repo_id} 에 없다.\n"
            f"있는 파일: {', '.join(sorted(files))}\n"
            "--filename 으로 골라 받을 것."
        )

    dest = args.out_dir / Path(args.filename).name
    if dest.exists() and not args.force:
        print(f"이미 있다 (건너뜀): {dest}")
        print("다시 받으려면 --force 를 붙일 것.")
        return 0

    try:
        cached = hub.hf_hub_download(
            repo_id=args.repo_id,
            filename=args.filename,
            revision=args.revision,
            token=token,
        )
    except EntryNotFoundError:
        sys.exit(f"'{args.filename}' 을 받지 못했다 (레포에 없음).")

    shutil.copyfile(cached, dest)
    size_mb = dest.stat().st_size / 1024 / 1024
    print(f"다운로드 완료: {dest} ({size_mb:.1f} MB)")
    print(f"  HF 캐시 원본: {cached}")
    print("\nultralytics 에서 쓰기:")
    print("    from ultralytics import YOLO")
    print(f'    model = YOLO("{dest}")')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
