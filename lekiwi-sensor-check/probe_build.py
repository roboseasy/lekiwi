"""Build-directory cleanup and device-free checks of native probe programs."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


def clean_build(root):
    build = root / "build"
    if build.is_symlink() or (build.exists() and not build.is_dir()):
        raise RuntimeError("build 경로가 일반 폴더가 아닙니다. 정리를 중단합니다.")
    if build.exists():
        shutil.rmtree(build)


def validate_probes(root, sensors=("imu", "lidar")):
    for sensor in sensors:
        name = f"{sensor}_probe"
        program = root / "build/bin" / name
        try:
            if not program.is_file() or program.stat().st_size == 0 or not os.access(program, os.X_OK):
                raise RuntimeError(f"{name}: 실행 파일이 없거나 비어 있거나 실행 권한이 없습니다.")
            with program.open("rb") as stream:
                if stream.read(4) != b"\x7fELF":
                    raise RuntimeError(f"{name}: 정상 ELF 실행 파일이 아닙니다.")
            # 두 프로그램은 --help에서 장치를 열기 전에 종료한다.
            result = subprocess.run([str(program), "--help"], capture_output=True, timeout=5)
            if result.returncode != 0 or not result.stdout.startswith(name.encode() + b" "):
                raise RuntimeError(f"{name}: --help 실행 검증에 실패했습니다.")
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(f"{name}: 실행 검증 실패 ({error})") from error


def main(argv=None):
    parser = argparse.ArgumentParser(description="센서를 작동시키지 않고 측정 프로그램 빌드를 확인합니다.")
    parser.add_argument("action", choices=("clean", "verify"))
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parent
    try:
        if args.action == "clean":
            clean_build(root)
            print("이전 build 정리 완료. 모든 측정 프로그램을 새로 빌드합니다.")
        else:
            validate_probes(root)
            os.sync()
            print("IMU·라이다 측정 프로그램 빌드 검증 완료")
        return 0
    except (OSError, RuntimeError) as error:
        print(f"측정 프로그램 준비 실패: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
