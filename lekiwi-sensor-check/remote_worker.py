#!/usr/bin/env python3
"""SSH가 시작한 Pi 작업을 준비하고, 연결 종료 시 측정 자식만 중단한다."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys

from probe_build import validate_probes

ROOT = Path(__file__).resolve().parent
CACHE = ROOT.parent.parent
PACKAGES = "git build-essential cmake python3 psmisc util-linux"


def prepare(job_dir):
    if not all(shutil.which(tool) for tool in ("git", "gcc", "g++", "cmake", "fuser", "flock")):
        if not shutil.which("apt-get") or not shutil.which("sudo"):
            raise RuntimeError(f"Pi 빌드 도구가 없습니다. 관리자에게 설치를 요청하세요: {PACKAGES}")
        print("Pi에 필요한 빌드 도구를 설치합니다. sudo 비밀번호를 입력하세요.", flush=True)
        subprocess.run(["sudo", "--", "apt-get", "update"], check=True)
        subprocess.run(["sudo", "--", "apt-get", "install", "-y", *PACKAGES.split()], check=True)
    ready = ROOT / "build/.remote-ready"
    reusable = False
    if ready.is_file():
        try:
            validate_probes(ROOT)
            reusable = True
        except RuntimeError as error:
            print(f"기존 측정 프로그램을 새로 빌드합니다: {error}", flush=True)
    if not reusable:
        ready.unlink(missing_ok=True)
        subprocess.run(["bash", str(ROOT / "setup.sh")], check=True)
        validate_probes(ROOT)
        os.sync()
        ready.touch()
        os.sync()
    job_dir.mkdir(parents=True, exist_ok=False)


def measure(job_dir, job, sensor_args):
    if os.geteuid() != 0:
        raise RuntimeError("원격 센서 점유·권한 확인을 위해 sudo로 실행해야 합니다")
    if not job_dir.is_dir():
        raise RuntimeError("이 작업의 Pi 준비 단계가 완료되지 않았습니다")
    # 파일은 읽을 수 있게 생성하고, 종료 시 전용 작업 폴더를 SSH 계정에 반환한다.
    os.umask(0o022)
    owner = job_dir.stat()
    child = None
    interrupted = False
    previous = {}

    def stop(signum, frame):
        nonlocal interrupted
        interrupted = True
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass  # 신호를 보내기 전에 이미 종료됐다.

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[sig] = signal.signal(sig, stop)
    code = 2
    error = None
    try:
        if not interrupted:
            child = subprocess.Popen(["/usr/bin/python3", "-u", str(ROOT / "check_sensors.py"),
                                      *sensor_args, "--output", str(job_dir / "reports")], start_new_session=True)
            if interrupted:
                stop(signal.SIGINT, None)
            code = child.wait()
        if interrupted:
            code = 130
    except OSError as exc:
        error = str(exc)
        print(f"Pi 측정 실행 오류: {error}", file=sys.stderr)
    finally:
        try:
            status = {"job": job, "returncode": code, "error": error, "source_release": ROOT.name}
            (job_dir / "remote_status.json").write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")
            # mkdtemp의 0700 폴더도 생성한 SSH 계정이 열 수 있게 한다.
            # 이 실행의 결과만 처리하며 심볼릭 링크의 대상 소유권은 바꾸지 않는다.
            for path in (job_dir, *job_dir.rglob("*")):
                os.chown(path, owner.st_uid, owner.st_gid, follow_symlinks=False)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description="Pi 원격 점검 작업자")
    phase = parser.add_mutually_exclusive_group(required=True)
    phase.add_argument("--prepare", action="store_true")
    phase.add_argument("--measure", action="store_true")
    parser.add_argument("--job", required=True)
    parser.add_argument("sensor_args", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    sensor_args = args.sensor_args[1:] if args.sensor_args[:1] == ["--"] else args.sensor_args
    if not re.fullmatch(r"[0-9a-f]{32}", args.job):
        parser.error("잘못된 작업 번호입니다")
    if args.prepare and sensor_args:
        parser.error("준비 단계에 알 수 없는 인수가 있습니다")
    job_dir = CACHE / "results" / args.job
    try:
        CACHE.mkdir(parents=True, exist_ok=True)
        with (CACHE / "remote.lock").open("a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("이 Pi에서 다른 원격 준비/검사가 실행 중입니다")
            if args.prepare:
                prepare(job_dir)
                return 0
            return measure(job_dir, args.job, sensor_args)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"Pi 준비/실행 오류: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
