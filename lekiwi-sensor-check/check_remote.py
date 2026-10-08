#!/usr/bin/env python3
"""노트북에서 IP로 Pi를 선택해 설치·센서 검사·기록 수집을 진행한다."""
import datetime as dt
import hashlib
import ipaddress
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import uuid

import check_sensors as check

ROOT = Path(__file__).resolve().parent
CACHE = ".cache/lekiwi-sensor-check"
SOURCES = ("CMakeLists.txt", "LICENSE", "README.md", "THIRD_PARTY_NOTICES.md",
           "setup.sh", "check_sensors.sh", "check_sensors.py", "remote_worker.py", "probe_build.py",
           "native", "vendor")


def package_sources(destination):
    # 노트북 바이너리, 결과, Git 설정과 자격 증명은 전송하지 않는다.
    files = []
    for name in SOURCES:
        path = ROOT / name
        files.extend(sorted(p for p in path.rglob("*") if p.is_file()) if path.is_dir() else [path])
    digest = hashlib.sha256()
    with tarfile.open(destination, "w:gz") as archive:
        for path in files:
            if path.is_symlink() or not path.is_file():
                raise RuntimeError(f"전송할 소스 파일을 확인하세요: {path}")
            relative = str(path.relative_to(ROOT))
            digest.update(relative.encode() + b"\0" + str(path.stat().st_mode & 0o777).encode() + b"\0")
            digest.update(path.read_bytes())
            archive.add(path, arcname=relative, recursive=False)
    return digest.hexdigest()


def sensor_arguments(args):
    values = ["--duration", str(args.duration), "--imu-device", args.imu_device,
              "--imu-address", args.imu_address]
    for option, value in (("--unit", args.unit), ("--lidar-port", args.lidar_port), ("--only", args.only)):
        if value is not None:
            values.extend([option, value])
    return values


def bootstrap_command(release, job):
    incoming = f"{CACHE}/incoming/{job}.tar.gz"
    directory = f"{CACHE}/releases/{release}"
    # 새 버전은 별도 디렉터리에 두며, 기존 저장소와 빌드 파일을 덮어쓰지 않는다.
    return (
        'set -eu; cd "$HOME"; '
        f"mkdir -p {CACHE}/releases; "
        f"if mkdir {directory} 2>/dev/null; then "
        f"tar -xzf {incoming} -C {directory} && touch {directory}/.source-ready || exit 2; "
        f"elif [ ! -f {directory}/.source-ready ]; then "
        "echo '이전 소스 준비가 완료되지 않았습니다. 해당 캐시를 확인하세요.' >&2; exit 2; fi; "
        f"rm -f -- {incoming}; "
        f"exec /usr/bin/python3 -u {directory}/remote_worker.py --prepare --job {job}"
    )


def parser():
    cli = check.parser()
    cli.description = "노트북에서 SSH로 BMI160·Tmini Plus 검사. IP는 check_robot.sh에서 설정하세요."
    cli.set_defaults(unit=None, output=ROOT / "reports")
    cli.add_argument("--ip", required=True, help="점검할 Pi의 IP")
    cli.add_argument("--user", default="roboseasy", help="Pi SSH 계정")
    return cli


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    try:
        address = ipaddress.ip_address(args.ip)
    except ValueError:
        cli.error("--ip에 실제 IPv4 또는 IPv6 주소를 입력하세요")
    if not re.fullmatch(r"[a-z_][a-z0-9_-]*", args.user):
        cli.error("--user에 Pi의 Linux 계정명을 입력하세요")
    if not check.number(args.duration) or not 3 <= args.duration <= 60:
        cli.error("--duration은 3~60초입니다")
    if any(shutil.which(name) is None for name in ("ssh", "scp")):
        print("노트북에 OpenSSH 클라이언트가 필요합니다: sudo apt install openssh-client", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    target = f"{args.user}@{address}"
    scp_target = f"{args.user}@[{address}]" if address.version == 6 else target
    job = uuid.uuid4().hex
    now = dt.datetime.now().astimezone()
    prefix = f"{now:%Y%m%d-%H%M%S}_{str(address).replace(':', '_')}_"
    local_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=args.output))
    connection = {"ip": str(address), "user": args.user, "job": job, "started_at": now.isoformat(),
                  "status": "ERROR", "sensor_returncode": None}
    result = 2
    with tempfile.TemporaryDirectory(prefix="lekiwi-ssh-") as temporary:
        archive = Path(temporary) / "source.tar.gz"
        socket_path = str(Path(temporary) / "control")
        options = ["-o", f"ControlPath={socket_path}", "-o", "ConnectTimeout=8",
                   "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=2"]
        ssh = ["ssh", *options]
        scp = ["scp", *options]
        connected = False
        try:
            release = package_sources(archive)
            connection["source_sha256"] = release
            print(f"대상: {target}\n로봇을 정지시켜 주세요. 검사 중 라이다가 회전합니다.", flush=True)
            print("[1/4] SSH 접속 (처음에는 호스트 확인과 비밀번호 입력이 필요할 수 있습니다)", flush=True)
            subprocess.run([*ssh, "-M", "-N", "-f", "-o", "ControlPersist=120", target], check=True)
            connected = True
            print("[2/4] Pi에 소스 준비 및 최초 빌드", flush=True)
            subprocess.run([*ssh, target, f'cd "$HOME" && mkdir -p {CACHE}/incoming'], check=True)
            subprocess.run([*scp, str(archive), f"{scp_target}:{CACHE}/incoming/{job}.tar.gz"], check=True)
            subprocess.run([*ssh, "-tt", target, bootstrap_command(release, job)], check=True)
            worker = f"{CACHE}/releases/{release}/remote_worker.py"
            command = ['sudo', '--', '/usr/bin/python3', '-u', worker, '--measure', '--job', job,
                       "--", *sensor_arguments(args)]
            print("[3/4] Pi에서 IMU·라이다 검사 (sudo 비밀번호를 물으면 Pi 계정 비밀번호 입력)", flush=True)
            try:
                measured = subprocess.run([*ssh, "-tt", target, 'cd "$HOME" && exec ' + shlex.join(command)])
                remote_code = measured.returncode
            except KeyboardInterrupt:
                # SSH의 PTY가 닫히면 Pi 작업자가 HUP을 받아 자신의 측정기를 중단한다.
                remote_code = 130
                print("\n검사 중단을 요청했습니다. 라이다의 실제 회전 정지를 확인하세요.", flush=True)
                status_path = f"{CACHE}/results/{job}/remote_status.json"
                wait = f'cd "$HOME"; for attempt in 1 2 3 4 5 6 7 8 9 10; do [ -f {status_path} ] && exit 0; sleep 1; done; exit 2'
                subprocess.run([*ssh, "-o", "BatchMode=yes", target, wait], check=True, timeout=20)
            print("[4/4] 검사 기록을 노트북으로 가져오기", flush=True)
            subprocess.run([*scp, "-o", "BatchMode=yes", "-r",
                            f"{scp_target}:{CACHE}/results/{job}/.", str(local_dir)], check=True)
            status = json.loads((local_dir / "remote_status.json").read_text())
            code = status.get("returncode")
            if status.get("job") != job or type(code) is not int or code not in (0, 1, 2, 130):
                raise RuntimeError("Pi 검사 완료 기록을 확인하지 못했습니다")
            if remote_code != code:
                raise RuntimeError(f"SSH 종료 상태({remote_code})와 Pi 검사 종료 상태({code})가 다릅니다")
            reports = list((local_dir / "reports").glob("*/report.json"))
            if code in (0, 1, 130) and len(reports) != 1:
                raise RuntimeError("센서 판정 보고서를 확인하지 못했습니다")
            if code == 0 and json.loads(reports[0].read_text()).get("status") != "PASS":
                raise RuntimeError("Pi 종료 코드와 센서 판정이 다릅니다")
            connection.update(status="PASS" if code == 0 else ("INTERRUPTED" if code == 130 else "FAIL"),
                              sensor_returncode=code)
            result = code
            print(f"노트북 기록: {local_dir}")
            if reports:
                print(f"판정 보고서: {reports[0]}")
        except KeyboardInterrupt:
            connection.update(status="INTERRUPTED", error="사용자가 원격 작업을 중단했습니다")
            result = 130
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            connection["error"] = str(error)
            print(f"원격 실행 오류: {error}\n로컬 기록: {local_dir}", file=sys.stderr)
        finally:
            if connected:
                try:
                    subprocess.run([*ssh, "-O", "exit", target], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=5)
                except (OSError, subprocess.TimeoutExpired):
                    pass  # 이 실행 전용 SSH 연결은 ControlPersist 제한으로도 종료된다.
            connection["finished_at"] = dt.datetime.now().astimezone().isoformat()
            (local_dir / "connection.json").write_text(json.dumps(connection, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as error:
        print(f"노트북 기록/실행 오류: {error}", file=sys.stderr)
        sys.exit(2)
