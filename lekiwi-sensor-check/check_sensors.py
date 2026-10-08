#!/usr/bin/env python3
"""ROS 없이 BMI160과 Tmini Plus를 검사하고 제품별 기록을 저장한다."""
import argparse
import csv
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import stat
import statistics
import subprocess
import sys
import tempfile

from probe_build import validate_probes

ROOT = Path(__file__).resolve().parent
VERSION = "0.1.0"
SDK_COMMIT = "42a82ed10d2304094c111fc63dee8e4a229b79b7"
BOSCH_COMMIT = "252ac2859ac1d2010915b7a7cfcbf501d7adfb40"
LIMITS = {
    "imu_rate_hz": [95.0, 105.0],
    "gravity_m_s2": [9.0, 10.6],
    "gyro_mean_abs_rad_s": 0.05,
    "gyro_std_rad_s": 0.015,
    "accel_std_m_s2": 0.20,
    "imu_max_gap_s": 0.10,
    "lidar_rate_hz": [8.0, 12.0],
    "lidar_min_valid_points": 30,
    "lidar_min_valid_fraction": 0.10,
    "lidar_good_scan_fraction": 0.90,
}
CHECK_LABELS = {
    "native_exit_ok": "측정 프로그램 정상 종료",
    "cleanup_completed": "측정 종료 처리",
    "no_native_errors": "센서 통신 오류 없음",
    "bmi160_identified": "BMI160 식별",
    "tmini_plus_identified": "Tmini Plus 식별",
    "sample_format": "IMU 측정값 형식",
    "scan_format": "라이다 스캔 형식",
    "sample_count": "IMU 표본 수",
    "scan_count": "스캔 수",
    "measurement_duration": "측정 시간",
    "rate_hz": "실제 수신 주기",
    "fresh_samples": "IMU 새 데이터 갱신",
    "fresh_scans": "스캔 시각 갱신",
    "no_long_gap": "IMU 데이터 끊김",
    "gravity": "중력 가속도 크기",
    "gyro_raw_bias": "정지 상태 자이로 편차",
    "stationary_acceleration": "정지 상태 가속도 흔들림",
    "stationary_gyro": "정지 상태 자이로 흔들림",
    "raw_data_changes": "IMU 값 반복 고정 여부",
    "valid_ranges": "유효 거리 비율",
    "enough_points": "스캔별 유효 거리값 수",
    "scan_errors": "스캔 수신 오류 비율",
    "range_data_changes": "라이다 거리값 반복 고정 여부",
    "preflight_or_execution": "장치 연결·권한·점유 상태",
    "not_interrupted": "검사 완료 여부",
}


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value):
    return isinstance(value, list) and len(value) == 3 and all(number(v) and abs(v) <= 1000 for v in value)


def sample_rate(samples):
    if len(samples) < 2 or samples[-1]["t"] <= samples[0]["t"]:
        return 0.0
    return (len(samples) - 1) / (samples[-1]["t"] - samples[0]["t"])


def result(checks, metrics, errors=()):
    return {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "metrics": metrics,
        "errors": list(errors),
    }


def native_checks(records, returncode):
    cleanup = [r for r in records if r.get("type") == "cleanup"]
    return {
        "native_exit_ok": returncode == 0,
        "cleanup_completed": bool(cleanup) and cleanup[-1].get("ok") is True,
        "no_native_errors": not any(r.get("type") == "error" for r in records),
    }


def evaluate_imu(records, returncode, duration):
    checks = native_checks(records, returncode)
    info = [r for r in records if r.get("type") == "imu_info"]
    checks["bmi160_identified"] = bool(info) and info[-1].get("chip_id") == 0xd1 and info[-1].get("address") in (0x68, 0x69)
    samples = [r for r in records if r.get("type") == "imu_sample"]
    metrics = {"samples": len(samples), "identity": info[-1] if info else None}
    checks["sample_format"] = bool(samples) and all(
        number(s.get("t")) and 0 <= s["t"] <= duration + 5
        and vector(s.get("acc")) and vector(s.get("gyro"))
        and type(s.get("sensor_time")) is int and 0 <= s["sensor_time"] < 2**24
        and isinstance(s.get("raw"), list) and len(s["raw"]) == 6
        and all(type(v) is int and -32768 <= v <= 32767 for v in s["raw"])
        for s in samples
    )
    errors = [r.get("message", "IMU 오류") for r in records if r.get("type") == "error"]
    if not checks["sample_format"]:
        return result(checks, metrics, errors)
    times = [s["t"] for s in samples]
    gaps = [b - a for a, b in zip(times, times[1:])]
    hz = sample_rate(samples)
    means_acc = [statistics.mean(s["acc"][axis] for s in samples) for axis in range(3)]
    means_gyro = [statistics.mean(s["gyro"][axis] for s in samples) for axis in range(3)]
    std_acc = [statistics.pstdev(s["acc"][axis] for s in samples) for axis in range(3)]
    std_gyro = [statistics.pstdev(s["gyro"][axis] for s in samples) for axis in range(3)]
    gravity = statistics.mean(math.hypot(*s["acc"]) for s in samples)
    distinct_raw = len({tuple(s["raw"]) for s in samples})
    checks.update({
        "sample_count": len(samples) >= duration * 90,
        "measurement_duration": times[-1] - times[0] >= duration * 0.90,
        "rate_hz": LIMITS["imu_rate_hz"][0] <= hz <= LIMITS["imu_rate_hz"][1],
        "fresh_samples": bool(gaps) and min(gaps) > 0 and all(
            (b["sensor_time"] - a["sensor_time"]) % 2**24 > 0 for a, b in zip(samples, samples[1:])),
        "no_long_gap": bool(gaps) and max(gaps) <= LIMITS["imu_max_gap_s"],
        "gravity": LIMITS["gravity_m_s2"][0] <= gravity <= LIMITS["gravity_m_s2"][1],
        # bias를 자동으로 빼지 않아 큰 offset이 정상처럼 표시되지 않는다.
        "gyro_raw_bias": max(abs(v) for v in means_gyro) <= LIMITS["gyro_mean_abs_rad_s"],
        "stationary_acceleration": max(std_acc) <= LIMITS["accel_std_m_s2"],
        "stationary_gyro": max(std_gyro) <= LIMITS["gyro_std_rad_s"],
        "raw_data_changes": distinct_raw >= 2,
    })
    metrics.update(rate_hz=hz, gravity_m_s2=gravity, acceleration_mean_m_s2=means_acc,
                   gyro_mean_rad_s=means_gyro, acceleration_std_m_s2=std_acc,
                   gyro_std_rad_s=std_gyro, max_gap_s=max(gaps, default=0), distinct_raw_frames=distinct_raw)
    return result(checks, metrics, errors)


def evaluate_lidar(records, returncode, duration):
    checks = native_checks(records, returncode)
    info = [r for r in records if r.get("type") == "lidar_info"]
    checks["tmini_plus_identified"] = bool(info) and info[-1].get("model_code") in (151, 152)
    scans = [r for r in records if r.get("type") == "lidar_scan"]
    metrics = {"scans": len(scans), "identity": info[-1] if info else None}
    checks["scan_format"] = bool(scans) and all(
        number(s.get("t")) and 0 <= s["t"] <= duration + 5
        and type(s.get("stamp_ns")) is int and s["stamp_ns"] > 0
        and type(s.get("points")) is int and 0 <= s["points"] <= 100000
        and type(s.get("valid")) is int and 0 <= s["valid"] <= s["points"]
        and isinstance(s.get("ranges"), list) and len(s["ranges"]) == s["valid"]
        and all(isinstance(p, list) and len(p) == 2 and number(p[0]) and abs(p[0]) <= 2 * math.pi
                and number(p[1]) and 0.05 <= p[1] <= 12.0 for p in s["ranges"])
        for s in scans
    )
    errors = [r.get("message", "라이다 오류") for r in records if r.get("type") == "error"]
    if not checks["scan_format"]:
        return result(checks, metrics, errors)
    hz = sample_rate(scans)
    total_points = sum(s["points"] for s in scans)
    valid_fraction = sum(s["valid"] for s in scans) / max(1, total_points)
    good_fraction = sum(s["valid"] >= LIMITS["lidar_min_valid_points"] for s in scans) / len(scans)
    failures = sum(r.get("type") == "scan_error" for r in records)
    checks.update({
        "scan_count": len(scans) >= duration * 8,
        "measurement_duration": scans[-1]["t"] - scans[0]["t"] >= duration * 0.80,
        "rate_hz": LIMITS["lidar_rate_hz"][0] <= hz <= LIMITS["lidar_rate_hz"][1],
        "fresh_scans": len(scans) > 1 and all(
            b["stamp_ns"] > a["stamp_ns"] and b["t"] > a["t"] for a, b in zip(scans, scans[1:])),
        "valid_ranges": valid_fraction >= LIMITS["lidar_min_valid_fraction"],
        "enough_points": good_fraction >= LIMITS["lidar_good_scan_fraction"],
        "scan_errors": failures / max(1, len(scans) + failures) <= 0.10,
        "range_data_changes": len({tuple(tuple(p) for p in s["ranges"]) for s in scans}) >= 2,
    })
    metrics.update(rate_hz=hz, valid_fraction=valid_fraction, good_scan_fraction=good_fraction,
                   mean_valid_points=statistics.mean(s["valid"] for s in scans), scan_errors=failures)
    return result(checks, metrics, errors)


def usb_identity(device, sysfs=Path("/sys/class/tty")):
    node = (sysfs / Path(device).name / "device").resolve()
    for parent in (node, *node.parents):
        try:
            return (parent / "idVendor").read_text().strip().lower(), (parent / "idProduct").read_text().strip().lower()
        except FileNotFoundError:
            continue
    return None


def select_lidar_port(requested=None, dev_root=Path("/dev")):
    if requested:
        candidates = [Path(requested)]
    else:
        candidates = list((dev_root / "serial/by-id").glob("*CP2102*"))
        if not candidates:
            candidates = list(dev_root.glob("ttyUSB*"))
    found = {}
    for path in candidates:
        try:
            resolved = path.resolve(strict=True)
        except FileNotFoundError:
            continue
        if resolved.name.startswith("ttyUSB") and usb_identity(resolved) == ("10c4", "ea60"):
            found[str(resolved)] = path
    if len(found) != 1:
        raise RuntimeError("Tmini Plus용 CP2102 USB 포트를 하나로 식별하지 못했습니다. --lidar-port /dev/serial/by-id/... 로 지정하세요. ttyACM 모터 포트는 허용하지 않습니다")
    return next(iter(found.values()))


def preflight(device):
    if not device.exists():
        raise RuntimeError(f"장치가 없습니다: {device}")
    if not stat.S_ISCHR(device.stat().st_mode):
        raise RuntimeError(f"센서 장치 파일이 아닙니다: {device}")
    if not os.access(device, os.R_OK | os.W_OK):
        raise RuntimeError(f"접근 권한이 없습니다: {device}. i2c/dialout 그룹 설정 후 다시 로그인하세요")
    fuser = shutil.which("fuser")
    if not fuser:
        raise RuntimeError("포트 점유 검사에 필요한 psmisc가 없습니다. ./setup.sh 안내를 확인하세요")
    # fuser는 '--'를 받지 않는다. 절대 경로로 전달해 옵션으로 해석되지 않게 한다.
    checked = subprocess.run([fuser, "-s", str(device.resolve(strict=True))], capture_output=True, text=True,
                             timeout=3, env={**os.environ, "LC_ALL": "C"})
    if checked.returncode == 0:
        raise RuntimeError(f"다른 프로세스가 사용 중입니다: {device}. 해당 센서 앱을 정상 종료한 뒤 검사하세요")
    if checked.returncode != 1 or checked.stderr.strip():
        detail = checked.stderr.strip() or f"fuser 종료 코드 {checked.returncode}"
        raise RuntimeError(f"장치 점유 상태를 확인하지 못했습니다: {device} ({detail})")


def stop_probe(process):
    # 이 검사에서 시작한 프로세스 그룹만 종료한다.
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass  # 종료 확인과 신호 전송 사이에 이미 끝난 경우이다.
    try:
        return process.communicate(timeout=3)[0]
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.communicate(timeout=3)[0]


def run_probe(command, log_path, timeout):
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, encoding="utf-8", errors="replace", start_new_session=True)
    error = None
    interrupted = False
    try:
        output = process.communicate(timeout=timeout)[0]
    except subprocess.TimeoutExpired:
        output = stop_probe(process)
        error = "검사 제한 시간을 초과했습니다. 종료를 시도했습니다. 라이다 회전 정지는 직접 확인하세요"
    except KeyboardInterrupt:
        output = stop_probe(process)
        error = "사용자가 검사를 중단했습니다"
        interrupted = True
    log_path.write_text(output, encoding="utf-8")
    records = []
    for line in output.splitlines():
        if not line.startswith("{"):
            continue  # SDK의 일반 출력도 log에는 그대로 보존한다.
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            error = "측정 프로그램이 잘못된 JSON을 출력했습니다"
            continue
        if isinstance(record, dict):
            records.append(record)
    if error:
        records.append({"type": "error", "message": error})
    return records, process.returncode if error is None else 1, interrupted


def write_samples(report_dir, sensor, records):
    samples = [r for r in records if r.get("type") == ("imu_sample" if sensor == "imu" else "lidar_scan")]
    with (report_dir / f"{sensor}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        if sensor == "imu":
            writer.writerow(["t_s", "sensor_time", "ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s"])
            for s in samples:
                writer.writerow([s.get("t"), s.get("sensor_time"), *s.get("acc", []), *s.get("gyro", [])])
        else:
            writer.writerow(["t_s", "stamp_ns", "points", "valid_points", "min_range_m", "max_range_m"])
            for s in samples:
                writer.writerow([s.get(k) for k in ("t", "stamp_ns", "points", "valid", "min_range", "max_range")])
    if sensor == "lidar" and samples:
        (report_dir / "last_scan.json").write_text(json.dumps(samples[-1], ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def parser():
    cli = argparse.ArgumentParser(description="ROS 없이 BMI160·YDLIDAR Tmini Plus 검사. IMU 측정 중 로봇을 정지시켜 주세요.")
    cli.add_argument("--unit", default=socket.gethostname(), help="제품 이름/번호 (기본: Pi hostname)")
    cli.add_argument("--duration", type=float, default=8.0, help="센서별 측정 시간, 3~60초 (기본: 8)")
    cli.add_argument("--imu-device", default="/dev/i2c-1")
    cli.add_argument("--imu-address", choices=["auto", "0x68", "0x69"], default="auto")
    cli.add_argument("--lidar-port", help="기본: CP2102 포트 한 개 자동 탐색")
    cli.add_argument("--only", choices=["imu", "lidar"], help="문제 분석을 위해 센서 하나만 검사")
    cli.add_argument("--output", type=Path, default=ROOT / "reports", help="검사 기록 저장 경로")
    return cli


def main(argv=None):
    args = parser().parse_args(argv)
    if not math.isfinite(args.duration) or not 3 <= args.duration <= 60:
        parser().error("--duration은 3~60초입니다")
    sensors = [args.only] if args.only else ["imu", "lidar"]
    binaries = {s: ROOT / "build/bin" / f"{s}_probe" for s in sensors}
    lock_path = ROOT / ".sensor-check.lock"
    with lock_path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("이 도구의 다른 검사가 실행 중입니다.", file=sys.stderr)
            return 2
        try:
            validate_probes(ROOT, sensors)
        except RuntimeError as error:
            print(f"측정 프로그램 준비 오류: {error}\n먼저 ./setup.sh를 실행하세요.", file=sys.stderr)
            return 2
        args.output.mkdir(parents=True, exist_ok=True)
        now = dt.datetime.now().astimezone()
        unit_name = re.sub(r"[^A-Za-z0-9_-]", "_", args.unit)[:64] or "unit"
        report_dir = Path(tempfile.mkdtemp(prefix=f"{now:%Y%m%d-%H%M%S}_{unit_name}_", dir=args.output))
        report = {
            "tool_version": VERSION, "started_at": now.isoformat(), "unit": args.unit,
            "hostname": socket.gethostname(), "duration_per_sensor_s": args.duration,
            "requested_sensors": sensors, "limits": LIMITS,
            "dependencies": {"ydlidar_sdk": SDK_COMMIT, "bosch_sensor_api": BOSCH_COMMIT},
            "sensors": {},
            "manual_checks_remaining": ["IMU 장착 축과 회전 부호", "라이다 앞뒤·좌우와 자체 가림", "실측 물체와 거리 정확도"],
        }
        print(f"제품: {args.unit} / 센서별 {args.duration:g}초 검사")
        print("IMU 측정 중에는 로봇을 정지시켜 주세요. 라이다 검사 중에는 스캐너가 회전합니다.")
        aborted = False
        for sensor in sensors:
            records = []
            print(f"[{sensor.upper()}] 검사 중...", flush=True)
            try:
                if aborted:
                    raise RuntimeError("중단되어 실행하지 않았습니다")
                device = Path(args.imu_device) if sensor == "imu" else select_lidar_port(args.lidar_port)
                preflight(device)
                if sensor == "imu":
                    command = [str(binaries[sensor]), "--device", str(device), "--address", args.imu_address]
                    evaluate = evaluate_imu
                else:
                    command = [str(binaries[sensor]), "--port", str(device)]
                    evaluate = evaluate_lidar
                command += ["--duration", str(args.duration)]
                records, code, interrupted = run_probe(command, report_dir / f"{sensor}.log", args.duration + 25)
                aborted = aborted or interrupted
                outcome = evaluate(records, code, args.duration)
                outcome["device"] = str(device)
                if outcome["checks"].get("sample_format", outcome["checks"].get("scan_format", False)):
                    write_samples(report_dir, sensor, records)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                outcome = result({"preflight_or_execution": False}, {}, [str(error)])
            except KeyboardInterrupt:
                aborted = True
                outcome = result({"not_interrupted": False}, {}, ["사용자가 검사를 중단했습니다"])
            report["sensors"][sensor] = outcome
            print(f"[{sensor.upper()}] {outcome['status']}")
            for key, passed in outcome["checks"].items():
                if not passed:
                    print(f"  실패 항목: {CHECK_LABELS.get(key, key)}")
            for message in outcome["errors"]:
                print(f"  {message}")
            if "rate_hz" in outcome["metrics"]:
                print(f"  실제 수신 주기: {outcome['metrics']['rate_hz']:.2f} Hz")
        report["finished_at"] = dt.datetime.now().astimezone().isoformat()
        report["status"] = "PASS" if not aborted and all(r["status"] == "PASS" for r in report["sensors"].values()) else "FAIL"
        report["aborted"] = aborted
        report_path = report_dir / "report.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        scope = "IMU·라이다" if len(sensors) == 2 else sensors[0].upper()
        print(f"\n{scope} 자동 검사: {report['status']}")
        print(f"기록: {report_path}")
        print("장착 방향·거리 정확도는 별도 현장 확인 항목입니다.")
        return 130 if aborted else (0 if report["status"] == "PASS" else 1)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as error:
        print(f"검사 기록/실행 오류: {error}", file=sys.stderr)
        sys.exit(2)
