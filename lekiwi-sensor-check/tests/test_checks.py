"""실물 없이 판정 오류·포트 선택·시간 제한·기록 저장을 검사한다."""
import copy
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_sensors as check


def imu_records(duration=3, rate=100):
    records = [{"type": "imu_info", "chip_id": 209, "address": 104}]
    for i in range(int(duration * rate)):
        records.append({"type": "imu_sample", "t": i / rate, "sensor_time": (i * 256 + 2**24 - 512) % 2**24,
                        "acc": [0.001 * (i % 3), 0.0, 9.80665 + 0.001 * (i % 3)],
                        "gyro": [0.001 * (i % 3), 0.0, 0.001], "raw": [i % 3, 0, 16384, i % 3, 0, 1]})
    records.append({"type": "cleanup", "ok": True})
    return records


def lidar_records(duration=3, rate=10):
    records = [{"type": "lidar_info", "model_code": 151, "serial": "test"}]
    for i in range(int(duration * rate)):
        records.append({"type": "lidar_scan", "t": i / rate, "stamp_ns": 1000000000 + i * 100000000,
                        "points": 100, "valid": 100, "min_range": 1.0, "max_range": 1.001,
                        "ranges": [[j * 0.05 - 2.5, 1.0 + (i % 3) * 0.001] for j in range(100)]})
    records.append({"type": "cleanup", "ok": True})
    return records


class ImuChecks(unittest.TestCase):
    def test_healthy_data_and_sensor_time_wrap_pass(self):
        self.assertEqual(check.evaluate_imu(imu_records(), 0, 3)["status"], "PASS")

    def test_chip_id_alone_does_not_pass(self):
        records = [{"type": "imu_info", "chip_id": 209, "address": 104}, {"type": "cleanup", "ok": True}]
        self.assertEqual(check.evaluate_imu(records, 0, 3)["status"], "FAIL")

    def test_poll_loop_with_old_sensor_data_fails(self):
        records = imu_records()
        for r in records[1:-1]:
            r["sensor_time"] = 10
        self.assertFalse(check.evaluate_imu(records, 0, 3)["checks"]["fresh_samples"])

    def test_raw_bias_is_not_hidden(self):
        records = imu_records()
        for r in records[1:-1]:
            r["gyro"][0] += 0.10
        self.assertFalse(check.evaluate_imu(records, 0, 3)["checks"]["gyro_raw_bias"])

    def test_motion_fails_stationary_check(self):
        records = imu_records()
        for i, r in enumerate(records[1:-1]):
            r["acc"][0] = 1.0 if i % 2 else -1.0
        self.assertFalse(check.evaluate_imu(records, 0, 3)["checks"]["stationary_acceleration"])

    def test_frozen_raw_values_fail_even_with_new_timestamps(self):
        records = imu_records()
        for r in records[1:-1]:
            r["raw"] = [0, 0, 16384, 0, 0, 0]
        self.assertFalse(check.evaluate_imu(records, 0, 3)["checks"]["raw_data_changes"])

    def test_low_rate_fails(self):
        self.assertEqual(check.evaluate_imu(imu_records(rate=50), 0, 3)["status"], "FAIL")

    def test_missing_cleanup_and_nonzero_exit_fail(self):
        self.assertEqual(check.evaluate_imu(imu_records()[:-1], 0, 3)["status"], "FAIL")
        self.assertEqual(check.evaluate_imu(imu_records(), 1, 3)["status"], "FAIL")

    def test_nan_or_missing_fields_fail_cleanly(self):
        for bad in ([float("nan"), 0.0, 9.8], None, [0.0, 9.8]):
            records = imu_records()
            records[1]["acc"] = bad
            self.assertEqual(check.evaluate_imu(records, 0, 3)["status"], "FAIL")


class LidarChecks(unittest.TestCase):
    def test_valid_scans_pass(self):
        self.assertEqual(check.evaluate_lidar(lidar_records(), 0, 3)["status"], "PASS")

    def test_duplicate_scan_timestamps_fail(self):
        records = lidar_records()
        records[2]["stamp_ns"] = records[1]["stamp_ns"]
        self.assertFalse(check.evaluate_lidar(records, 0, 3)["checks"]["fresh_scans"])

    def test_empty_ranges_fail(self):
        records = lidar_records()
        for r in records[1:-1]:
            r["valid"] = 0
            r["ranges"] = []
        self.assertEqual(check.evaluate_lidar(records, 0, 3)["status"], "FAIL")

    def test_frozen_ranges_fail(self):
        records = lidar_records()
        for r in records[1:-1]:
            r["ranges"] = copy.deepcopy(records[1]["ranges"])
        self.assertFalse(check.evaluate_lidar(records, 0, 3)["checks"]["range_data_changes"])

    def test_wrong_model_and_short_measurement_fail(self):
        records = lidar_records()
        records[0]["model_code"] = 150
        self.assertEqual(check.evaluate_lidar(records, 0, 3)["status"], "FAIL")
        self.assertEqual(check.evaluate_lidar(lidar_records(duration=1), 0, 3)["status"], "FAIL")

    def test_reported_sdk_failure_cannot_pass(self):
        records = lidar_records() + [{"type": "error", "message": "연결 오류"}]
        self.assertEqual(check.evaluate_lidar(records, 0, 3)["status"], "FAIL")


class DeviceAndExecutionChecks(unittest.TestCase):
    def test_regular_file_cannot_be_opened_as_sensor(self):
        with tempfile.NamedTemporaryFile() as handle:
            with self.assertRaisesRegex(RuntimeError, "장치 파일"):
                check.preflight(Path(handle.name))

    def test_busy_device_is_rejected(self):
        with patch("check_sensors.subprocess.run", return_value=subprocess.CompletedProcess([], 0, "", "")):
            with self.assertRaisesRegex(RuntimeError, "사용 중"):
                check.preflight(Path("/dev/null"))

    def test_actual_fuser_allows_unused_file_and_rejects_open_file(self):
        # 센서 대신 일반 파일로 실제 fuser 호출을 검증한다.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "probe"
            path.touch()
            with patch("check_sensors.stat.S_ISCHR", return_value=True):
                check.preflight(path)
                with path.open():
                    with self.assertRaisesRegex(RuntimeError, "사용 중"):
                        check.preflight(path)

    def test_motor_acm_port_is_rejected_even_if_cp2102(self):
        with tempfile.TemporaryDirectory() as directory:
            port = Path(directory) / "ttyACM0"
            port.touch()
            with patch("check_sensors.usb_identity", return_value=("10c4", "ea60")):
                with self.assertRaisesRegex(RuntimeError, "모터 포트"):
                    check.select_lidar_port(str(port))

    def test_multiple_cp2102_ports_require_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "ttyUSB0").touch()
            (root / "ttyUSB1").touch()
            with patch("check_sensors.usb_identity", return_value=("10c4", "ea60")):
                with self.assertRaises(RuntimeError):
                    check.select_lidar_port(dev_root=root)
                self.assertEqual(check.select_lidar_port(str(root / "ttyUSB1")), root / "ttyUSB1")

    def test_same_device_aliases_are_not_counted_twice(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            port = root / "ttyUSB0"
            port.touch()
            by_id = root / "serial/by-id"
            by_id.mkdir(parents=True)
            (by_id / "CP2102_a").symlink_to(port)
            (by_id / "CP2102_b").symlink_to(port)
            with patch("check_sensors.usb_identity", return_value=("10c4", "ea60")):
                self.assertEqual(check.select_lidar_port(dev_root=root).resolve(), port)

    def test_timeout_is_failure_even_if_cleanup_exits_zero(self):
        code = ('import signal,sys,time; '
                'signal.signal(signal.SIGTERM,lambda a,b:sys.exit(0)); '
                'print(\'{"type":"cleanup","ok":true}\',flush=True); time.sleep(30)')
        with tempfile.TemporaryDirectory() as directory:
            records, status, interrupted = check.run_probe([sys.executable, "-c", code], Path(directory) / "probe.log", 0.2)
            self.assertEqual(status, 1)
            self.assertFalse(interrupted)
            self.assertTrue(any(r.get("type") == "error" for r in records))

    def test_failed_preflight_is_saved_for_both_sensors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            for name in ("imu", "lidar"):
                binary = root / f"build/bin/{name}_probe"
                binary.write_text("unused")
                binary.chmod(0o755)
            with patch.object(check, "ROOT", root), patch.object(check, "validate_probes"), patch("check_sensors.select_lidar_port", side_effect=RuntimeError("포트 없음")), contextlib.redirect_stdout(io.StringIO()):
                status = check.main(["--unit", "fixture", "--imu-device", str(root / "missing"), "--output", str(root / "reports")])
            self.assertEqual(status, 1)
            report = json.loads(next((root / "reports").glob("*/report.json")).read_text())
            self.assertEqual(report["status"], "FAIL")
            self.assertEqual(set(report["sensors"]), {"imu", "lidar"})
            self.assertEqual(report["unit"], "fixture")

    def test_full_report_and_csv_pipeline_with_synthetic_helpers(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            for sensor, records in (("imu", imu_records()), ("lidar", lidar_records())):
                binary = root / f"build/bin/{sensor}_probe"
                output = "\n".join(json.dumps(r) for r in records) + "\n"
                binary.write_text("#!/usr/bin/python3\nimport sys\nsys.stdout.write(" + repr(output) + ")\n")
                binary.chmod(0o755)
            with patch.object(check, "ROOT", root), patch.object(check, "validate_probes"), patch("check_sensors.preflight"), patch("check_sensors.select_lidar_port", return_value=Path("/dev/ttyUSB_fixture")), contextlib.redirect_stdout(io.StringIO()):
                status = check.main(["--unit", "fixture", "--duration", "3", "--output", str(root / "reports")])
            self.assertEqual(status, 0)
            report_path = next((root / "reports").glob("*/report.json"))
            report = json.loads(report_path.read_text())
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(len((report_path.parent / "imu.csv").read_text().splitlines()), 301)
            self.assertEqual(len((report_path.parent / "lidar.csv").read_text().splitlines()), 31)
            self.assertTrue((report_path.parent / "last_scan.json").is_file())
            self.assertTrue(report["manual_checks_remaining"])

    def test_interruption_does_not_start_remaining_sensor(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            for sensor in ("imu", "lidar"):
                binary = root / f"build/bin/{sensor}_probe"
                binary.touch()
                binary.chmod(0o755)
            with patch.object(check, "ROOT", root), patch.object(check, "validate_probes"), patch("check_sensors.preflight"), patch("check_sensors.run_probe", return_value=(imu_records(), 1, True)), patch("check_sensors.select_lidar_port") as selected, contextlib.redirect_stdout(io.StringIO()):
                status = check.main(["--unit", "fixture", "--output", str(root / "reports")])
            self.assertEqual(status, 130)
            selected.assert_not_called()
            report = json.loads(next((root / "reports").glob("*/report.json")).read_text())
            self.assertEqual(report["status"], "FAIL")
            self.assertTrue(report["aborted"])

    def test_empty_native_files_stop_before_device_access(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            for sensor in ("imu", "lidar"):
                program = root / f"build/bin/{sensor}_probe"
                program.touch()
                program.chmod(0o755)
            with patch.object(check, "ROOT", root), patch.object(check, "preflight") as preflight, \
                    patch.object(check, "run_probe") as run, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(check.main(["--output", str(root / "reports")]), 2)
            preflight.assert_not_called()
            run.assert_not_called()
            self.assertFalse((root / "reports").exists())


if __name__ == "__main__":
    unittest.main()
