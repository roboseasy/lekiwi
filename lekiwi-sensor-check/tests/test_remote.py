"""실제 SSH·sudo·센서를 실행하지 않고 원격 점검 흐름을 확인한다."""
import contextlib
import fcntl
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_remote as remote
import remote_worker as worker

JOB = "a" * 32


class RemoteChecks(unittest.TestCase):
    def test_launchers_work_after_relocation_without_ros(self):
        with tempfile.TemporaryDirectory(prefix="lekiwi checkout ") as directory:
            root = Path(directory)
            relocated = root / "renamed sensor tool"
            relocated.mkdir()
            for name in ("check_robot.sh", "check_sensors.sh", "check_remote.py", "check_sensors.py", "probe_build.py"):
                shutil.copy2(remote.ROOT / name, relocated / name)
            for location in (remote.ROOT, relocated):
                for launcher in ("check_robot.sh", "check_sensors.sh"):
                    with self.subTest(location=location, launcher=launcher):
                        result = subprocess.run(
                            [str(location / launcher), "--help"], cwd=root,
                            env={"LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"},
                            capture_output=True, text=True, timeout=5)
                        self.assertEqual(result.returncode, 0, result.stderr)
                        self.assertIn("usage:", result.stdout)

    def test_package_only_contains_pi_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            archive = Path(directory) / "sources.tar.gz"
            release = remote.package_sources(archive)
            with tarfile.open(archive) as stream:
                names = stream.getnames()
                self.assertIn("remote_worker.py", names)
                self.assertIn("probe_build.py", names)
                self.assertIn("vendor/bosch_bmi160/LICENSE", names)
                self.assertEqual(stream.getmember("setup.sh").mode & 0o111, 0o111)
                self.assertFalse(any(n.startswith((".git/", ".deps/", "build/", "reports/")) for n in names))
            self.assertEqual(remote.package_sources(archive), release)

    def test_source_content_changes_release(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "source.py").write_text("first")
            with patch.object(remote, "ROOT", root), patch.object(remote, "SOURCES", ("source.py",)):
                first = remote.package_sources(root / "source.tar.gz")
                (root / "source.py").write_text("second")
                self.assertNotEqual(remote.package_sources(root / "source.tar.gz"), first)

    def test_bootstrap_in_path_with_spaces_preserves_cached_release(self):
        with tempfile.TemporaryDirectory(prefix="lekiwi fake home ") as directory:
            root = Path(directory)
            incoming = root / remote.CACHE / "incoming"
            incoming.mkdir(parents=True)
            source = root / "remote_worker.py"
            source.write_text('print("prepared")\n')
            archive = incoming / f"{JOB}.tar.gz"
            command = remote.bootstrap_command("b" * 64, JOB).replace('cd "$HOME"', "cd " + shlex.quote(str(root)), 1)
            for contents in ('print("prepared")\n', 'raise RuntimeError("overwritten")\n'):
                source.write_text(contents)
                with tarfile.open(archive, "w:gz") as stream:
                    stream.add(source, arcname="remote_worker.py")
                result = subprocess.run(["bash", "-c", command], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), "prepared")
            self.assertFalse(archive.exists())

    def test_incomplete_release_stops_bootstrap(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            release = root / remote.CACHE / "releases" / ("b" * 64)
            release.mkdir(parents=True)
            command = remote.bootstrap_command("b" * 64, JOB).replace('cd "$HOME"', "cd " + shlex.quote(str(root)), 1)
            result = subprocess.run(["bash", "-c", command], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("이전 소스 준비", result.stderr)

    def test_invalid_target_is_rejected_before_connecting(self):
        with patch.object(remote.subprocess, "run") as run, contextlib.redirect_stderr(io.StringIO()):
            for values in (("--ip", "127.0.0.1;touch /tmp/x"), ("--ip", "127.0.0.1", "--user", "-oProxyCommand=x")):
                with self.assertRaises(SystemExit):
                    remote.main(list(values))
            run.assert_not_called()

    def simulate(self, code=0, report_status="PASS", mismatch=False, missing_report=False,
                 copy_failure=False, prepare_failure=False, interrupted=False, ip="10.42.0.136"):
        calls = []
        with tempfile.TemporaryDirectory() as directory:
            def fake_run(command, **kwargs):
                calls.append(command)
                if command[0] == "ssh" and "--prepare" in command[-1] and prepare_failure:
                    raise subprocess.CalledProcessError(2, command)
                if command[0] == "ssh" and "--measure" in command[-1]:
                    if interrupted:
                        raise KeyboardInterrupt
                    return subprocess.CompletedProcess(command, 0 if mismatch else code)
                if command[0] == "scp" and "-r" in command:
                    if copy_failure:
                        raise subprocess.CalledProcessError(255, command)
                    destination = Path(command[-1])
                    (destination / "remote_status.json").write_text(json.dumps({"job": JOB, "returncode": code}))
                    if not missing_report:
                        report = destination / "reports/fixture/report.json"
                        report.parent.mkdir(parents=True)
                        report.write_text(json.dumps({"status": report_status}))
                return subprocess.CompletedProcess(command, 0)

            with patch.object(remote.uuid, "uuid4", return_value=uuid.UUID(JOB)), patch.object(remote.subprocess, "run", side_effect=fake_run), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result = remote.main(["--ip", ip, "--output", directory, "--unit", "robot ' $(touch /tmp/no) --prepare"])
            connection = json.loads(next(Path(directory).glob("*/connection.json")).read_text())
        return result, connection, calls

    def test_pass_fetches_results_and_preserves_argument_as_text(self):
        code, connection, calls = self.simulate()
        self.assertEqual(code, 0)
        self.assertEqual(connection["status"], "PASS")
        measure = next(c[-1] for c in calls if c[0] == "ssh" and "--measure" in c[-1])
        parsed = shlex.split(measure.split("exec ", 1)[1])
        self.assertEqual(parsed[parsed.index("--unit") + 1], "robot ' $(touch /tmp/no) --prepare")
        self.assertIn("--", parsed[parsed.index("--job") + 2:])
        self.assertEqual(calls[-1][-3], "-O")

    def test_sensor_failure_still_fetches_report_and_returns_one(self):
        code, connection, calls = self.simulate(code=1, report_status="FAIL")
        self.assertEqual(code, 1)
        self.assertEqual(connection["sensor_returncode"], 1)
        self.assertTrue(any(c[0] == "scp" and "-r" in c for c in calls))

    def test_setup_failure_never_starts_sensor(self):
        code, connection, calls = self.simulate(prepare_failure=True)
        self.assertEqual(code, 2)
        self.assertFalse(any(c[0] == "ssh" and "--measure" in c[-1] for c in calls))

    def test_missing_mismatched_or_uncopied_records_cannot_pass(self):
        for values in ({"code": 1, "report_status": "FAIL", "mismatch": True}, {"missing_report": True},
                       {"report_status": "FAIL"}, {"copy_failure": True}):
            code, connection, _ = self.simulate(**values)
            self.assertEqual(code, 2)
            self.assertEqual(connection["status"], "ERROR")

    def test_interruption_collects_failure_without_restart(self):
        code, connection, calls = self.simulate(code=130, report_status="FAIL", interrupted=True)
        self.assertEqual(code, 130)
        self.assertEqual(connection["status"], "INTERRUPTED")
        self.assertEqual(sum(c[0] == "ssh" and "--measure" in c[-1] for c in calls), 1)

    def test_ipv6_scp_target_is_bracketed(self):
        code, _, calls = self.simulate(ip="::1")
        self.assertEqual(code, 0)
        self.assertTrue(any("roboseasy@[::1]:" in arg for c in calls if c[0] == "scp" for arg in c))


class WorkerChecks(unittest.TestCase):
    def test_private_root_reports_are_returned_to_ssh_account(self):
        with tempfile.TemporaryDirectory() as directory:
            job = Path(directory)
            report = job / "reports/private-run"
            report.mkdir(parents=True, mode=0o700)
            (report / "report.json").write_text('{"status":"FAIL"}')
            (report / "imu.csv").write_text('t_s,ax\n')
            owner = job.stat()
            child = Mock()
            child.wait.return_value = 1
            original_umask = os.umask(0o022)
            try:
                with patch.object(worker.os, "geteuid", return_value=0), patch.object(worker.subprocess, "Popen", return_value=child), patch.object(worker.os, "chown") as chown:
                    self.assertEqual(worker.measure(job, JOB, []), 1)
                    changed = {Path(call.args[0]) for call in chown.call_args_list}
                    self.assertTrue({report, report / "report.json", report / "imu.csv", job / "remote_status.json"} <= changed)
                    for call in chown.call_args_list:
                        self.assertEqual(call.args[1:], (owner.st_uid, owner.st_gid))
                        self.assertFalse(call.kwargs["follow_symlinks"])
            finally:
                os.umask(original_umask)

    def test_existing_remote_lock_stops_second_check(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory)
            with (cache / "remote.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.object(worker, "CACHE", cache), patch.object(worker, "prepare") as prepare, contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(worker.main(["--prepare", "--job", JOB]), 2)
                    prepare.assert_not_called()

    def test_ready_release_skips_install_and_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            (root / "build/.remote-ready").touch()
            for sensor in ("imu", "lidar"):
                binary = root / f"build/bin/{sensor}_probe"
                binary.write_bytes(b"\x7fELFfixture")
                binary.chmod(0o755)
            def help_result(command, **kwargs):
                return subprocess.CompletedProcess(command, 0, Path(command[0]).name.encode() + b" --fixture\n", b"")
            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", return_value="/usr/bin/tool"), \
                    patch.object(worker.subprocess, "run", side_effect=help_result) as run:
                worker.prepare(root / "job")
                self.assertEqual([call.args[0] for call in run.call_args_list],
                                 [[str(root / "build/bin/imu_probe"), "--help"],
                                  [str(root / "build/bin/lidar_probe"), "--help"]])
                self.assertTrue((root / "job").is_dir())

    def test_missing_packages_are_installed_before_build(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build").mkdir()
            def tool(name):
                return None if name == "gcc" else "/usr/bin/" + name
            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", side_effect=tool), \
                    patch.object(worker.subprocess, "run") as run, patch.object(worker, "validate_probes"), \
                    patch.object(worker.os, "sync"), contextlib.redirect_stdout(io.StringIO()):
                worker.prepare(root / "job")
                self.assertEqual(run.call_args_list[0].args[0], ["sudo", "--", "apt-get", "update"])
                self.assertEqual(run.call_args_list[-1].args[0], ["bash", str(root / "setup.sh")])

    def test_new_release_build_failure_does_not_mark_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", return_value="tool"), patch.object(worker.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "setup")):
                with self.assertRaises(subprocess.CalledProcessError):
                    worker.prepare(root / "job")
            self.assertFalse((root / "build/.remote-ready").exists())

    def test_empty_cached_probes_trigger_rebuild(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            (root / "build/.remote-ready").touch()
            for sensor in ("imu", "lidar"):
                binary = root / f"build/bin/{sensor}_probe"
                binary.touch()
                binary.chmod(0o755)

            def build_or_help(command, **kwargs):
                if command[0] == "bash":
                    for sensor in ("imu", "lidar"):
                        (root / f"build/bin/{sensor}_probe").write_bytes(b"\x7fELFfixture")
                return subprocess.CompletedProcess(command, 0, Path(command[0]).name.encode() + b" --fixture\n", b"")

            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", return_value="/usr/bin/tool"), \
                    patch.object(worker.subprocess, "run", side_effect=build_or_help) as run, patch.object(worker.os, "sync"):
                worker.prepare(root / "job")
                run.assert_any_call(["bash", str(root / "setup.sh")], check=True)

    def test_failed_rebuild_clears_existing_ready_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            marker = root / "build/.remote-ready"
            marker.touch()
            for sensor in ("imu", "lidar"):
                program = root / f"build/bin/{sensor}_probe"
                program.touch()
                program.chmod(0o755)
            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", return_value="tool"), \
                    patch.object(worker.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "setup")):
                with self.assertRaises(subprocess.CalledProcessError):
                    worker.prepare(root / "job")
            self.assertFalse(marker.exists())
            self.assertFalse((root / "job").exists())

    def test_successful_build_command_with_empty_outputs_does_not_mark_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "build/bin").mkdir(parents=True)
            for sensor in ("imu", "lidar"):
                program = root / f"build/bin/{sensor}_probe"
                program.touch()
                program.chmod(0o755)
            with patch.object(worker, "ROOT", root), patch.object(worker.shutil, "which", return_value="tool"), \
                    patch.object(worker.subprocess, "run", return_value=subprocess.CompletedProcess([], 0)):
                with self.assertRaises(RuntimeError):
                    worker.prepare(root / "job")
            self.assertFalse((root / "build/.remote-ready").exists())
            self.assertFalse((root / "job").exists())

    def test_hangup_and_term_only_interrupt_own_child_group(self):
        original_umask = os.umask(0o022)
        try:
            for sig in (signal.SIGHUP, signal.SIGTERM):
                with tempfile.TemporaryDirectory() as directory:
                    job = Path(directory)
                    child = Mock(pid=9876)
                    child.poll.return_value = None
                    def wait():
                        signal.getsignal(sig)(sig, None)
                        return 130
                    child.wait.side_effect = wait
                    with patch.object(worker.os, "geteuid", return_value=0), patch.object(worker.subprocess, "Popen", return_value=child), patch.object(worker.os, "killpg") as kill:
                        self.assertEqual(worker.measure(job, JOB, []), 130)
                        kill.assert_called_once_with(9876, signal.SIGINT)
                    self.assertEqual(json.loads((job / "remote_status.json").read_text())["returncode"], 130)
        finally:
            os.umask(original_umask)

    def test_real_worker_hangup_stops_synthetic_measurement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "job").mkdir()
            helper = root / "check_sensors.py"
            helper.write_text('import signal,sys,time\nfrom pathlib import Path\n'
                              f'root=Path({str(root)!r})\n'
                              'def stop(a,b):\n root.joinpath("stopped").touch()\n sys.exit(130)\n'
                              'signal.signal(signal.SIGINT,stop)\nroot.joinpath("started").touch()\n'
                              'while True: time.sleep(0.05)\n')
            code = ('import sys; from pathlib import Path; '
                    f'sys.path.insert(0,{str(worker.ROOT)!r}); import remote_worker as w; '
                    f'w.ROOT=Path({str(root)!r}); w.os.geteuid=lambda:0; '
                    f'sys.exit(w.measure(w.ROOT/"job",{JOB!r},[]))')
            process = subprocess.Popen([sys.executable, "-c", code], start_new_session=True)
            try:
                deadline = time.monotonic() + 5
                while not (root / "started").exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue((root / "started").exists())
                process.send_signal(signal.SIGHUP)
                self.assertEqual(process.wait(timeout=5), 130)
                self.assertTrue((root / "stopped").exists())
                self.assertEqual(json.loads((root / "job/remote_status.json").read_text())["returncode"], 130)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    unittest.main()
