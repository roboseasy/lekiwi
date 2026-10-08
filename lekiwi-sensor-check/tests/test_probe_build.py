"""Native build validation and cleanup without sensors or system installs."""
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import probe_build

ROOT = Path(__file__).resolve().parents[1]
SDK_COMMIT = "42a82ed10d2304094c111fc63dee8e4a229b79b7"


class ProbeValidationTests(unittest.TestCase):
    def fixture(self, root, content=b"\x7fELFfixture"):
        program = root / "build/bin/imu_probe"
        program.parent.mkdir(parents=True)
        program.write_bytes(content)
        program.chmod(0o755)
        return program

    def test_empty_or_wrong_format_stops_before_execution(self):
        for content in (b"", b"not an executable"):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.fixture(root, content)
                with patch.object(probe_build.subprocess, "run") as run:
                    with self.assertRaises(RuntimeError):
                        probe_build.validate_probes(root, ("imu",))
                    run.assert_not_called()

    def test_wrong_architecture_and_timeout_are_preparation_errors(self):
        for error in (OSError(8, "Exec format error"), subprocess.TimeoutExpired("probe", 5)):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.fixture(root)
                with patch.object(probe_build.subprocess, "run", side_effect=error):
                    with self.assertRaisesRegex(RuntimeError, "실행 검증 실패"):
                        probe_build.validate_probes(root, ("imu",))

    def test_nonzero_or_wrong_help_output_is_rejected(self):
        for status, output in ((1, b"imu_probe --fixture"), (0, b""), (0, b"other_program --fixture")):
            with self.subTest(status=status, output=output), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                self.fixture(root)
                with patch.object(probe_build.subprocess, "run", return_value=subprocess.CompletedProcess([], status, output)):
                    with self.assertRaises(RuntimeError):
                        probe_build.validate_probes(root, ("imu",))

    def test_clean_removes_only_build_including_bad_objects(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("build/bin/imu_probe", "build/CMakeFiles/imu_probe.dir/native/imu_probe.cpp.o",
                         "build/.remote-ready", "reports/report.json", ".deps/sdk/source.cpp", "native/imu_probe.cpp"):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(name)
            probe_build.clean_build(root)
            self.assertFalse((root / "build").exists())
            for name in ("reports/report.json", ".deps/sdk/source.cpp", "native/imu_probe.cpp"):
                self.assertEqual((root / name).read_text(), name)
            self.assertFalse(list(root.glob("build.*")))

    def test_clean_refuses_symlink_to_external_build(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "source"
            external = Path(temporary) / "external"
            root.mkdir()
            external.mkdir()
            (external / "keep").write_text("preserve")
            (root / "build").symlink_to(external, target_is_directory=True)
            with self.assertRaises(RuntimeError):
                probe_build.clean_build(root)
            self.assertEqual((external / "keep").read_text(), "preserve")


class SetupCommandTests(unittest.TestCase):
    def fixture(self, base):
        root = base / "sensor tool"
        root.mkdir()
        for name in ("setup.sh", "probe_build.py"):
            shutil.copy2(ROOT / name, root / name)
        shutil.copytree(ROOT / "vendor", root / "vendor")
        (root / ".deps/YDLidar-SDK").mkdir(parents=True)
        binaries = base / "commands"
        binaries.mkdir()
        git = binaries / "git"
        git.write_text('#!/bin/sh\ncase "$3" in\nrev-parse) echo ' + SDK_COMMIT + ';;\nstatus) :;;\nesac\n')
        git.chmod(0o755)
        # Compile a device-free ELF fixture for the post-build --help validation.
        source = base / "fixture.cpp"
        source.write_text('#include <iostream>\n#include <string>\nint main(int argc, char** argv) {'
                          'if(argc != 2 || std::string(argv[1]) != "--help") return 1; '
                          'std::string path(argv[0]); '
                          'std::cout << path.substr(path.find_last_of("/")+1) << " --fixture\\n"; return 0;}\n')
        executable = base / "fixture"
        subprocess.run(["g++", str(source), "-o", str(executable)], check=True, capture_output=True)
        cmake = binaries / "cmake"
        cmake.write_text('''#!/usr/bin/python3
import os
from pathlib import Path
import shutil
import sys
args = sys.argv[1:]
if args[0] == '-S':
    build = Path(args[args.index('-B')+1])
    assert not build.exists(), 'the old build was reused'
    build.mkdir()
else:
    build = Path(args[1])
    (build/'bin').mkdir()
    for sensor in ('imu', 'lidar'):
        target = build/'bin'/f'{sensor}_probe'
        if os.environ.get('TEST_BAD_OUTPUT'):
            target.touch(); target.chmod(0o755)
        else:
            shutil.copy2(os.environ['TEST_PROBE_FIXTURE'], target)
''')
        cmake.chmod(0o755)
        environment = dict(os.environ, PATH=str(binaries) + os.pathsep + os.environ['PATH'],
                           TEST_PROBE_FIXTURE=str(executable))
        return root, environment

    def test_every_setup_removes_stale_objects_without_backup(self):
        with tempfile.TemporaryDirectory(prefix="probe setup ") as temporary:
            root, environment = self.fixture(Path(temporary))
            for _ in range(2):
                stale = root / "build/CMakeFiles/lidar_probe.dir/native/lidar_probe.cpp.o"
                stale.parent.mkdir(parents=True, exist_ok=True)
                stale.touch()
                result = subprocess.run(["bash", str(root / "setup.sh")], env=environment, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertFalse(stale.exists())
                self.assertFalse(list(root.glob("build.*")))
                probe_build.validate_probes(root)

    def test_zero_output_is_never_reported_as_install_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root, environment = self.fixture(Path(temporary))
            environment['TEST_BAD_OUTPUT'] = 'yes'
            result = subprocess.run(["bash", str(root / "setup.sh")], env=environment, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            self.assertNotIn('설치 완료.', result.stdout)
            self.assertFalse((root / "build/.remote-ready").exists())

    def test_measurement_lock_prevents_build_deletion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root, environment = self.fixture(Path(temporary))
            (root / "build").mkdir()
            sentinel = root / "build/keep"
            sentinel.write_text("active build")
            for path in (root / ".sensor-check.lock", root / "build/run.lock"):
                with self.subTest(path=path), path.open('a') as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    result = subprocess.run(["bash", str(root / "setup.sh")], env=environment, capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                    self.assertEqual(sentinel.read_text(), "active build")


if __name__ == "__main__":
    unittest.main()
