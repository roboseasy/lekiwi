"""Map-save regression tests use disposable files and mocked ROS commands only."""

import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class SaveMapTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='lekiwi map test ')
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.root = self.base / 'desktop with spaces' / 'test' / 'lekiwi'
        tools = self.root / 'tools'
        tools.mkdir(parents=True)
        self.script = tools / 'save_map.sh'
        shutil.copy2(ROOT / 'tools/save_map.sh', self.script)
        self.map_dir = self.root.parent / 'maps' / 'lekiwi'
        self.record = self.root / '.colcon/last_map.txt'
        self.bin = self.base / 'mock-bin'
        self.bin.mkdir()
        self.log = self.base / 'calls.jsonl'
        self.environment = dict(os.environ, MOCK_CALLS=str(self.log),
                                MOCK_MAP_DIR=str(self.map_dir),
                                PATH=f'{self.bin}:/usr/bin:/bin')
        self.environment.pop('LEKIWI_MAP_DIR', None)
        self.environment.pop('ROS_DOMAIN_ID', None)
        self.env_file = tools / 'ros_env.sh'
        self.env_file.write_text('''[[ "$1" == pc ]] || return 97
export LEKIWI_MAP_DIR="$MOCK_MAP_DIR"
export ROS_DOMAIN_ID=42
export MOCK_ENV_READY=yes
''')
        ros2 = self.bin / 'ros2'
        ros2.write_text('''#!/usr/bin/python3
import json
import os
from pathlib import Path
import sys
args = sys.argv[1:]
assert os.environ.get('MOCK_ENV_READY') == 'yes'
assert os.environ.get('ROS_DOMAIN_ID') == '42'
with open(os.environ['MOCK_CALLS'], 'a') as log:
    log.write(json.dumps(args) + '\\n')
stage = 'map' if args[:3] == ['run', 'nav2_map_server', 'map_saver_cli'] else 'state'
if stage == 'map':
    assert '-f' in args
    assert 'save_map_timeout:=10.0' in args
    base = Path(args[args.index('-f') + 1])
    extensions = ['yaml', 'pgm']
else:
    assert args[:4] == ['service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState']
    request = json.loads(args[-1])
    assert request['include_unfinished_submaps'] is True
    filename = request['filename']
    assert filename.endswith('.pbstream')
    base = Path(filename[:-len('.pbstream')])
    extensions = ['pbstream']
if os.environ.get('MOCK_FAIL_STAGE') == stage:
    print('mock operation failed', file=sys.stderr)
    sys.exit(int(os.environ.get('MOCK_FAIL_CODE', '7')))
for extension in extensions:
    if os.environ.get('MOCK_SKIP_EXTENSION') == extension:
        continue
    data = b'' if os.environ.get('MOCK_EMPTY_EXTENSION') == extension else (extension + ' map data').encode()
    Path(str(base) + '.' + extension).write_bytes(data)
if stage == 'map':
    print('mock map saved')
elif os.environ.get('MOCK_UNKNOWN_RESPONSE'):
    print('request accepted, no confirmed response')
else:
    print('status=StatusResponse(code=' + os.environ.get('MOCK_RESPONSE_CODE', '0') + ', message="ok")')
''')
        ros2.chmod(0o755)

    def run_script(self, *arguments):
        # The caller intentionally starts outside the checkout with no ROS environment.
        return subprocess.run(['bash', str(self.script), *arguments], cwd=self.base,
                              env=self.environment, capture_output=True, text=True,
                              timeout=10)

    def calls(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def assert_failure(self, result):
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn('Nav2에서 사용할 지도:', result.stdout)

    def seed_record(self):
        self.record.parent.mkdir(parents=True, exist_ok=True)
        self.record.write_text('map_20261007_111111_000000001.yaml\n')
        return self.record.read_bytes()

    def test_sources_environment_and_saves_from_unrelated_directory_with_spaces(self):
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(len(self.calls()), 2)
        for extension in ('yaml', 'pgm', 'pbstream'):
            self.assertTrue((self.map_dir / ('map.' + extension)).stat().st_size)
        self.assertEqual(self.record.read_text(), 'map.yaml\n')
        self.assertIn(str(self.map_dir / 'map.yaml'), result.stdout)

    def test_help_and_invalid_arguments_do_not_prepare_ros_or_maps(self):
        self.env_file.write_text('return 97\n')
        for argument, expected in (('--help', 0), ('-h', 0), ('unexpected.yaml', 2)):
            with self.subTest(argument=argument):
                result = self.run_script(argument)
                self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
                self.assertEqual(self.calls(), [])
                self.assertFalse(self.map_dir.exists())
                self.assertFalse(self.record.exists())

    def test_ros_environment_failure_stops_before_map_directory_or_ros_calls(self):
        self.env_file.write_text('echo "mock missing ROS installation" >&2\nreturn 23\n')
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(result.returncode, 23)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.map_dir.exists())

    def test_unset_map_directory_stops_before_any_ros_call(self):
        self.env_file.write_text('unset LEKIWI_MAP_DIR\n')
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.record.exists())

    def test_empty_map_directory_stops_before_any_ros_call(self):
        self.environment['MOCK_MAP_DIR'] = ''
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.record.exists())

    def test_map_directory_that_is_a_file_stops_before_ros(self):
        self.map_dir.parent.mkdir(parents=True)
        self.map_dir.write_text('user file')
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(self.calls(), [])
        self.assertEqual(self.map_dir.read_text(), 'user file')

    def test_map_saver_failure_preserves_last_selection_and_skips_state_export(self):
        old_record = self.seed_record()
        self.environment['MOCK_FAIL_STAGE'] = 'map'
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(result.returncode, 7)
        self.assertEqual(len(self.calls()), 1)
        self.assertEqual(self.record.read_bytes(), old_record)

    def test_timeout_exit_is_not_reported_as_success(self):
        for stage in ('map', 'state'):
            with self.subTest(stage=stage):
                self.environment.update(MOCK_FAIL_STAGE=stage, MOCK_FAIL_CODE='124')
                result = self.run_script()
                self.assert_failure(result)
                self.assertEqual(result.returncode, 124)
                self.assertFalse(self.record.exists())

    def test_state_timeout_rejects_success_text_and_partial_artifact(self):
        old_record = self.seed_record()
        timeout = self.bin / 'timeout'
        timeout.write_text('''#!/bin/bash
[[ "$1" == 60 ]] || exit 97
shift
"$@"
exit 124
''')
        timeout.chmod(0o755)
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(result.returncode, 124)
        self.assertIn('StatusResponse(code=0,', result.stdout)
        self.assertTrue((self.map_dir / 'map.pbstream').stat().st_size)
        self.assertEqual(self.record.read_bytes(), old_record)

    def test_state_export_failure_keeps_occupancy_map_but_not_selection(self):
        old_record = self.seed_record()
        self.environment['MOCK_FAIL_STAGE'] = 'state'
        result = self.run_script()
        self.assert_failure(result)
        self.assertTrue((self.map_dir / 'map.yaml').is_file())
        self.assertTrue((self.map_dir / 'map.pgm').is_file())
        self.assertFalse((self.map_dir / 'map.pbstream').exists())
        self.assertEqual(self.record.read_bytes(), old_record)

    def test_missing_or_empty_artifact_does_not_replace_last_selection(self):
        old_record = self.seed_record()
        for flag in ('MOCK_SKIP_EXTENSION', 'MOCK_EMPTY_EXTENSION'):
            for extension in ('yaml', 'pgm', 'pbstream'):
                with self.subTest(flag=flag, extension=extension):
                    self.environment.pop('MOCK_SKIP_EXTENSION', None)
                    self.environment.pop('MOCK_EMPTY_EXTENSION', None)
                    self.environment[flag] = extension
                    result = self.run_script()
                    self.assert_failure(result)
                    self.assertEqual(self.record.read_bytes(), old_record)

    def test_service_application_failure_is_rejected_even_with_nonempty_artifacts(self):
        old_record = self.seed_record()
        self.environment['MOCK_RESPONSE_CODE'] = '2'
        result = self.run_script()
        self.assert_failure(result)
        for extension in ('yaml', 'pgm', 'pbstream'):
            self.assertTrue((self.map_dir / ('map.' + extension)).stat().st_size)
        self.assertEqual(self.record.read_bytes(), old_record)

    def test_unconfirmed_service_response_does_not_select_map(self):
        self.environment['MOCK_UNKNOWN_RESPONSE'] = 'yes'
        result = self.run_script()
        self.assert_failure(result)
        self.assertFalse(self.record.exists())

    def test_state_request_preserves_apostrophes_in_map_path(self):
        self.map_dir = self.root.parent / "operator's maps" / 'lekiwi'
        self.environment['MOCK_MAP_DIR'] = str(self.map_dir)
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        request = json.loads(self.calls()[1][-1])
        self.assertEqual(request['filename'], str(self.map_dir / 'map.pbstream'))
        self.assertTrue((self.map_dir / 'map.pbstream').is_file())

    def test_existing_map_artifacts_are_preserved_and_new_name_is_selected(self):
        self.map_dir.mkdir(parents=True)
        original = {}
        for extension in ('yaml', 'pgm', 'pbstream'):
            path = self.map_dir / ('map.' + extension)
            path.write_bytes(('original ' + extension).encode())
            original[path] = path.read_bytes()
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for path, contents in original.items():
            self.assertEqual(path.read_bytes(), contents)
        selected = self.record.read_text().strip()
        self.assertRegex(selected, r'^map_[0-9]{8}_[0-9]{6}_[0-9]+\.yaml$')
        self.assertTrue((self.map_dir / selected).is_file())

    def test_existing_dangling_symlink_is_preserved(self):
        self.map_dir.mkdir(parents=True)
        original = self.map_dir / 'map.yaml'
        original.symlink_to(self.base / 'missing-user-map')
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(original.is_symlink())
        self.assertEqual(original.readlink(), self.base / 'missing-user-map')
        self.assertNotEqual(self.record.read_text(), 'map.yaml\n')

    def test_generated_name_collision_preserves_files_and_stops(self):
        self.map_dir.mkdir(parents=True)
        (self.map_dir / 'map.yaml').write_text('old default')
        collision = self.map_dir / 'map_20261007_112233_000000001.pgm'
        collision.write_text('old timestamp')
        date = self.bin / 'date'
        date.write_text('#!/bin/sh\nprintf "%s\\n" 20261007_112233_000000001\n')
        date.chmod(0o755)
        result = self.run_script()
        self.assert_failure(result)
        self.assertEqual(self.calls(), [])
        self.assertEqual(collision.read_text(), 'old timestamp')
        self.assertEqual((self.map_dir / 'map.yaml').read_text(), 'old default')

    def test_busy_save_lock_stops_before_any_ros_command(self):
        self.map_dir.mkdir(parents=True)
        with (self.map_dir / '.save-map.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_script()
        self.assert_failure(result)
        self.assertIn('다른 지도 저장', result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertFalse(self.record.exists())

    def test_success_atomically_replaces_previous_map_selection(self):
        self.seed_record()
        result = self.run_script()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.record.read_text(), 'map.yaml\n')
        self.assertEqual(list(self.record.parent.glob('.last-map.*')), [])


if __name__ == '__main__':
    unittest.main()
