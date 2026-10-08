"""Preparation tests use temporary files and fake commands, never Pi/hardware."""

import importlib.util
import hashlib
import io
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tarfile
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


prepare = load_tool('prepare_ros')
wifi = load_tool('configure_wifi')


def fixture_source(root):
    for package in prepare.PACKAGES:
        (root / package).mkdir(parents=True)
        (root / package / 'source.txt').write_text(package)
    for name in prepare.HELPERS:
        destination = root / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, destination)
    shutil.copy2(ROOT / 'lekiwi.repos', root / 'lekiwi.repos')
    (root / 'lekiwi_bringup/param').mkdir()
    (root / 'lekiwi_bringup/param/base_profile.example.yaml').write_text('example profile')


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / 'source'
        fixture_source(self.root)
        self.bundle = self.base / 'source.tar.gz'
        self.destination = self.base / 'pi/src/lekiwi'

    def test_fresh_and_repeat_deployment(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(prepare.source_hash(self.root), prepare.source_hash(self.destination))
        profile = self.destination.parent.parent / 'base_profile.yaml'
        profile.write_text('product calibration')
        prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(profile.read_text(), 'product calibration')

    def test_existing_same_ros_sources_get_helpers(self):
        prepare.make_bundle(self.root, self.bundle)
        self.destination.mkdir(parents=True)
        for name in (*prepare.PACKAGES, 'lekiwi.repos'):
            entry = self.root / name
            if entry.is_dir():
                shutil.copytree(entry, self.destination / name)
            else:
                shutil.copy2(entry, self.destination / name)
        prepare.deploy_bundle(self.bundle, self.destination)
        self.assertTrue((self.destination / 'tools/ros_env.sh').is_file())

    def test_modified_sources_preserved(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        source = self.destination / 'lekiwi_node/source.txt'
        source.write_text('customer changes')
        with self.assertRaisesRegex(RuntimeError, '기존 Pi ROS 소스가 다릅니다'):
            prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(source.read_text(), 'customer changes')

    def test_extra_old_source_is_not_silently_kept(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        (self.destination / 'lekiwi_node/old.cpp').write_text('obsolete source')
        with self.assertRaises(RuntimeError):
            prepare.deploy_bundle(self.bundle, self.destination)

    def test_local_builds_and_sensor_reports_never_bundled(self):
        for name in ('build/bin/native', 'lekiwi-sensor-check/reports/report.json',
                     'lekiwi_node/__pycache__/test.pyc'):
            entry = self.root / name
            entry.parent.mkdir(parents=True, exist_ok=True)
            entry.write_text('private local data')
        prepare.make_bundle(self.root, self.bundle)
        with tarfile.open(self.bundle) as archive:
            names = archive.getnames()
        self.assertFalse(any('reports' in name or '__pycache__' in name or name.startswith('build/') for name in names))

    def test_archive_cannot_escape_destination(self):
        with tarfile.open(self.bundle, 'w:gz') as archive:
            member = tarfile.TarInfo('../escaped')
            member.size = 4
            archive.addfile(member, io.BytesIO(b'nope'))
        with self.assertRaises(RuntimeError):
            prepare.deploy_bundle(self.bundle, self.destination)
        self.assertFalse((self.base / 'escaped').exists())

    def test_existing_symlink_is_preserved(self):
        prepare.make_bundle(self.root, self.bundle)
        self.destination.parent.mkdir(parents=True)
        self.destination.symlink_to(self.root, target_is_directory=True)
        with self.assertRaises(RuntimeError):
            prepare.deploy_bundle(self.bundle, self.destination)
        self.assertTrue(self.destination.is_symlink())

    def test_changed_helper_is_preserved(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        helper = self.destination / 'tools/ros_env.sh'
        helper.write_text('custom environment')
        with self.assertRaises(RuntimeError):
            prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(helper.read_text(), 'custom environment')

    def test_known_released_helper_is_upgraded_and_new_helpers_are_installed(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        helper_name = 'tools/prepare_ros.py'
        helper = self.destination / helper_name
        previous_release = b'known repository-owned preparation helper\n'
        helper.write_bytes(previous_release)
        for name in ('tools/validation.sh', 'tools/save_map.sh'):
            (self.destination / name).unlink()
        released = {helper_name: {hashlib.sha256(previous_release).hexdigest()}}
        with mock.patch.object(prepare, 'RELEASED_HELPERS', released):
            prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(helper.read_bytes(), (self.root / helper_name).read_bytes())
        self.assertEqual(helper.stat().st_mode & 0o111,
                         (self.root / helper_name).stat().st_mode & 0o111)
        self.assertEqual(prepare.source_hash(self.root), prepare.source_hash(self.destination))

    def test_custom_preparation_helper_is_preserved_even_when_other_release_is_allowed(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        helper = self.destination / 'tools/prepare_ros.py'
        helper.write_text('customer customization\n')
        previous_release = b'known repository-owned preparation helper\n'
        released = {'tools/prepare_ros.py': {hashlib.sha256(previous_release).hexdigest()}}
        with mock.patch.object(prepare, 'RELEASED_HELPERS', released):
            with self.assertRaisesRegex(RuntimeError, '기존 파일을 보존'):
                prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(helper.read_text(), 'customer customization\n')

    def test_failed_preflight_does_not_upgrade_release_or_add_missing_helpers(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        helper_name = 'tools/prepare_ros.py'
        helper = self.destination / helper_name
        previous_release = b'known repository-owned preparation helper\n'
        helper.write_bytes(previous_release)
        custom = self.destination / 'tools/configure_wifi.py'
        custom.write_text('customer Wi-Fi settings\n')
        for name in ('tools/validation.sh', 'tools/save_map.sh'):
            (self.destination / name).unlink()
        released = {helper_name: {hashlib.sha256(previous_release).hexdigest()}}
        with mock.patch.object(prepare, 'RELEASED_HELPERS', released):
            with self.assertRaisesRegex(RuntimeError, '기존 파일을 보존'):
                prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(helper.read_bytes(), previous_release)
        self.assertEqual(custom.read_text(), 'customer Wi-Fi settings\n')
        self.assertFalse((self.destination / 'tools/validation.sh').exists())
        self.assertFalse((self.destination / 'tools/save_map.sh').exists())

    def test_symlinked_helper_is_preserved_without_upgrading_other_helper(self):
        prepare.make_bundle(self.root, self.bundle)
        prepare.deploy_bundle(self.bundle, self.destination)
        helper_name = 'tools/prepare_ros.py'
        helper = self.destination / helper_name
        previous_release = b'known repository-owned preparation helper\n'
        helper.write_bytes(previous_release)
        linked = self.destination / 'tools/ros_env.sh'
        linked.unlink()
        outside = self.base / 'personal-environment.sh'
        outside.write_text('personal environment\n')
        linked.symlink_to(outside)
        released = {helper_name: {hashlib.sha256(previous_release).hexdigest()}}
        with mock.patch.object(prepare, 'RELEASED_HELPERS', released):
            with self.assertRaisesRegex(RuntimeError, '기존 파일을 보존'):
                prepare.deploy_bundle(self.bundle, self.destination)
        self.assertEqual(helper.read_bytes(), previous_release)
        self.assertTrue(linked.is_symlink())
        self.assertEqual(outside.read_text(), 'personal environment\n')

    def test_target_is_read_without_running_sensor_script(self):
        config = self.root / 'lekiwi-sensor-check/check_robot.sh'
        config.parent.mkdir()
        sentinel = self.base / 'executed'
        config.write_text(f'ROBOT_IP="10.42.0.85"\nROBOT_USER="roboseasy"\ntouch {sentinel}\n')
        self.assertEqual(prepare.robot_target(self.root), ('roboseasy', '10.42.0.85'))
        self.assertFalse(sentinel.exists())
        config.write_text('ROBOT_IP="$(touch /tmp/should-not-run)"\nROBOT_USER="roboseasy"\n')
        with self.assertRaises(ValueError):
            prepare.robot_target(self.root)


class WifiTests(unittest.TestCase):
    def test_renderer_is_scoped_to_wifi_only(self):
        for renderer in ('NetworkManager', 'networkd'):
            config = wifi.wifi_config('wlan0', 'test network', 'test-key-123', renderer)
            self.assertEqual(set(config['network']), {'version', 'wifis'})
            device = config['network']['wifis']['wlan0']
            self.assertEqual(device['renderer'], renderer)
            self.assertEqual('networkmanager' in device, renderer == 'NetworkManager')

    def test_invalid_credentials(self):
        for ssid, key in (('', '12345678'), ('s' * 33, '12345678'), ('test', 'short'), ('test', 'z' * 64)):
            with self.assertRaises(ValueError):
                wifi.wifi_config('wlan0', ssid, key, 'networkd')

    def test_config_is_private_and_existing_file_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / 'wifi.yaml'
            config = wifi.wifi_config('wlan0', 'test network', 'a' * 64, 'NetworkManager')
            wifi.write_config(target, config)
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            original = target.read_bytes()
            with self.assertRaises(FileExistsError):
                wifi.write_config(target, config)
            self.assertEqual(target.read_bytes(), original)


class InstallerTests(unittest.TestCase):
    def exercise_installer(self, role, fail_sdk=False):
        temporary = tempfile.TemporaryDirectory(prefix='lekiwi setup ')
        self.addCleanup(temporary.cleanup)
        base = Path(temporary.name)
        root = base / ('lekiwi_ws/src/lekiwi' if role == 'pi' else 'lekiwi')
        fixture_source(root)
        workspace = root.parent.parent if role == 'pi' else root
        dependencies = workspace if role == 'pi' else root.parent / 'lekiwi_deps'
        bin_path = base / 'mock-bin'
        bin_path.mkdir()
        log = base / 'commands.log'
        setup = base / 'jazzy.bash'
        setup.write_text(':\n')
        for name in ('YDLidar-SDK', 'ydlidar_ros2_driver'):
            (dependencies / 'src' / name).mkdir(parents=True, exist_ok=True)
        original = (root / 'tools/install_ros.sh').read_text()
        # Redirect OS entry points only in this disposable copy. Nothing is installed.
        original = original.replace('/opt/ros/jazzy/setup.bash', shlex.quote(str(setup)))
        original = original.replace('export PATH=/usr/bin:/bin:/usr/sbin:/sbin',
                                    'export PATH=' + shlex.quote(f'{bin_path}:/usr/bin:/bin'))
        original = original.replace('/etc/ros/rosdep/sources.list.d/20-default.list', shlex.quote(str(base / 'rosdep.list')))
        (root / 'tools/install_ros.sh').write_text(original)
        mock = '''#!/usr/bin/python3
import os
from pathlib import Path
import sys
name = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['MOCK_LOG'], 'a') as log:
    log.write(name + ' ' + ' '.join(args) + '\\n')
ws = Path(os.environ['MOCK_WS'])
deps = Path(os.environ['MOCK_DEPS'])
if name == 'colcon':
    assert not os.environ.get('VIRTUAL_ENV')
    assert os.environ.get('MAKEFLAGS') == '-j2'
    assert '--executor' in args and 'sequential' in args
    index = args.index('--packages-select') + 1
    packages = []
    while index < len(args) and not args[index].startswith('--'):
        packages.append(args[index]); index += 1
    if os.environ.get('MOCK_FAIL_SDK') and packages == ['ydlidar_sdk']:
        sys.exit(7)
    for package in packages:
        parent = deps if package.startswith('ydlidar') else ws
        destination = parent / 'install' / package
        destination.mkdir(parents=True, exist_ok=True)
        (parent / 'install/local_setup.bash').write_text(':\\n')
        if package == 'ydlidar_sdk':
            (destination / 'lib').mkdir()
            (destination / 'lib/libydlidar_sdk.a').write_bytes(b'mock-library')
elif name == 'ros2':
    package = args[-1]
    parent = deps if package.startswith('ydlidar') else ws
    destination = parent / 'install' / package
    if package.startswith('lekiwi') or package.startswith('ydlidar'):
        if not destination.exists(): sys.exit(1)
        print(destination)
    else:
        print('/opt/ros/jazzy')
elif name == 'git' and 'rev-parse' in args:
    print('42a82ed10d2304094c111fc63dee8e4a229b79b7' if 'YDLidar-SDK' in args[1] else '4ef70d3f32a85704ade0be54b214f3763b1ab3e8')
elif name == 'id':
    print('i2c dialout' if '-nG' in args else 'test-user')
'''
        for name in ('sudo', 'rosdep', 'colcon', 'ros2', 'git', 'id', 'getent'):
            entry = bin_path / name
            entry.write_text(mock)
            entry.chmod(0o755)
        environment = dict(os.environ, MOCK_WS=str(workspace), MOCK_DEPS=str(dependencies),
                           MOCK_LOG=str(log), VIRTUAL_ENV='/broken/virtualenv')
        if fail_sdk:
            environment['MOCK_FAIL_SDK'] = 'yes'
        command = ['bash', str(root / 'tools/install_ros.sh'), role]
        if role == 'pi':
            (workspace / 'base_profile.yaml').write_text('product calibration')
        result = subprocess.run(command, env=environment, capture_output=True, text=True)
        return command, environment, result, log, workspace

    def test_build_order_and_idempotent_pc_setup(self):
        command, environment, result, log, workspace = self.exercise_installer('pc')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        builds = [line for line in log.read_text().splitlines() if line.startswith('colcon ')]
        self.assertEqual(len(builds), 3)
        self.assertIn('--packages-select ydlidar_sdk ', builds[0])
        self.assertIn('--packages-select ydlidar_ros2_driver ', builds[1])
        self.assertIn('--packages-select lekiwi ', builds[2])
        repeat = subprocess.run(command, env=environment, capture_output=True, text=True)
        self.assertEqual(repeat.returncode, 0, repeat.stdout + repeat.stderr)
        self.assertIn('설치·빌드를 생략', repeat.stdout)
        self.assertEqual(log.read_text().count('colcon build'), 3)
        self.assertTrue((workspace / '.colcon/lekiwi-setup.ready').is_file())

    def test_sdk_failure_stops_before_driver_and_no_ready_marker(self):
        _, _, result, log, workspace = self.exercise_installer('pc', fail_sdk=True)
        self.assertEqual(result.returncode, 7, result.stdout + result.stderr)
        self.assertEqual(log.read_text().count('colcon build'), 1)
        self.assertFalse((workspace / '.colcon/lekiwi-setup.ready').exists())

    def test_pi_keeps_existing_product_profile(self):
        _, _, result, _, workspace = self.exercise_installer('pi')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((workspace / 'base_profile.yaml').read_text(), 'product calibration')


class RemoteTests(unittest.TestCase):
    def test_failed_upload_closes_connection_and_cleans_only_own_temporary_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'source'
            fixture_source(root)
            config = root / 'lekiwi-sensor-check/check_robot.sh'
            config.parent.mkdir()
            config.write_text('ROBOT_IP="10.42.0.85"\nROBOT_USER="roboseasy"\n')

            def run(command, **kwargs):
                if command[0] == 'scp':
                    raise subprocess.CalledProcessError(1, command)
                return subprocess.CompletedProcess(command, 0)

            with mock.patch.object(prepare.subprocess, 'run', side_effect=run) as calls, \
                    mock.patch.object(prepare.subprocess, 'check_output', return_value='/tmp/lekiwi-ros.abc123\n'):
                with self.assertRaises(subprocess.CalledProcessError):
                    prepare.prepare_pi(root)
            commands = [call.args[0] for call in calls.call_args_list]
            self.assertTrue(any(command[-1] == 'rm -rf -- /tmp/lekiwi-ros.abc123' for command in commands))
            self.assertTrue(any('-O' in command and 'exit' in command for command in commands))
            self.assertFalse(any('flock' in command[-1] for command in commands))


if __name__ == '__main__':
    unittest.main()
