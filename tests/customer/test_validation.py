"""Exercise first-use commands without ROS, networking, or physical hardware."""

import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[2]
MOCK = r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import subprocess
import sys

command = Path(sys.argv[0]).name
args = sys.argv[1:]
with open(os.environ['CALL_LOG'], 'a') as stream:
    stream.write(json.dumps([command, *args]) + '\n')
if command == 'ros2':
    if args[:2] == ['node', 'list']:
        print(os.environ.get('NODES', '/lekiwi_node'))
    elif args[:2] == ['topic', 'hz']:
        if not os.environ.get('NO_DATA'):
            print('average rate: 10.0')
    elif args[:2] == ['topic', 'echo']:
        if args[2] == '/motor_ready':
            print('data: ' + os.environ.get('MOTOR_READY', 'false'))
        elif not os.environ.get('NO_DATA'):
            print('width: 87\nheight: 103')
    elif args[:2] == ['lifecycle', 'get']:
        print(os.environ.get('LIFECYCLE', 'active [3]'))
    elif args[:2] == ['topic', 'info']:
        print('Publisher count: ' + os.environ.get('PUBLISHERS', '1'))
        print('Node name: ' + os.environ.get('PUBLISHER_NODE', 'collision_monitor'))
    elif args[:3] == ['run', 'tf2_ros', 'tf2_echo']:
        if not os.environ.get('NO_DATA'):
            print('Translation: [0.0, 0.0, 0.0]')
elif command == 'timeout':
    status = subprocess.run(args[1:]).returncode
    if status:
        sys.exit(status)
    sys.exit(124 if 'hz' in args or 'tf2_echo' in args else 0)
elif command == 'nmcli':
    if 'IN-USE,SSID,MODE' in args:
        print('IN-USE SSID MODE\n* lekiwi-ap AP')
    elif '-t' in args:
        print(os.environ.get('WIRED_DEVICES', 'eno9:ethernet:connected'))
    else:
        print('DEVICE TYPE STATE CONNECTION\nwlan0 wifi connected fixture')
elif command == 'ip':
    if args[:2] == ['route', 'get']:
        print(args[2] + ' dev ' + os.environ.get('ROUTE_DEVICE', 'wlan0') + ' src 192.168.0.81')
    elif '-o' in args:
        print('2: eno9 inet ' + os.environ.get('CIDR', '10.42.0.1/24') + ' scope global eno9')
elif command == 'ssh':
    if args[-1].startswith('ssh-keygen -lf'):
        print('256 ' + os.environ.get('TRUSTED_KEY', 'SHA256:fixtureKEY') + ' root@lekiwi01 (ED25519)')
    elif os.environ.get('SSH_FAIL'):
        print('Host key verification failed.', file=sys.stderr)
        sys.exit(255)
    else:
        print('lekiwi01')
elif command == 'ssh-keyscan':
    print(args[-1] + ' ssh-ed25519 FIXTURE-KEY')
elif command == 'ssh-keygen':
    if args[0] == '-lf':
        print('256 ' + os.environ.get('SCANNED_KEY', 'SHA256:fixtureKEY') + ' (ED25519)')
elif command == 'fuser':
    sys.exit(0 if os.environ.get('BUSY_DEVICE') else 1)
elif command == 'timedatectl':
    print('NTPSynchronized=' + os.environ.get('NTP', 'yes'))
elif command == 'sudo':
    sys.exit(subprocess.run(args).returncode)
'''


class ValidationCommandsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='lekiwi-validation-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.root = self.directory / 'desktop test/lekiwi_ws/src/lekiwi'
        self.tools = self.root / 'tools'
        self.tools.mkdir(parents=True)
        self.bin = self.directory / 'bin'
        self.bin.mkdir()
        self.log = self.directory / 'calls.jsonl'
        self.maps = self.directory / 'maps with spaces'
        self.maps.mkdir()
        self.sysnet = self.directory / 'sysnet'
        (self.sysnet / 'wlan0/wireless').mkdir(parents=True)
        self.devices = self.directory / 'serial/by-id'
        self.devices.mkdir(parents=True)
        self.motor = self.devices / 'usb-1a86_USB_Single_Serial_fixture-if00'
        self.lidar = self.devices / 'usb-CP2102-fixture'
        self.i2c = self.directory / 'i2c-1'
        for path in (self.motor, self.lidar, self.i2c):
            path.touch()
        (self.root.parents[1] / 'base_profile.yaml').write_text('fixture: true\n')
        self.sshdir = self.directory / 'ssh'
        self.sshdir.mkdir()
        self.known_hosts = self.sshdir / 'known_hosts'
        self.known_hosts.write_text('existing unrelated key\n')
        # Redirect only the disposable copy's filesystem checks; never change HOME.
        script = (SOURCE / 'tools/validation.sh').read_text()
        script = script.replace('/sys/class/net/', str(self.sysnet) + '/')
        script = script.replace('serial_root=/dev/serial/by-id',
                                'serial_root=' + shlex.quote(str(self.devices)))
        script = script.replace('/dev/i2c-1', shlex.quote(str(self.i2c)))
        script = script.replace('$HOME/.ssh/known_hosts', str(self.known_hosts))
        script = script.replace('$HOME/.ssh', str(self.sshdir))
        self.command = self.tools / 'validation.sh'
        self.command.write_text(script)
        self.command.chmod(0o755)
        (self.tools / 'ros_env.sh').write_text(
            '[[ -z "${FAIL_ENV:-}" ]] || return 1\n'
            'export LEKIWI_MAP_DIR="$TEST_MAP_DIR" ROS_DOMAIN_ID=42\n'
            'echo "ROS fixture: $1"\n')
        (self.tools / 'prepare_ros.py').write_text(
            'import os, sys\n'
            'if sys.argv[1:] == ["--target"]:\n'
            '    print(os.environ.get("TARGET", "roboseasy@10.42.0.136"))\n'
            'else:\n'
            '    print("prepare " + " ".join(sys.argv[1:]))\n')
        for name in ('ros2', 'timeout', 'nmcli', 'ip', 'ssh', 'ssh-keyscan',
                     'ssh-keygen', 'fuser', 'timedatectl', 'sudo', 'ping', 'nmap', 'systemctl'):
            path = self.bin / name
            path.write_text(MOCK)
            path.chmod(0o755)
        self.environment = dict(os.environ, PATH=f'{self.bin}:/usr/bin:/bin',
                                CALL_LOG=str(self.log), TEST_MAP_DIR=str(self.maps))
        for key in ('MOTOR_PORT', 'LIDAR_PORT', 'LEKIWI_MAP_DIR'):
            self.environment.pop(key, None)

    def run_command(self, *arguments, **environment):
        return subprocess.run([str(self.command), *arguments], cwd=self.directory,
                              env=dict(self.environment, **environment),
                              text=True, capture_output=True, timeout=15)

    def calls(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def launches(self):
        return [call for call in self.calls() if call[:2] == ['ros2', 'launch']]

    def write_map(self, name='map.yaml'):
        path = self.maps / name
        path.write_text('image: map.pgm\n')
        (self.maps / 'map.pgm').write_text('P2\n1 1\n255\n0\n')
        return path

    def write_wifi(self, filename, address='192.168.0.206'):
        state = self.root / '.colcon'
        state.mkdir(exist_ok=True)
        (state / filename).write_text(f'roboseasy@10.42.0.136 {address}\n')

    def test_help_and_extra_arguments_do_not_access_devices_or_network(self):
        self.assertEqual(self.run_command('--help').returncode, 0)
        self.assertNotEqual(self.run_command('bringup', '104', 'ignored').returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_folder_command_enters_checkout_from_new_terminal(self):
        result = self.run_command('folder')
        moved = subprocess.run(['bash', '-ec', result.stdout + '\npwd -P'],
                               cwd=self.directory, text=True, capture_output=True)
        self.assertEqual(moved.returncode, 0, moved.stderr)
        self.assertEqual(moved.stdout.strip(), str(self.root))

    def test_prepare_forwards_one_stage_without_ros_runtime(self):
        result = self.run_command('prepare', '--pc-only')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('prepare --pc-only', result.stdout)
        self.assertEqual(self.calls(), [])

    def test_scan_uses_actual_ethernet_device_and_network(self):
        result = self.run_command('scan')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(['nmap', '-sn', '-e', 'eno9', '10.42.0.0/24'], self.calls())

    def test_ambiguous_devices_or_large_subnet_never_start_scan(self):
        for environment in ({'WIRED_DEVICES': 'eno9:ethernet:connected\neno8:ethernet:connected'},
                            {'CIDR': '10.42.0.1/16'}):
            with self.subTest(environment=environment):
                self.assertNotEqual(self.run_command('scan', **environment).returncode, 0)
                self.assertFalse(any(call[0] == 'nmap' for call in self.calls()))

    def test_bringup_selects_devices_with_torque_disabled(self):
        result = self.run_command('bringup', '105')
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = self.launches()[0]
        self.assertIn('torque_enable:=false', launch)
        self.assertIn('imu_i2c_address:=105', launch)
        self.assertIn(f'serial_port:={self.motor}', launch)
        self.assertIn(f'lidar_port:={self.lidar}', launch)

    def test_missing_or_busy_devices_prevent_bringup(self):
        self.assertNotEqual(self.run_command('bringup', BUSY_DEVICE='yes').returncode, 0)
        self.motor.unlink()
        self.assertNotEqual(self.run_command('bringup').returncode, 0)
        self.assertEqual(self.launches(), [])

    def test_duplicate_port_is_rejected_before_bringup(self):
        result = self.run_command('bringup', MOTOR_PORT=str(self.motor), LIDAR_PORT=str(self.motor))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.launches(), [])

    def test_pc_cannot_apply_pi_wifi_settings(self):
        relocated = self.directory / 'laptop clone'
        shutil.copytree(self.root, relocated)
        result = subprocess.run([str(relocated / 'tools/validation.sh'), 'wifi-apply'],
                                env=self.environment, text=True, capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_nav2_selects_last_successful_map_in_current_checkout(self):
        path = self.write_map('map_20261007_123456_123.yaml')
        state = self.root / '.colcon'
        state.mkdir()
        (state / 'last_map.txt').write_text(path.name + '\n')
        result = self.run_command('nav2')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f'map:={path}', self.launches()[0])
        self.assertIn('auto_arm_motors:=true', self.launches()[0])
        self.assertIn('motor_guard_duration:=3600', self.launches()[0])

    def test_map_preview_disables_automatic_arming(self):
        path = self.write_map()
        result = self.run_command('nav2-view', str(path))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('auto_arm_motors:=false', self.launches()[0])

    def test_missing_yaml_image_or_invalid_record_prevents_nav2(self):
        self.assertNotEqual(self.run_command('nav2').returncode, 0)
        path = self.write_map()
        (self.maps / 'map.pgm').unlink()
        self.assertNotEqual(self.run_command('nav2', str(path)).returncode, 0)
        state = self.root / '.colcon'
        state.mkdir()
        (state / 'last_map.txt').write_text('../../other/map.yaml\n')
        self.assertNotEqual(self.run_command('nav2').returncode, 0)
        self.assertEqual(self.launches(), [])

    def test_running_slam_navigation_or_teleop_prevents_new_navigation(self):
        path = self.write_map()
        for node in ('/cartographer_node', '/amcl', '/bt_navigator',
                     '/lekiwi_nav2_motor_guard', '/lekiwi_guarded_base_teleop', '/rviz2'):
            with self.subTest(node=node):
                self.assertNotEqual(self.run_command('nav2', str(path), NODES=node).returncode, 0)
        self.assertEqual(self.launches(), [])

    def test_slam_and_teleop_keep_existing_launch_options(self):
        self.assertEqual(self.run_command('slam').returncode, 0)
        self.assertIn('configuration_basename:=lekiwi_2d_imu.lua', self.launches()[0])
        self.assertEqual(self.run_command('teleop').returncode, 0)
        teleop = [call for call in self.calls() if call[:3] == ['ros2', 'run', 'lekiwi_teleop']][0]
        self.assertIn('--yes-i-confirm-motion', teleop)
        self.assertEqual(teleop[-2:], ['--max-session-duration', '3600'])

    def test_teleop_is_blocked_while_nav2_is_running(self):
        result = self.run_command('teleop', NODES='/bt_navigator')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(call[:2] == ['ros2', 'run'] for call in self.calls()))

    def test_timeout_with_received_data_is_accepted(self):
        result = self.run_command('check')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('브링업 입력 확인 완료', result.stdout)

    def test_timeout_without_data_never_reports_success(self):
        result = self.run_command('check', NO_DATA='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('브링업 입력 확인 완료', result.stdout)

    def test_safe_check_rejects_armed_motors_or_unfinished_nav2(self):
        for environment in ({'MOTOR_READY': 'true'}, {'NODES': '/bt_navigator'}):
            with self.subTest(environment=environment):
                self.assertNotEqual(self.run_command('safe-check', **environment).returncode, 0)

    def test_nav_check_requires_active_nodes_and_only_collision_monitor(self):
        self.assertEqual(self.run_command('nav-check', MOTOR_READY='true').returncode, 0)
        for environment in ({'LIFECYCLE': 'inactive [2]'}, {'PUBLISHERS': '2'},
                            {'PUBLISHER_NODE': 'unexpected_teleop'}):
            with self.subTest(environment=environment):
                result = self.run_command('nav-check', MOTOR_READY='true', **environment)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn('Nav2 준비 상태 확인 완료', result.stdout)

    def test_failed_ros_environment_prevents_runtime_start(self):
        result = self.run_command('slam', FAIL_ENV='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_wifi_verification_persists_address_for_new_terminal(self):
        result = self.run_command('wifi-check', '192.168.0.206')
        self.assertEqual(result.returncode, 0, result.stderr)
        result = self.run_command('ssh', 'wifi')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(['ssh', 'roboseasy@192.168.0.206'], self.calls())

    def test_wifi_show_includes_active_network_for_detected_interface(self):
        result = self.run_command('wifi-show')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('lekiwi-ap', result.stdout)
        self.assertIn(['ip', '-br', '-4', 'addr', 'show', 'dev', 'wlan0'], self.calls())

    def test_wired_route_or_invalid_address_never_verifies_wifi(self):
        self.assertNotEqual(self.run_command('wifi-check', '192.168.0.206', ROUTE_DEVICE='eno9').returncode, 0)
        self.assertNotEqual(self.run_command('wifi-check', 'not-an-ip').returncode, 0)
        self.assertFalse((self.root / '.colcon/wifi_verified.txt').exists())
        self.assertFalse(any(call[0] == 'ssh' for call in self.calls()))

    def test_key_warning_records_candidate_but_does_not_verify_it(self):
        result = self.run_command('wifi-check', '192.168.0.206', SSH_FAIL='yes')
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue((self.root / '.colcon/wifi_candidate.txt').exists())
        self.assertFalse((self.root / '.colcon/wifi_verified.txt').exists())

    def test_verified_address_cannot_be_reused_after_robot_or_candidate_changes(self):
        self.write_wifi('wifi_verified.txt')
        self.write_wifi('wifi_candidate.txt', '192.168.0.210')
        self.assertNotEqual(self.run_command('ssh', 'wifi').returncode, 0)
        self.write_wifi('wifi_candidate.txt')
        self.assertNotEqual(self.run_command('ssh', 'wifi', TARGET='roboseasy@10.42.0.86').returncode, 0)
        self.assertFalse(any(call[0] == 'ssh' for call in self.calls()))

    def test_ssh_reset_preserves_known_hosts_when_fingerprints_differ(self):
        self.write_wifi('wifi_candidate.txt')
        before = self.known_hosts.read_bytes()
        result = self.run_command('ssh-reset', SCANNED_KEY='SHA256:differentKEY')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.known_hosts.read_bytes(), before)
        self.assertFalse(any('-R' in call for call in self.calls()))

    def test_ssh_reset_changes_only_verified_ip_after_key_comparison(self):
        self.write_wifi('wifi_candidate.txt')
        result = self.run_command('ssh-reset')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(['ssh-keygen', '-f', str(self.known_hosts), '-R', '192.168.0.206'], self.calls())
        self.assertIn('existing unrelated key', self.known_hosts.read_text())
        self.assertIn('192.168.0.206 ssh-ed25519 FIXTURE-KEY', self.known_hosts.read_text())

    def test_time_requires_positive_synchronization_result(self):
        self.assertEqual(self.run_command('time').returncode, 0)
        self.assertNotEqual(self.run_command('time', NTP='no').returncode, 0)


if __name__ == '__main__':
    unittest.main()
