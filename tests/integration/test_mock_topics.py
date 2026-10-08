#!/usr/bin/python3
"""Check installed teleop, cmd_vel and odom in an isolated mock ROS domain.

Run in a sourced workspace. This never launches sensor or hardware drivers.
The existing topic checker verifies mock/write-disabled/torque-off settings
before publishing any motion commands. Logs and JSON reports stay in output-dir.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from process_cleanup import cleanup_processes


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=93)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    if not 0 <= args.domain_id <= 232:
        parser.error('domain ID must be between 0 and 232')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(
        ROS_DOMAIN_ID=str(args.domain_id),
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_STATIC_PEERS='',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
    )
    os.environ.pop('ROS_LOCALHOST_ONLY', None)

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.signals import SignalHandlerOptions

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()

    handlers = {sig: signal.signal(sig, interrupted)
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    processes = []
    node = None
    result = 1
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('lekiwi_mock_topics_preflight')
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        others = [name for name in node.get_node_names() if name != node.get_name()]
        if others:
            raise RuntimeError(f'ROS domain is not empty: {others}')
        node.destroy_node()
        node = None
        rclpy.shutdown()

        params = Path(get_package_share_directory('lekiwi_bringup')) / 'param/base.yaml'
        with (output / 'base.log').open('w') as base_log:
            base = subprocess.Popen(
                ['ros2', 'run', 'lekiwi_node', 'lekiwi_node', '--ros-args',
                 '--params-file', str(params), '-p', 'motor_backend:=mock',
                 '-p', 'enable_motor_write:=false', '-p', 'torque_enable:=false',
                 '-p', 'odom_source:=command'],
                stdout=base_log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            processes.append(base)
            report = output / 'report.json'
            checker = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve().parents[2]
                                     / 'lekiwi_bringup/tools/diagnostics/test_ros_topics.py'),
                 '--checks', 'teleop', 'cmd_vel', 'odom', '--duration', '15',
                 '--report', str(report)],
                start_new_session=True,
            )
            processes.append(checker)
            result = checker.wait(timeout=150)
            if base.poll() is not None:
                raise RuntimeError(f'mock base exited early: {base.returncode}')
            if result == 0:
                data = json.loads(report.read_text())
                if not data.get('passed'):
                    raise RuntimeError('topic report did not confirm success')
                print(f'[통과] mock teleop·cmd_vel·odom pub/sub. 결과: {report}', flush=True)
    except KeyboardInterrupt:
        result = 130
        print('mock topic check interrupted', file=sys.stderr)
    except Exception as error:
        result = 1
        print(f'mock topic check failed: {error}', file=sys.stderr)
    finally:
        errors = cleanup_processes([(process, None) for process in processes], interrupt_timeout=15)
        if errors:
            result = 1
            print('mock process cleanup failed: ' + '; '.join(map(str, errors)), file=sys.stderr)
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
