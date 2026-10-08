"""Failure cases that must not produce a successful integration report."""

import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from sensor_msgs.msg import CompressedImage, LaserScan

spec = importlib.util.spec_from_file_location(
    'topic_checks', Path(__file__).resolve().parents[2] / 'tools/diagnostics/test_ros_topics.py')
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)


def scan():
    message = LaserScan()
    message.header.frame_id = 'lidar_link'
    message.header.stamp.sec = 1
    message.range_min, message.range_max = .05, 12.
    message.angle_increment, message.scan_time = .01, .1
    message.ranges = [.1, .2, float('inf')]
    return message


class TestMockGraphDiscovery(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        clock = patch.object(checks.time, 'monotonic', side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        self.runner = checks.TopicChecks.__new__(checks.TopicChecks)
        self.runner.args = SimpleNamespace(discovery_timeout=.1)
        self.runner.node = Mock()
        self.runner.node.count_publishers.return_value = 0
        self.parameters = dict(motor_backend='mock', enable_motor_write=False,
                               torque_enable=False, odom_source='command', cmd_vel_timeout=.3)
        self.runner.controller_parameters = Mock(return_value=self.parameters)
        self.runner.streams = {'odom': SimpleNamespace(count=1)}
        self.runner.endpoints = Mock(return_value={
            'subscribers': ['/lekiwi_topic_test', '_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_']})
        self.runner.spin = Mock(side_effect=self.advance)

    def advance(self, seconds):
        self.now += seconds

    def test_node_names_can_arrive_after_odom(self):
        def discover(seconds):
            self.advance(seconds)
            self.runner.endpoints.return_value = {
                'subscribers': ['/lekiwi_topic_test', '/lekiwi_node']}
        self.runner.spin.side_effect = discover
        self.assertEqual(self.runner.mock_preflight(), self.parameters)
        self.assertGreater(self.now, 0.)

    def test_unresolved_consumers_fail_within_discovery_window(self):
        with self.assertRaisesRegex(RuntimeError, 'exclusive mock cmd_vel subscriber discovery'):
            self.runner.mock_preflight()
        self.assertGreaterEqual(self.now, self.runner.args.discovery_timeout)
        self.assertLessEqual(self.now, self.runner.args.discovery_timeout + .02)

    def test_extra_consumer_is_never_accepted(self):
        self.runner.endpoints.return_value = {
            'subscribers': ['/lekiwi_node', '/lekiwi_topic_test', '/hardware_base']}
        with self.assertRaisesRegex(RuntimeError, 'exclusive mock cmd_vel subscriber discovery'):
            self.runner.mock_preflight()

    def test_publisher_appearing_during_discovery_is_rejected(self):
        def discover(seconds):
            self.advance(seconds)
            self.runner.endpoints.return_value = {
                'subscribers': ['/lekiwi_node', '/lekiwi_topic_test']}
            self.runner.node.count_publishers.return_value = 1
        self.runner.spin.side_effect = discover
        with self.assertRaisesRegex(RuntimeError, 'Another cmd_vel publisher exists'):
            self.runner.mock_preflight()


class TestFailedInputs(unittest.TestCase):
    def test_missing_base_does_not_prevent_lidar_results_in_full_suite(self):
        runner = checks.TopicChecks.__new__(checks.TopicChecks)
        runner.args = SimpleNamespace(checks=['odom', 'lidar'], duration=1., discovery_timeout=1., camera_indices=[])
        runner.streams = {key: Mock(count=1, latest=None) for key in ('cmd', 'odom', 'scan', 'scan_raw')}
        for stream in runner.streams.values():
            stream.result.return_value = {'passed': True}
        runner.topics = {key: '/' + key for key in runner.streams}
        runner.controller_parameters = Mock(side_effect=RuntimeError('base unavailable'))
        runner.spin = Mock()
        runner.endpoints = lambda topic: {'publishers': ['/sensor']}
        results = runner.sensors()
        self.assertTrue(results['lidar']['passed'])
        self.assertFalse(results['odom']['passed'])
        self.assertEqual(results['odom']['error'], 'base unavailable')

    def test_long_receive_gap_fails_despite_high_average_rate(self):
        with patch.object(checks.time, 'monotonic', return_value=100.) as clock:
            stream = checks.Stream(checks.scan_details)
            for index in range(30):
                clock.return_value = 100. + index * .02 + (1. if index >= 15 else 0.)
                message = scan()
                message.header.stamp.sec = 100
                message.header.stamp.nanosec = index * 20_000_000
                stream.receive(message)
            result = stream.result(5.)
        self.assertGreater(result['hz'], 5.)
        self.assertGreater(result['max_receive_gap_s'], 1.)
        self.assertFalse(result['passed'])

    def test_late_burst_does_not_hide_missing_first_half_of_window(self):
        with patch.object(checks.time, 'monotonic', return_value=100.) as clock:
            stream = checks.Stream(checks.scan_details)
            for index in range(20):
                clock.return_value = 102. + index * .01
                message = scan()
                message.header.stamp.nanosec = index * 10_000_000
                stream.receive(message)
            self.assertFalse(stream.result(5.)['passed'])

    def test_continuous_sensor_stream_passes(self):
        with patch.object(checks.time, 'monotonic', return_value=100.) as clock:
            stream = checks.Stream(checks.scan_details)
            for index in range(20):
                clock.return_value = 100. + index * .1
                message = scan()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(
                    100_000_000_000 + index * 100_000_000, 1_000_000_000)
                stream.receive(message)
            self.assertTrue(stream.result(5.)['passed'])

    def test_real_motor_settings_are_rejected(self):
        safe = dict(motor_backend='mock', enable_motor_write=False,
                    torque_enable=False, odom_source='command')
        checks.validate_mock(safe)
        for key, value in [('motor_backend', 'feetech'), ('enable_motor_write', True),
                           ('torque_enable', True), ('odom_source', 'encoder')]:
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                checks.validate_mock({**safe, key: value})

    def test_obstructed_scan_can_pass_reception_without_claiming_accuracy(self):
        result = checks.scan_details(scan())
        self.assertEqual(result['valid_samples'], 2)
        self.assertLess(result['max_m'], .21)

    def test_all_invalid_scan_fails(self):
        message = scan()
        message.ranges = [float('nan'), float('inf'), .001, 13.]
        with self.assertRaisesRegex(RuntimeError, 'No finite ranges'):
            checks.scan_details(message)

    def test_wrong_lidar_frame_fails(self):
        message = scan()
        message.header.frame_id = 'base_link'
        with self.assertRaisesRegex(RuntimeError, 'frame'):
            checks.scan_details(message)

    def test_corrupt_camera_bytes_fail(self):
        message = CompressedImage()
        message.header.frame_id = 'camera'
        message.data = list(b'not a JPEG')
        with self.assertRaisesRegex(RuntimeError, 'decoding failed'):
            checks.image_details(message)

    def test_duplicate_timestamp_fails_even_with_many_messages(self):
        stream = checks.Stream(checks.scan_details)
        for _ in range(20):
            stream.receive(scan())
        result = stream.result(5.)
        self.assertFalse(result['passed'])
        self.assertIn('Non-increasing', result['error'])

    def test_stopped_stream_fails_even_when_historical_rate_is_high(self):
        stream = checks.Stream(checks.scan_details)
        for index in range(20):
            message = scan()
            message.header.stamp.nanosec = index
            stream.receive(message)
        stream.first -= 5
        stream.last -= 5
        self.assertFalse(stream.result(5.)['fresh'])
        self.assertFalse(stream.result(5.)['passed'])

    def test_missing_publisher_is_not_a_pass(self):
        self.assertFalse(checks.Stream().result(5.)['passed'])


if __name__ == '__main__':
    unittest.main()
