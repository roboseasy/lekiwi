"""Motor service and readiness checks with mocked ROS clients and no hardware."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from std_msgs.msg import Bool

from lekiwi_teleop import motor_session as session


class TestMotorSession(unittest.TestCase):
    def test_readiness_reset_requires_a_new_observation(self):
        endpoint = session.RosMotorEndpoint()
        self.assertIsNone(endpoint.motor_ready)
        endpoint._on_motor_ready(Bool(data=True))
        self.assertIs(endpoint.motor_ready, True)
        endpoint.begin_ready_observation()
        self.assertIsNone(endpoint.motor_ready)
        with patch.object(session.rclpy, 'ok', return_value=False):
            self.assertFalse(endpoint.wait_for_motor_ready(0.1))
        endpoint._on_motor_ready(Bool(data=False))
        self.assertIs(endpoint.motor_ready, False)

    def test_safe_cmd_vel_keeps_separate_discovery_and_response_limits(self):
        client = Mock()
        client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        with patch.object(session.time, 'monotonic') as clock, \
                patch.object(session.rclpy, 'spin_until_future_complete') as spin:
            session.request_motor_power('node', client, False)
        clock.assert_not_called()
        client.wait_for_service.assert_called_once_with(timeout_sec=2.0)
        spin.assert_called_once_with('node', client.call_async.return_value, timeout_sec=3.0)
        self.assertFalse(client.call_async.call_args.args[0].data)

    def test_invalid_total_budget_cannot_send_a_power_request(self):
        for budget in (0.0, -1.0, float('nan'), float('inf')):
            with self.subTest(budget=budget):
                client = Mock()
                with self.assertRaisesRegex(RuntimeError, 'budget'):
                    session.request_motor_power('node', client, True, timeout_sec=budget)
                client.wait_for_service.assert_not_called()
                client.call_async.assert_not_called()

    def test_unavailable_service_does_not_send_a_power_request(self):
        client = Mock()
        client.wait_for_service.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            session.request_motor_power('node', client, True)
        client.call_async.assert_not_called()

    def test_missing_or_rejected_response_is_not_accepted_as_disarmed(self):
        for response in (None, SimpleNamespace(success=False, message='rejected')):
            with self.subTest(response=response):
                client = Mock()
                client.call_async.return_value.result.return_value = response
                with patch.object(session.rclpy, 'spin_until_future_complete'), \
                        self.assertRaisesRegex(RuntimeError, 'failed'):
                    session.request_motor_power('node', client, False)


if __name__ == '__main__':
    unittest.main()
