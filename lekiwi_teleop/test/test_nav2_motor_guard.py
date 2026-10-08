from types import SimpleNamespace
from unittest.mock import patch

import pytest

from lekiwi_teleop import nav2_motor_guard as guard


def endpoint(name):
    return SimpleNamespace(node_namespace="/", node_name=name)


class GraphNode:
    def __init__(self, publishers, subscribers):
        self.publishers = publishers
        self.subscribers = subscribers

    def get_publishers_info_by_topic(self, topic):
        assert topic == "/cmd_vel"
        return [endpoint(name) for name in self.publishers]

    def get_subscriptions_info_by_topic(self, topic):
        assert topic == "/cmd_vel"
        return [endpoint(name) for name in self.subscribers]


def test_nav2_guard_rejects_other_velocity_publishers_and_missing_base():
    guard.check_command_graph(GraphNode(["collision_monitor"], ["lekiwi_node"]))
    with pytest.raises(RuntimeError, match="only /collision_monitor"):
        guard.check_command_graph(GraphNode(
            ["collision_monitor", "lekiwi_guarded_base_teleop"], ["lekiwi_node"]
        ))
    with pytest.raises(RuntimeError, match="only /collision_monitor"):
        guard.check_command_graph(GraphNode(
            ["collision_monitor", "collision_monitor"], ["lekiwi_node"]
        ))
    with pytest.raises(RuntimeError, match="only /lekiwi_node"):
        guard.check_command_graph(GraphNode(["collision_monitor"], []))


def test_nav2_guard_requires_real_encoder_base_with_initial_torque_off():
    def values(backend="feetech", writes=True, odom="encoder", torque=False):
        return [
            SimpleNamespace(string_value=backend),
            SimpleNamespace(bool_value=writes),
            SimpleNamespace(string_value=odom),
            SimpleNamespace(bool_value=torque),
        ]

    guard.check_hardware_parameters(values())
    for invalid in (
        values(backend="mock"), values(writes=False),
        values(odom="command"), values(torque=True),
    ):
        with pytest.raises(RuntimeError):
            guard.check_hardware_parameters(invalid)


def test_nav2_guard_requires_recent_scan_and_odom():
    class Node:
        def create_subscription(self, _type, topic, callback, _qos):
            self.callbacks[topic] = callback
            return object()

        callbacks = {}

    current = [0.0]
    observer = guard.SensorObserver(Node(), clock=lambda: current[0])
    assert not observer.fresh()
    Node.callbacks["/scan_navigation"](object())
    Node.callbacks["/odom"](object())
    assert observer.fresh()
    current[0] = 1.1
    assert not observer.fresh()


def test_nav2_guard_disarms_even_when_preflight_fails():
    class FakeEndpoint:
        last = None

        def __init__(self, **_kwargs):
            FakeEndpoint.last = self
            self.node = None
            self.power_calls = []

        def initialize(self):
            self.node = object()

        def set_motor_power(self, enabled):
            self.power_calls.append(enabled)

        def destroy(self):
            pass

    with patch.object(guard, "RosMotorEndpoint", FakeEndpoint), \
            patch.object(guard.rclpy, "init"), \
            patch.object(guard.rclpy, "try_shutdown"), \
            patch.object(guard, "publish_zero"), \
            patch.object(guard, "supervise", side_effect=RuntimeError("preflight")):
        result = guard.main(["--yes-i-confirm-motion"])
        assert result == 1
        assert FakeEndpoint.last.power_calls == [False]


def test_nav2_guard_waits_for_navigation_command_path_before_power_on():
    now = [0.0]

    class Node(GraphNode):
        def __init__(self):
            super().__init__([], ["lekiwi_node"])

        def get_publishers_info_by_topic(self, topic):
            self.publishers = ["collision_monitor"] if now[0] >= 0.5 else []
            return super().get_publishers_info_by_topic(topic)

    class Endpoint:
        def __init__(self):
            self.node = Node()
            self.motor_ready = False
            self.power_on_at = None

        def begin_ready_observation(self):
            pass

        def set_motor_power(self, enabled):
            assert enabled is True
            self.power_on_at = now[0]
            self.motor_ready = True

        def wait_for_motor_ready(self, _timeout):
            return True

    endpoint = Endpoint()

    def spin(_node, timeout_sec):
        now[0] += timeout_sec

    with patch.object(guard, "SensorObserver") as observer, \
            patch.object(guard.rclpy, "ok", return_value=True), \
            patch.object(guard.rclpy, "spin_once", side_effect=spin), \
            patch.object(guard, "read_hardware_parameters"), \
            patch.object(guard, "publish_zero"):
        observer.return_value.fresh.return_value = True
        guard.supervise(endpoint, duration=0.1, startup_timeout=1.0, clock=lambda: now[0])

    assert endpoint.power_on_at is not None
    assert endpoint.power_on_at >= 0.5


@pytest.mark.parametrize('use_process_argv', [False, True])
def test_nav2_guard_accepts_ros_arguments_added_by_launch(use_process_argv):
    command = ['--yes-i-confirm-motion', '--startup-timeout', '60',
               '--max-session-duration', '3600', '--ros-args', '-r',
               '__node:=lekiwi_nav2_motor_guard']
    process_args = ['lekiwi_nav2_motor_guard', *command]
    motor = SimpleNamespace(node=object())
    motor.initialize = lambda: None
    motor.destroy = lambda: None
    motor.set_motor_power = lambda enabled: None
    with patch.object(guard.sys, 'argv', process_args), \
            patch.object(guard, 'RosMotorEndpoint', return_value=motor), \
            patch.object(guard.rclpy, 'init') as initialize, \
            patch.object(guard.rclpy, 'try_shutdown'), \
            patch.object(guard, 'publish_zero'), \
            patch.object(guard, 'supervise') as supervise:
        assert guard.main(None if use_process_argv else command) == 0
        supervise.assert_called_once_with(motor, 3600.0, 60.0)
        initialize.assert_called_once_with(args=process_args,
                                           signal_handler_options=guard.SignalHandlerOptions.NO)


@pytest.mark.parametrize('command', [
    ['--ros-args', '-r', '__node:=lekiwi_nav2_motor_guard'],
    ['--yes-i-confirm-motion', '--unknown-option', '--ros-args', '-r', '__node:=guard'],
    ['--yes-i-confirm-motion', '--startup-timeout', '-1', '--ros-args', '-r', '__node:=guard'],
])
def test_nav2_guard_ros_arguments_do_not_bypass_required_or_validated_options(command):
    with patch.object(guard.rclpy, 'init') as initialize, \
            patch.object(guard, 'RosMotorEndpoint') as endpoint, \
            pytest.raises(SystemExit) as error:
        guard.main(command)
    assert error.value.code == 2
    initialize.assert_not_called()
    endpoint.assert_not_called()
