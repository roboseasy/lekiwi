import contextlib
import importlib
import io
import unittest

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from pathlib import Path
from unittest.mock import patch


def load_api():
    try:
        module = importlib.import_module("lekiwi_teleop.guarded_base_teleop")
        return tuple(
            getattr(module, name)
            for name in (
                "GuardedTeleopConfig",
                "Motion",
                "DirectionalSpeedProfile",
                "build_parser",
                "config_from_args",
                "run_guarded_session",
            )
        )
    except (ImportError, AttributeError) as error:
        raise AssertionError("guarded physical teleop API is not implemented") from error


@contextlib.contextmanager
def mock_tk_window():
    import tkinter as tk

    class FakeRoot:
        def title(self, *_args):
            pass

        def geometry(self, *_args):
            pass

        def resizable(self, *_args):
            pass

        def update(self):
            pass

        def destroy(self):
            pass

    class FakeLabel:
        def pack(self, **_kwargs):
            pass

    class FakeStringVar:
        def __init__(self, value):
            self.value = value

        def set(self, value):
            self.value = value

    with (
        patch.object(tk, "Tk", return_value=FakeRoot()),
        patch.object(tk, "Label", return_value=FakeLabel()),
        patch.object(tk, "StringVar", side_effect=FakeStringVar),
    ):
        yield


class FakeEndpoint:
    def __init__(self, *, ready=True, publish_error=None, power_errors=None):
        self.ready = ready
        self.publish_error = publish_error
        self.power_errors = power_errors or {}
        self.events = []
        self.power_timeouts = []

    def initialize(self):
        pass

    def publish_motion(self, motion):
        self.events.append(("publish", motion))
        if self.publish_error is not None:
            raise self.publish_error

    def set_motor_power(self, enabled, timeout_sec=None):
        self.events.append(("power", enabled))
        self.power_timeouts.append((enabled, timeout_sec))
        error = self.power_errors.get(enabled)
        if error is not None:
            raise error

    def begin_ready_observation(self):
        self.events.append(("begin_ready",))

    def wait_for_motor_ready(self, timeout_sec):
        self.events.append(("ready", timeout_sec))
        return self.ready


class TestGuardedBaseTeleop(unittest.TestCase):
    def setUp(self):
        (
            self.Config,
            self.Motion,
            self.DirectionalSpeedProfile,
            self.build_parser,
            self.config_from_args,
            self.run_session,
        ) = load_api()

    def assert_direction_transition_publishes_zero_then_new_axis(
        self,
        old_key,
        new_key,
        old_first_step,
        old_second_step,
        new_first_step,
    ):
        endpoint = FakeEndpoint()
        keys = iter([old_key, None, new_key, None, "\x03"])

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                lambda timeout: next(keys),
                config=self.Config(zero_cycles=1),
                clock=lambda: 0.0,
            )

        published = [
            event[1]
            for event in endpoint.events
            if event[0] == "publish"
        ]
        self.assertEqual(
            published[1:-1],
            [old_first_step, old_second_step, self.Motion(), new_first_step],
        )

    def test_parser_requires_explicit_motion_confirmation(self):
        parser = self.build_parser()

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args([])

        args = parser.parse_args(["--yes-i-confirm-motion"])
        config = self.config_from_args(args)
        self.assertEqual(config.max_session_duration, 3600.0)
        option_strings = {
            option
            for action in parser._actions
            for option in action.option_strings
        }
        self.assertNotIn("--nav2-max-speed", option_strings)
        self.assertNotIn("--gui-deadman", option_strings)

    def test_directional_profile_starts_at_speed_stage_one_stopped(self):
        profile = self.DirectionalSpeedProfile()

        self.assertEqual(profile.speed_stage, 1)
        self.assertEqual(profile.target, self.Motion())
        self.assertEqual(profile.control, self.Motion())

    def test_directional_profile_uses_exact_stage_fractions_and_clamps(self):
        profile = self.DirectionalSpeedProfile()

        profile.handle_key("w")
        self.assertEqual(profile.target, self.Motion(linear_x=0.10 / 3.0))

        profile.handle_key("r")
        self.assertEqual(profile.target, self.Motion(linear_x=2.0 * 0.10 / 3.0))

        profile.handle_key("r")
        profile.handle_key("r")
        self.assertEqual(profile.speed_stage, 3)
        self.assertEqual(profile.target, self.Motion(linear_x=0.10))

        profile.handle_key("f")
        self.assertEqual(profile.target, self.Motion(linear_x=2.0 * 0.10 / 3.0))
        profile.handle_key("f")
        self.assertEqual(profile.target, self.Motion(linear_x=0.10 / 3.0))
        profile.handle_key("f")
        profile.handle_key("f")
        self.assertEqual(profile.speed_stage, 0)
        self.assertEqual(profile.target, self.Motion())

    def test_directional_profile_maps_all_six_directions_with_exact_signs(self):
        expected = {
            "w": self.Motion(linear_x=0.10),
            "s": self.Motion(linear_x=-0.10),
            "a": self.Motion(linear_y=0.10),
            "d": self.Motion(linear_y=-0.10),
            "q": self.Motion(angular_z=0.30),
            "e": self.Motion(angular_z=-0.30),
        }

        for key, motion in expected.items():
            with self.subTest(key=key):
                profile = self.DirectionalSpeedProfile()
                profile.handle_key("r")
                profile.handle_key("r")

                self.assertTrue(profile.handle_key(key))
                self.assertEqual(profile.target, motion)

    def test_direction_key_replaces_previous_direction_with_one_axis(self):
        profile = self.DirectionalSpeedProfile()
        profile.handle_key("r")
        profile.handle_key("r")

        profile.handle_key("w")
        profile.handle_key("q")
        self.assertEqual(profile.target, self.Motion(angular_z=0.30))

        profile.handle_key("d")
        self.assertEqual(profile.target, self.Motion(linear_y=-0.10))

    def test_speed_keys_preserve_selected_direction(self):
        profile = self.DirectionalSpeedProfile()
        profile.handle_key("e")

        self.assertTrue(profile.handle_key("r"))
        self.assertEqual(profile.target, self.Motion(angular_z=-2.0 * 0.30 / 3.0))
        self.assertTrue(profile.handle_key("f"))
        self.assertEqual(profile.target, self.Motion(angular_z=-0.30 / 3.0))

    def test_lowering_to_zero_stops_immediately_and_keeps_direction(self):
        profile = self.DirectionalSpeedProfile()
        profile.handle_key("w")
        profile.next_motion()

        self.assertTrue(profile.handle_key("f"))
        self.assertEqual(profile.speed_stage, 0)
        self.assertEqual(profile.target, self.Motion())
        self.assertEqual(profile.control, self.Motion())
        self.assertEqual(profile.next_motion(), self.Motion())

        profile.handle_key("r")
        self.assertEqual(profile.target, self.Motion(linear_x=0.10 / 3.0))

    def test_directional_profile_steps_each_axis_at_simple_profile_rates(self):
        profile = self.DirectionalSpeedProfile()

        for key, first_step in (
            ("w", self.Motion(linear_x=0.005)),
            ("a", self.Motion(linear_y=0.005)),
            ("q", self.Motion(angular_z=0.05)),
        ):
            with self.subTest(key=key):
                profile = self.DirectionalSpeedProfile()
                profile.handle_key(key)
                self.assertEqual(profile.next_motion(), first_step)

    def test_session_x_to_y_publishes_one_zero_then_only_y(self):
        self.assert_direction_transition_publishes_zero_then_new_axis(
            "w",
            "a",
            self.Motion(linear_x=0.005),
            self.Motion(linear_x=0.01),
            self.Motion(linear_y=0.005),
        )

    def test_session_y_to_x_publishes_one_zero_then_only_x(self):
        self.assert_direction_transition_publishes_zero_then_new_axis(
            "a",
            "w",
            self.Motion(linear_y=0.005),
            self.Motion(linear_y=0.01),
            self.Motion(linear_x=0.005),
        )

    def test_session_translation_to_yaw_publishes_one_zero_then_only_yaw(self):
        self.assert_direction_transition_publishes_zero_then_new_axis(
            "w",
            "q",
            self.Motion(linear_x=0.005),
            self.Motion(linear_x=0.01),
            self.Motion(angular_z=0.05),
        )

    def test_session_yaw_to_translation_publishes_one_zero_then_only_translation(self):
        self.assert_direction_transition_publishes_zero_then_new_axis(
            "q",
            "a",
            self.Motion(angular_z=0.05),
            self.Motion(angular_z=0.30 / 3.0),
            self.Motion(linear_y=0.005),
        )

    def test_session_reversal_publishes_one_zero_then_only_reversed_axis(self):
        self.assert_direction_transition_publishes_zero_then_new_axis(
            "w",
            "s",
            self.Motion(linear_x=0.005),
            self.Motion(linear_x=0.01),
            self.Motion(linear_x=-0.005),
        )

    def test_session_first_direction_profiles_without_zero_transition(self):
        endpoint = FakeEndpoint()
        keys = iter(["w", "\x03"])

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                lambda timeout: next(keys),
                config=self.Config(zero_cycles=1),
                clock=lambda: 0.0,
            )

        published = [
            event[1]
            for event in endpoint.events
            if event[0] == "publish"
        ]
        self.assertEqual(published[1:-1], [self.Motion(linear_x=0.005)])

    def test_session_same_direction_and_stage_keys_do_not_publish_zero_transition(self):
        endpoint = FakeEndpoint()
        keys = iter(["w", "w", "r", "f", "\x03"])

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                lambda timeout: next(keys),
                config=self.Config(zero_cycles=1),
                clock=lambda: 0.0,
            )

        published = [
            event[1]
            for event in endpoint.events
            if event[0] == "publish"
        ]
        self.assertEqual(
            published[1:-1],
            [
                self.Motion(linear_x=0.005),
                self.Motion(linear_x=0.01),
                self.Motion(linear_x=0.015),
                self.Motion(linear_x=0.02),
            ],
        )

    def test_space_sets_stage_zero_and_stops_immediately(self):
        profile = self.DirectionalSpeedProfile()
        profile.handle_key("r")
        profile.handle_key("a")
        profile.next_motion()

        self.assertTrue(profile.handle_key(" "))
        self.assertEqual(profile.speed_stage, 0)
        self.assertEqual(profile.target, self.Motion())
        self.assertEqual(profile.control, self.Motion())
        self.assertEqual(profile.next_motion(), self.Motion())

    def test_directional_profile_rejects_unknown_keys(self):
        profile = self.DirectionalSpeedProfile()

        for key in ("x", "?", ""):
            with self.subTest(key=key):
                self.assertFalse(profile.handle_key(key))
        self.assertEqual(profile.target, self.Motion())

    def test_parser_rejects_non_finite_or_non_positive_duration(self):
        parser = self.build_parser()

        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parser.parse_args(
                        ["--yes-i-confirm-motion", "--max-session-duration", value]
                    )

    def test_session_zeros_then_arms_then_requires_ready_before_reading_keys(self):
        endpoint = FakeEndpoint()
        keys = iter(["w", None, "s", None, "\x03"])
        read_events = []

        def read_key(timeout_sec):
            read_events.append(("read", timeout_sec))
            return next(keys)

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                read_key,
                config=self.Config(zero_cycles=3),
                clock=lambda: 0.0,
            )

        self.assertEqual(
            endpoint.events[:6],
            [
                ("publish", self.Motion()),
                ("publish", self.Motion()),
                ("publish", self.Motion()),
                ("power", True),
                ("begin_ready",),
                ("ready", 5.0),
            ],
        )
        self.assertEqual(endpoint.events[6:10], [
            ("publish", self.Motion(linear_x=0.005)),
            ("publish", self.Motion(linear_x=0.01)),
            ("publish", self.Motion()),
            ("publish", self.Motion(linear_x=-0.005)),
        ])
        self.assertEqual(len(read_events), 5)
        self.assertEqual(endpoint.events[-4:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_session_direction_speed_space_stop_and_idle_publish(self):
        endpoint = FakeEndpoint()
        keys = iter(["w", "r", " ", None, "\x03"])
        config = self.Config(zero_cycles=2)

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                lambda timeout: next(keys),
                config=config,
                clock=lambda: 0.0,
            )

        self.assertEqual(endpoint.events[5:9], [
            ("publish", self.Motion(linear_x=0.005)),
            ("publish", self.Motion(linear_x=0.01)),
            ("publish", self.Motion()),
            ("publish", self.Motion()),
        ])
        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_session_timeout_is_finite_and_cleans_up(self):
        endpoint = FakeEndpoint()
        clock_values = iter([10.0, 10.0, 10.0, 11.0])

        result = self.run_session(
            endpoint,
            lambda timeout: self.fail("timeout must be checked before reading a key"),
            config=self.Config(max_session_duration=1.0, zero_cycles=2),
            clock=lambda: next(clock_values),
        )

        self.assertEqual(result, "timeout")
        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_key_returned_at_deadline_cannot_publish_motion_and_cleans_up(self):
        endpoint = FakeEndpoint()
        now = [10.0]

        def read_key(timeout_sec):
            self.assertEqual(timeout_sec, 0.1)
            now[0] = 11.0
            return "w"

        result = self.run_session(
            endpoint,
            read_key,
            config=self.Config(max_session_duration=1.0, zero_cycles=2),
            clock=lambda: now[0],
        )

        self.assertEqual(result, "timeout")
        self.assertFalse(
            any(
                event[0] == "publish" and event[1] != self.Motion()
                for event in endpoint.events
            )
        )
        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_preparation_uses_only_the_remaining_overall_session_budget(self):
        class DeadlineEndpoint(FakeEndpoint):
            def set_motor_power(self, enabled, timeout_sec=None):
                self.events.append(("power_budget", enabled, timeout_sec))

            def wait_for_motor_ready(self, timeout_sec):
                self.events.append(("ready_budget", timeout_sec))
                return False

        endpoint = DeadlineEndpoint()
        clock_values = iter([10.0, 10.0, 10.75])

        with self.assertRaisesRegex(RuntimeError, "motor_ready"):
            self.run_session(
                endpoint,
                lambda timeout: self.fail("preparation must fail before key input"),
                config=self.Config(max_session_duration=1.0, zero_cycles=1),
                clock=lambda: next(clock_values),
            )

        self.assertIn(("power_budget", True, 1.0), endpoint.events)
        self.assertIn(("ready_budget", 0.25), endpoint.events)

    def test_readiness_failure_prevents_motion_and_cleans_up(self):
        endpoint = FakeEndpoint(ready=False)

        with self.assertRaisesRegex(RuntimeError, "motor_ready"):
            self.run_session(
                endpoint,
                lambda timeout: self.fail("keys must not be read before readiness"),
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
            )

        self.assertFalse(
            any(
                event[0] == "publish" and event[1] != self.Motion()
                for event in endpoint.events
            )
        )
        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_ready_true_observed_during_arm_service_is_not_fresh_enough(self):
        class ArmRaceEndpoint(FakeEndpoint):
            def set_motor_power(self, enabled, timeout_sec=None):
                super().set_motor_power(enabled, timeout_sec=timeout_sec)
                if enabled:
                    self.ready = True

            def begin_ready_observation(self):
                super().begin_ready_observation()
                self.ready = False

        endpoint = ArmRaceEndpoint(ready=False)

        with self.assertRaisesRegex(RuntimeError, "motor_ready"):
            self.run_session(
                endpoint,
                lambda timeout: "\x03",
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
            )

        self.assertLess(
            endpoint.events.index(("power", True)),
            endpoint.events.index(("begin_ready",)),
        )

    def test_arm_service_failure_still_attempts_zero_and_bounded_disarm(self):
        endpoint = FakeEndpoint(power_errors={True: RuntimeError("arm rejected")})

        with self.assertRaisesRegex(RuntimeError, "arm rejected"):
            self.run_session(
                endpoint,
                lambda timeout: self.fail("keys must not be read after arm failure"),
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
            )

        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_ctrl_c_and_other_errors_clean_up(self):
        for original in (KeyboardInterrupt(), ValueError("key reader failed")):
            with self.subTest(original=type(original).__name__):
                endpoint = FakeEndpoint()

                with self.assertRaises(type(original)):
                    self.run_session(
                        endpoint,
                        lambda timeout, error=original: (_ for _ in ()).throw(error),
                        config=self.Config(zero_cycles=2),
                        clock=lambda: 0.0,
                    )

                self.assertEqual(endpoint.events[-3:], [
                    ("publish", self.Motion()),
                    ("publish", self.Motion()),
                    ("power", False),
                ])

    def test_raw_terminal_ctrl_c_byte_raises_and_cleans_up(self):
        endpoint = FakeEndpoint()

        with self.assertRaises(KeyboardInterrupt):
            self.run_session(
                endpoint,
                lambda timeout: "\x03",
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
            )

        self.assertEqual(endpoint.events[-3:], [
            ("publish", self.Motion()),
            ("publish", self.Motion()),
            ("power", False),
        ])

    def test_cleanup_failure_is_reported_without_masking_original_error(self):
        endpoint = FakeEndpoint(power_errors={False: RuntimeError("disarm failed")})
        reports = []

        with self.assertRaisesRegex(ValueError, "key reader failed") as caught:
            self.run_session(
                endpoint,
                lambda timeout: (_ for _ in ()).throw(ValueError("key reader failed")),
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
                report_cleanup_error=reports.append,
            )

        self.assertEqual(len(reports), 1)
        self.assertIn("disarm failed", reports[0])
        self.assertIn("disarm failed", " ".join(getattr(caught.exception, "__notes__", [])))

    def test_cleanup_reporter_failure_does_not_mask_primary_error(self):
        endpoint = FakeEndpoint(power_errors={False: RuntimeError("disarm failed")})
        stderr = io.StringIO()

        with contextlib.redirect_stderr(stderr):
            with self.assertRaisesRegex(ValueError, "primary failure") as caught:
                self.run_session(
                    endpoint,
                    lambda timeout: (_ for _ in ()).throw(ValueError("primary failure")),
                    config=self.Config(zero_cycles=1),
                    clock=lambda: 0.0,
                    report_cleanup_error=lambda message: (_ for _ in ()).throw(
                        RuntimeError("logger failed")
                    ),
                )

        notes = " ".join(getattr(caught.exception, "__notes__", []))
        self.assertIn("disarm failed", notes)
        self.assertIn("logger failed", notes)
        self.assertIn("disarm failed", stderr.getvalue())
        self.assertIn("logger failed", stderr.getvalue())

    def test_cleanup_attempts_disarm_when_zero_publish_fails(self):
        endpoint = FakeEndpoint(publish_error=RuntimeError("zero publish failed"))
        reports = []

        with self.assertRaisesRegex(RuntimeError, "zero publish failed"):
            self.run_session(
                endpoint,
                lambda timeout: "x",
                config=self.Config(zero_cycles=2),
                clock=lambda: 0.0,
                report_cleanup_error=reports.append,
            )

        self.assertIn(("power", False), endpoint.events)
        self.assertTrue(any("zero publish failed" in report for report in reports))


class TestGuardedRosAdapter(unittest.TestCase):
    def setUp(self):
        self.module = importlib.import_module("lekiwi_teleop.guarded_base_teleop")

    def test_main_warning_documents_direction_and_speed_keys(self):
        warnings = []

        class FakeLogger:
            def warning(self, message):
                warnings.append(message)

            def info(self, message):
                pass

        class FakeNode:
            def get_logger(self):
                return FakeLogger()

        class FakeRosEndpoint(FakeEndpoint):
            def __init__(self):
                super().__init__()
                self.node = FakeNode()

            def report_cleanup_error(self, message):
                pass

            def destroy(self):
                pass

        with (
            mock_tk_window(),
            patch.object(self.module.rclpy, "init"),
            patch.object(self.module.rclpy, "try_shutdown"),
            patch.object(self.module, "RosMotorEndpoint", return_value=FakeRosEndpoint()),
            patch.object(self.module, "run_hold_gui_session"),
        ):
            result = self.module.main(["--yes-i-confirm-motion"])

        self.assertEqual(result, 0)
        self.assertEqual(len(warnings), 1)
        for token in ("w/s", "a/d", "q/e", "1/2/3", "Space", "Esc"):
            self.assertIn(token, warnings[0])

    def test_endpoint_uses_exact_guarded_ros_interfaces_and_ready_qos(self):
        class FakeNode:
            def __init__(self):
                self.publisher_args = None
                self.client_args = None
                self.subscription_args = None

            def create_publisher(self, message_type, topic, depth):
                self.publisher_args = (message_type, topic, depth)
                return object()

            def create_client(self, service_type, name):
                self.client_args = (service_type, name)
                return object()

            def create_subscription(self, message_type, topic, callback, qos):
                self.subscription_args = (message_type, topic, callback, qos)
                return object()

        node = FakeNode()
        with patch.object(self.module.rclpy, "create_node", return_value=node):
            endpoint = self.module.RosMotorEndpoint()
            endpoint.initialize()

        self.assertEqual(node.publisher_args[1:], ("/cmd_vel", 10))
        self.assertEqual(node.client_args[1], "/motor_power")
        self.assertEqual(node.subscription_args[1], "/motor_ready")
        qos = node.subscription_args[3]
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertIs(endpoint.node, node)

    def test_motor_power_call_is_bounded_and_requires_success(self):
        class FakeFuture:
            def __init__(self, response):
                self.response = response

            def done(self):
                return True

            def result(self):
                return self.response

        class FakeClient:
            def __init__(self, response):
                self.response = response
                self.wait_timeouts = []
                self.requests = []

            def wait_for_service(self, timeout_sec):
                self.wait_timeouts.append(timeout_sec)
                return True

            def call_async(self, request):
                self.requests.append(request.data)
                return FakeFuture(self.response)

        endpoint = self.module.RosMotorEndpoint.__new__(self.module.RosMotorEndpoint)
        endpoint.node = object()
        success = type("Response", (), {"success": True, "message": "armed"})()
        endpoint.motor_power_client = FakeClient(success)

        with patch.object(self.module.rclpy, "spin_until_future_complete") as spin:
            endpoint.set_motor_power(True)

        self.assertEqual(endpoint.motor_power_client.wait_timeouts, [2.0])
        self.assertEqual(endpoint.motor_power_client.requests, [True])
        spin.assert_called_once_with(endpoint.node, unittest.mock.ANY, timeout_sec=3.0)

        failure = type("Response", (), {"success": False, "message": "rejected"})()
        endpoint.motor_power_client = FakeClient(failure)
        with patch.object(self.module.rclpy, "spin_until_future_complete"):
            with self.assertRaisesRegex(RuntimeError, "rejected"):
                endpoint.set_motor_power(False)

    def test_motor_power_discovery_and_response_share_the_supplied_deadline(self):
        class FakeFuture:
            def done(self):
                return True

            def result(self):
                return type("Response", (), {"success": True, "message": ""})()

        class FakeClient:
            def __init__(self):
                self.wait_timeouts = []

            def wait_for_service(self, timeout_sec):
                self.wait_timeouts.append(timeout_sec)
                return True

            def call_async(self, request):
                return FakeFuture()

        endpoint = self.module.RosMotorEndpoint.__new__(self.module.RosMotorEndpoint)
        endpoint.node = object()
        endpoint.motor_power_client = FakeClient()
        clock_values = iter([20.0, 20.75])

        with (
            patch.object(self.module.time, "monotonic", side_effect=lambda: next(clock_values)),
            patch.object(self.module.rclpy, "spin_until_future_complete") as spin,
        ):
            try:
                endpoint.set_motor_power(True, timeout_sec=1.0)
            except TypeError as error:
                self.fail(f"set_motor_power lacks an overall timeout budget: {error}")

        self.assertEqual(endpoint.motor_power_client.wait_timeouts, [1.0])
        spin.assert_called_once_with(endpoint.node, unittest.mock.ANY, timeout_sec=0.25)

    def test_motor_power_response_timeout_reports_actual_shared_budget(self):
        class PendingFuture:
            def done(self):
                return False

        class FakeClient:
            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, request):
                return PendingFuture()

        endpoint = self.module.RosMotorEndpoint.__new__(self.module.RosMotorEndpoint)
        endpoint.node = object()
        endpoint.motor_power_client = FakeClient()
        clock_values = iter([30.0, 30.75])

        with (
            patch.object(self.module.time, "monotonic", side_effect=lambda: next(clock_values)),
            patch.object(self.module.rclpy, "spin_until_future_complete"),
        ):
            with self.assertRaisesRegex(RuntimeError, "0.25"):
                endpoint.set_motor_power(True, timeout_sec=1.0)

    def test_ready_wait_requires_an_observed_true_sample(self):
        endpoint = self.module.RosMotorEndpoint.__new__(self.module.RosMotorEndpoint)
        endpoint.node = object()
        endpoint._on_motor_ready(Bool(data=True))

        with (
            patch.object(self.module.time, "monotonic", return_value=0.0),
            patch.object(self.module.rclpy, "ok", return_value=True),
            patch.object(
                self.module.rclpy,
                "spin_once",
                side_effect=AssertionError("an already observed true sample must be accepted"),
            ),
        ):
            self.assertTrue(endpoint.wait_for_motor_ready(5.0))

    def test_main_keeps_cleanup_and_destroy_before_idempotent_ros_shutdown(self):
        events = []

        class FailingLogger:
            def warning(self, message):
                raise ValueError("pre-session failure")

            def error(self, message):
                pass

        class FakeNode:
            def get_logger(self):
                return FailingLogger()

        class FakeRosEndpoint(FakeEndpoint):
            def __init__(self):
                super().__init__()
                self.node = FakeNode()

            def destroy(self):
                events.append("destroy_node")

            def report_cleanup_error(self, message):
                pass

        def init(*, args, signal_handler_options):
            self.assertEqual(args, [])
            self.assertIs(
                signal_handler_options,
                self.module.SignalHandlerOptions.NO,
            )
            events.append("init_no_signal_handlers")

        def cleanup(endpoint, zero_cycles):
            events.append("zero_and_motor_off")
            return None

        endpoint = FakeRosEndpoint()
        with (
            mock_tk_window(),
            patch.object(self.module.rclpy, "init", side_effect=init),
            patch.object(
                self.module.rclpy,
                "try_shutdown",
                side_effect=lambda: events.append("try_shutdown"),
            ),
            patch.object(self.module.rclpy, "shutdown") as ros_shutdown,
            patch.object(self.module, "RosMotorEndpoint", return_value=endpoint),
            patch.object(self.module, "cleanup_session", side_effect=cleanup),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            result = self.module.main(["--yes-i-confirm-motion"])

        self.assertEqual(result, 1)
        self.assertEqual(events, [
            "init_no_signal_handlers",
            "zero_and_motor_off",
            "destroy_node",
            "try_shutdown",
        ])
        ros_shutdown.assert_not_called()

    def test_pre_session_cleanup_reporter_failure_does_not_escape_main(self):
        class FailingLogger:
            def warning(self, message):
                raise ValueError("primary pre-session failure")

            def error(self, message):
                raise RuntimeError("logger failed")

        class FakeNode:
            def get_logger(self):
                return FailingLogger()

        class FakeRosEndpoint(FakeEndpoint):
            def __init__(self):
                super().__init__(power_errors={False: RuntimeError("disarm failed")})
                self.node = FakeNode()

            def destroy(self):
                pass

            def report_cleanup_error(self, message):
                self.node.get_logger().error(message)

        endpoint = FakeRosEndpoint()
        with (
            mock_tk_window(),
            patch.object(self.module.rclpy, "init"),
            patch.object(self.module.rclpy, "try_shutdown"),
            patch.object(self.module, "RosMotorEndpoint", return_value=endpoint),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            try:
                result = self.module.main(["--yes-i-confirm-motion"])
            except RuntimeError as error:
                self.fail(f"cleanup reporter masked the primary failure: {error}")

        self.assertEqual(result, 1)
        self.assertIn(("power", False), endpoint.events)

    def test_partial_endpoint_initialization_is_owned_and_cleaned_up(self):
        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class FakeFuture:
            def done(self):
                return True

            def result(self):
                return type("Response", (), {"success": True, "message": ""})()

        class FakeClient:
            def __init__(self):
                self.requests = []

            def wait_for_service(self, timeout_sec):
                return True

            def call_async(self, request):
                self.requests.append(request.data)
                return FakeFuture()

        class FakeNode:
            def __init__(self):
                self.publisher = FakePublisher()
                self.client = FakeClient()
                self.destroyed = False

            def create_publisher(self, message_type, topic, depth):
                return self.publisher

            def create_client(self, service_type, name):
                return self.client

            def create_subscription(self, message_type, topic, callback, qos):
                raise ValueError("subscription construction failed")

            def destroy_node(self):
                self.destroyed = True

        node = FakeNode()
        with (
            mock_tk_window(),
            patch.object(self.module.rclpy, "init"),
            patch.object(self.module.rclpy, "try_shutdown"),
            patch.object(self.module.rclpy, "create_node", return_value=node),
            patch.object(self.module.rclpy, "spin_until_future_complete"),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            result = self.module.main(["--yes-i-confirm-motion"])

        self.assertEqual(result, 1)
        self.assertEqual(len(node.publisher.messages), 20)
        self.assertEqual(node.client.requests, [False])
        self.assertTrue(node.destroyed)

    def test_package_and_readme_register_only_the_guarded_physical_teleop(self):
        repo = Path(__file__).resolve().parents[2]
        setup_text = (repo / "lekiwi_teleop/setup.py").read_text()
        package_text = (repo / "lekiwi_teleop/package.xml").read_text()
        self.assertIn("docs/hardware/base_operation.md", (repo / "README.md").read_text())
        readme = (repo / "docs/hardware/base_operation.md").read_text()

        self.assertIn(
            "lekiwi_guarded_base_teleop = lekiwi_teleop.guarded_base_teleop:main",
            setup_text,
        )
        self.assertIn("<exec_depend>std_msgs</exec_depend>", package_text)
        self.assertIn("<exec_depend>python3-tk</exec_depend>", package_text)
        self.assertIn(
            "ros2 run lekiwi_teleop lekiwi_guarded_base_teleop "
            "--yes-i-confirm-motion",
            readme,
        )
        self.assertIn("lekiwi_base_teleop", readme)
        self.assertIn("demonstration only", readme)
        self.assertIn("--max-session-duration 3600", readme)
        self.assertNotIn("--nav2-max-speed", readme)
        self.assertNotIn("--gui-deadman", readme)
        self.assertIn("separate guarded bounded wrapper", readme)
        self.assertNotIn(
            "combined forward/yaw commands use one shared wheel-envelope scale",
            readme,
        )
        self.assertIn("방향 키는 누르는", readme)
        self.assertIn("방향 전환 중 zero 명령을 먼저 보낸다", readme)
        self.assertIn("창의 포커스를 잃거나 Space를 누르면", readme)
        for token in (
            "`w` / `s`",
            "`a` / `d`",
            "`q` / `e`",
            "`1` / `2` / `3`",
            "Space",
            "Esc 또는 창 닫기",
        ):
            self.assertIn(token, readme)


if __name__ == "__main__":
    unittest.main()
