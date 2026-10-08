import contextlib
import io
import unittest

from lekiwi_teleop.safe_cmd_vel import (
    MotionLimits,
    MotionCommand,
    build_publish_plan,
    build_parser,
    cleanup_after_motion,
    command_from_args,
    has_motion_command,
    publish_plan,
    publish_zero,
    validate_motion_request,
    wait_for_motor_ready,
    wait_for_subscribers,
)


def twist_tuple(twist):
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TestSafeCmdVelTest(unittest.TestCase):
    def test_wait_for_motor_ready_requires_observed_transition(self):
        states = [False]
        clock = [0.0]

        def spin(_node, timeout_sec):
            clock[0] += timeout_sec
            if clock[0] >= 0.1:
                states.append(True)

        wait_for_motor_ready(
            object(), states, True, 0.2,
            spin=spin, monotonic=lambda: clock[0],
        )
        self.assertGreaterEqual(clock[0], 0.1)

        states[:] = [False]
        clock[0] = 0.0
        with self.assertRaisesRegex(RuntimeError, "motor_ready=true"):
            wait_for_motor_ready(
                object(), states, True, 0.1,
                spin=lambda _node, timeout_sec: clock.__setitem__(0, clock[0] + timeout_sec),
                monotonic=lambda: clock[0],
            )

    def test_wait_for_subscribers_requires_matching_endpoints_before_motion(self):
        class DiscoveryPublisher:
            def __init__(self):
                self.counts = iter((0, 1, 2))

            def get_subscription_count(self):
                return next(self.counts, 2)

        clock = [0.0]

        def sleep(seconds):
            clock[0] += seconds

        wait_for_subscribers(
            DiscoveryPublisher(), 2, 1.0,
            sleep=sleep, monotonic=lambda: clock[0],
        )
        self.assertGreater(clock[0], 0.0)

        class NoSubscribers:
            def get_subscription_count(self):
                return 0

        with self.assertRaisesRegex(RuntimeError, "구독자 1개"):
            wait_for_subscribers(
                NoSubscribers(), 1, 0.1,
                sleep=sleep, monotonic=lambda: clock[0],
            )

    def test_has_motion_command_detects_nonzero_axis(self):
        self.assertFalse(has_motion_command(MotionCommand()))
        self.assertTrue(has_motion_command(MotionCommand(linear_x=0.01)))
        self.assertTrue(has_motion_command(MotionCommand(linear_y=-0.01)))
        self.assertTrue(has_motion_command(MotionCommand(angular_z=0.1)))

    def test_validate_motion_request_requires_explicit_confirmation(self):
        command = MotionCommand(linear_x=0.03)

        with self.assertRaisesRegex(ValueError, "yes-i-confirm-motion"):
            validate_motion_request(command, confirmed=False)

        validate_motion_request(command, confirmed=True)

        with self.assertRaisesRegex(ValueError, "yes-i-confirm-motion"):
            validate_motion_request(
                MotionCommand(),
                confirmed=False,
                arm_motors=True,
            )

    def test_validate_motion_request_rejects_values_above_default_limits(self):
        with self.assertRaisesRegex(ValueError, "linear"):
            validate_motion_request(MotionCommand(linear_x=0.2), confirmed=True)
        with self.assertRaisesRegex(ValueError, "angular"):
            validate_motion_request(MotionCommand(angular_z=1.0), confirmed=True)
        with self.assertRaisesRegex(ValueError, "duration"):
            validate_motion_request(MotionCommand(duration=10.0), confirmed=True)
        with self.assertRaisesRegex(ValueError, "rate"):
            validate_motion_request(MotionCommand(rate=100.0), confirmed=True)

    def test_validate_motion_request_allows_confirmed_motion_with_explicit_limits(self):
        validate_motion_request(
            MotionCommand(
                linear_x=0.2,
                linear_y=-0.2,
                angular_z=1.0,
                duration=10.0,
                rate=100.0,
            ),
            confirmed=True,
            limits=MotionLimits(
                max_linear=0.3,
                max_angular=1.2,
                max_duration=12.0,
                max_rate=120.0,
            ),
        )

    def test_validate_motion_request_rejects_non_positive_timing(self):
        with self.assertRaisesRegex(ValueError, "duration"):
            validate_motion_request(MotionCommand(linear_x=0.01, duration=0.0), confirmed=True)
        with self.assertRaisesRegex(ValueError, "rate"):
            validate_motion_request(MotionCommand(linear_x=0.01, rate=0.0), confirmed=True)

    def test_build_publish_plan_is_finite_and_rate_based(self):
        command = MotionCommand(linear_x=0.03, duration=2.0, rate=20.0)

        plan = build_publish_plan(command)

        self.assertEqual(len(plan), 40)
        self.assertEqual(
            [twist_tuple(message) for message in plan[:2]],
            [
                (0.03, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.03, 0.0, 0.0, 0.0, 0.0, 0.0),
            ],
        )

    def test_publish_zero_repeats_zero_twist(self):
        publisher = FakePublisher()

        publish_zero(publisher, cycles=3)

        self.assertEqual(
            [twist_tuple(message) for message in publisher.messages],
            [
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ],
        )

    def test_cleanup_after_motion_attempts_disarm_even_when_zero_publish_fails(self):
        class AlwaysFailingPublisher:
            def publish(self, message):
                raise RuntimeError("zero publish failed")

        disarm_calls = []

        with self.assertRaisesRegex(RuntimeError, "zero publish failed"):
            cleanup_after_motion(
                AlwaysFailingPublisher(),
                zero_cycles=3,
                disarm=disarm_calls.append,
            )

        self.assertEqual(disarm_calls, [False])

    def test_publish_plan_sends_zero_even_when_publish_fails(self):
        class FailingPublisher(FakePublisher):
            def publish(self, message):
                super().publish(message)
                if len(self.messages) == 2:
                    raise RuntimeError("publish failed")

        publisher = FailingPublisher()
        plan = build_publish_plan(MotionCommand(linear_x=0.03, duration=0.1, rate=20.0))

        with self.assertRaisesRegex(RuntimeError, "publish failed"):
            publish_plan(publisher, plan, zero_cycles=3, sleep=lambda _: None)

        self.assertEqual(
            [twist_tuple(message) for message in publisher.messages[-3:]],
            [
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ],
        )

    def test_publish_plan_does_not_sleep_after_last_motion_message(self):
        publisher = FakePublisher()
        sleep_calls = []
        plan = build_publish_plan(MotionCommand(linear_x=0.03, duration=1.0, rate=3.0))

        publish_plan(
            publisher,
            plan,
            zero_cycles=1,
            sleep_period=1.0 / 3.0,
            sleep=sleep_calls.append,
        )

        self.assertEqual(len(plan), 3)
        self.assertEqual(sleep_calls, [1.0 / 3.0, 1.0 / 3.0])

    def test_parser_values_feed_validation_contract(self):
        parser = build_parser()
        args = parser.parse_args(
            [
                "--linear-x",
                "0.03",
                "--duration",
                "1.5",
            ]
        )
        command = command_from_args(args)

        self.assertEqual(command, MotionCommand(linear_x=0.03, duration=1.5))
        self.assertEqual(args.max_linear, 0.05)
        self.assertEqual(args.max_angular, 0.3)
        self.assertEqual(args.max_duration, 2.0)
        self.assertEqual(args.max_rate, 50.0)
        self.assertEqual(args.min_subscribers, 1)
        self.assertEqual(args.discovery_timeout, 8.0)
        with self.assertRaisesRegex(ValueError, "yes-i-confirm-motion"):
            validate_motion_request(command, confirmed=args.yes_i_confirm_motion)

    def test_parser_rejects_zero_zero_cycles_and_rejects_high_rate_by_default(self):
        parser = build_parser()

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(["--zero-cycles", "0"])

        args = parser.parse_args(["--rate", "1000000000"])
        command = command_from_args(args)

        with self.assertRaisesRegex(ValueError, "rate"):
            validate_motion_request(command, confirmed=True)

        args = parser.parse_args(["--rate", "100.0", "--max-rate", "120.0"])
        command = command_from_args(args)
        validate_motion_request(
            command,
            confirmed=True,
            limits=MotionLimits(max_rate=args.max_rate),
        )


if __name__ == "__main__":
    unittest.main()
