import unittest

from lekiwi_teleop.base_teleop import BaseTeleopMapper, LekiwiBaseTeleop, publish_stop_command


def twist_tuple(twist):
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


class TestBaseTeleopMapping(unittest.TestCase):
    def test_default_speed_limits_match_slow_mapping_profile(self):
        mapper = BaseTeleopMapper()

        self.assertAlmostEqual(mapper.max_linear_speed, 0.10)
        self.assertAlmostEqual(mapper.max_angular_speed, 0.30)

    def test_motion_keys_create_expected_twist_values(self):
        mapper = BaseTeleopMapper(linear_speed=0.05, angular_speed=0.3)

        self.assertEqual(
            twist_tuple(mapper.twist_for_key("w")),
            (0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            twist_tuple(mapper.twist_for_key("s")),
            (-0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            twist_tuple(mapper.twist_for_key("a")),
            (0.0, 0.05, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            twist_tuple(mapper.twist_for_key("d")),
            (0.0, -0.05, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(
            twist_tuple(mapper.twist_for_key("q")),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.3),
        )
        self.assertEqual(
            twist_tuple(mapper.twist_for_key("e")),
            (0.0, 0.0, 0.0, 0.0, 0.0, -0.3),
        )

    def test_stop_key_creates_zero_twist(self):
        mapper = BaseTeleopMapper()

        self.assertEqual(
            twist_tuple(mapper.twist_for_key(" ")),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_unknown_key_returns_none(self):
        mapper = BaseTeleopMapper()

        self.assertIsNone(mapper.twist_for_key("?"))

    def test_speed_adjustment_keys_return_updated_mapper_without_twist(self):
        mapper = BaseTeleopMapper(
            linear_speed=0.05,
            angular_speed=0.5,
            speed_scale_step=1.2,
            max_angular_speed=1.0,
        )

        faster = mapper.adjust_speed_for_key("+")

        self.assertIsNotNone(faster)
        self.assertAlmostEqual(faster.linear_speed, 0.06)
        self.assertAlmostEqual(faster.angular_speed, 0.6)
        self.assertIsNone(mapper.twist_for_key("+"))

        faster_from_equal = mapper.adjust_speed_for_key("=")

        self.assertIsNotNone(faster_from_equal)
        self.assertAlmostEqual(faster_from_equal.linear_speed, 0.06)
        self.assertAlmostEqual(faster_from_equal.angular_speed, 0.6)
        self.assertIsNone(mapper.twist_for_key("="))

        slower = faster.adjust_speed_for_key("-")

        self.assertIsNotNone(slower)
        self.assertAlmostEqual(slower.linear_speed, 0.05)
        self.assertAlmostEqual(slower.angular_speed, 0.5)
        self.assertIsNone(faster.twist_for_key("-"))

    def test_speed_adjustment_clamps_to_configured_limits(self):
        mapper = BaseTeleopMapper(
            linear_speed=0.14,
            angular_speed=0.95,
            speed_scale_step=1.2,
            min_linear_speed=0.02,
            max_linear_speed=0.15,
            min_angular_speed=0.2,
            max_angular_speed=1.0,
        )

        faster = mapper.adjust_speed_for_key("+")

        self.assertAlmostEqual(faster.linear_speed, 0.15)
        self.assertAlmostEqual(faster.angular_speed, 1.0)

        slow_mapper = BaseTeleopMapper(
            linear_speed=0.021,
            angular_speed=0.21,
            speed_scale_step=1.2,
            min_linear_speed=0.02,
            max_linear_speed=0.15,
            min_angular_speed=0.2,
            max_angular_speed=1.0,
        )

        slower = slow_mapper.adjust_speed_for_key("_")

        self.assertAlmostEqual(slower.linear_speed, 0.02)
        self.assertAlmostEqual(slower.angular_speed, 0.2)

    def test_teleop_node_speed_key_updates_mapper_without_publishing_twist(self):
        class FakeLogger:
            def __init__(self):
                self.messages = []

            def info(self, message):
                self.messages.append(message)

        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = LekiwiBaseTeleop.__new__(LekiwiBaseTeleop)
        node.mapper = BaseTeleopMapper(
            linear_speed=0.05,
            angular_speed=0.5,
            max_angular_speed=1.0,
        )
        node.publisher = FakePublisher()
        logger = FakeLogger()
        node.get_logger = lambda: logger

        self.assertTrue(node.publish_for_key("+"))

        self.assertEqual(node.publisher.messages, [])
        self.assertAlmostEqual(node.mapper.linear_speed, 0.06)
        self.assertAlmostEqual(node.mapper.angular_speed, 0.6)
        self.assertIn("teleop speed updated", logger.messages[-1])

    def test_exit_key_is_detected_without_twist(self):
        mapper = BaseTeleopMapper()

        self.assertTrue(mapper.is_exit_key("x"))
        self.assertIsNone(mapper.twist_for_key("x"))

    def test_publish_stop_command_repeats_zero_twist_and_waits(self):
        class FakePublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        sleep_calls = []
        publisher = FakePublisher()

        publish_stop_command(
            publisher,
            publish_count=3,
            sleep_sec=0.02,
            sleeper=sleep_calls.append,
        )

        self.assertEqual(
            [twist_tuple(message) for message in publisher.messages],
            [
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ],
        )
        self.assertEqual(sleep_calls, [0.02, 0.02, 0.02])
