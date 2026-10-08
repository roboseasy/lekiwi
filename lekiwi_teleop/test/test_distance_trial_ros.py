"""ROS message validation without starting a ROS context or any hardware."""

import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nav_msgs.msg import Odometry
from lekiwi_teleop.distance_trial_ros import DistanceEndpoint


class TestDistanceOdometry(unittest.TestCase):
    def setUp(self):
        self.endpoint = DistanceEndpoint()
        self.endpoint.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=200_000_000_000)))
        self.endpoint.expected_frames = ("odom", "base_footprint")

    def message(self, seconds=200., x=0., yaw=0.):
        msg = Odometry()
        ns = round(seconds * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.header.frame_id, msg.child_frame_id = "odom", "base_footprint"
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        return msg

    def test_old_or_nonadvancing_stamps_are_rejected(self):
        self.endpoint.on_odom(self.message(199.))
        self.assertIn("Stale", self.endpoint.error)
        self.setUp()
        self.endpoint.on_odom(self.message())
        self.endpoint.on_odom(self.message())
        self.assertIn("advance", self.endpoint.error)

    def test_position_and_yaw_resets_are_rejected(self):
        for options in ({"x": .1}, {"yaw": .3}):
            self.setUp()
            self.endpoint.on_odom(self.message())
            self.endpoint.on_odom(self.message(200.02, **options))
            self.assertIn("jumped", self.endpoint.error)

    def test_quaternion_and_frames_are_checked(self):
        msg = self.message()
        msg.pose.pose.orientation.w = 0.
        self.endpoint.on_odom(msg)
        self.assertIn("quaternion", self.endpoint.error)
        self.setUp()
        msg = self.message()
        msg.child_frame_id = "different_base"
        self.endpoint.on_odom(msg)
        self.assertIn("frames", self.endpoint.error)

    def test_receive_gap_cannot_be_hidden_by_a_fresh_latest_sample(self):
        with patch("lekiwi_teleop.distance_trial_ros.time.monotonic", side_effect=[10., 10.4]):
            self.endpoint.on_odom(self.message())
            self.endpoint.on_odom(self.message(200.02))
        self.assertIn("gap", self.endpoint.error)

    def test_small_motion_is_accepted_and_recorded(self):
        self.endpoint.on_odom(self.message())
        self.endpoint.on_odom(self.message(200.02, x=.0004, yaw=.001))
        self.assertIsNone(self.endpoint.error)
        self.assertEqual(len(self.endpoint.samples), 2)
        self.assertAlmostEqual(self.endpoint.latest["yaw"], .001)


if __name__ == "__main__":
    unittest.main()
