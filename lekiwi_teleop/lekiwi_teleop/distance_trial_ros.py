"""ROS adapter for distance_trial; imported only by its explicit run command."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions

from lekiwi_teleop.distance_trial import displacement, require
from lekiwi_teleop.motor_session import Motion, RosMotorEndpoint, cleanup_session


class DistanceEndpoint(RosMotorEndpoint):
    def __init__(self):
        super().__init__()
        self.latest = None
        self.error = None
        self.samples = []
        self.expected_frames = None

    @property
    def ready(self):
        return self.motor_ready

    def initialize(self):
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        super().initialize()
        self.node.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_odom(self, message):
        try:
            p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            require(all(math.isfinite(x) for x in
                        (p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z)),
                    "Non-finite odometry")
            require(abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < .01,
                    "Invalid odometry quaternion")
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            require(stamp > 0, "Zero odometry timestamp")
            require(abs(self.node.get_clock().now().nanoseconds - stamp) <= 300_000_000,
                    "Stale odometry header timestamp")
            frames = (message.header.frame_id, message.child_frame_id)
            require(all(frames), "Empty odometry frame")
            if self.expected_frames is not None:
                require(frames == self.expected_frames, "Unexpected odometry frames")
            sample = {"received_s": time.monotonic(), "stamp_ns": stamp,
                      "frame": frames[0], "child": frames[1], "x": p.x, "y": p.y,
                      "yaw": math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)),
                      "vx": v.linear.x, "vy": v.linear.y, "wz": v.angular.z}
            if self.latest is not None:
                require(stamp > self.latest["stamp_ns"], "Odometry timestamp did not advance")
                require(frames == (self.latest["frame"], self.latest["child"]), "Odometry frame changed")
                require(math.hypot(p.x-self.latest["x"], p.y-self.latest["y"]) <= .05,
                        "Odometry jumped or was reset")
                require(abs(displacement(self.latest, sample, "forward")["yaw_rad"]) <= .2,
                        "Odometry yaw jumped or was reset")
                require(sample["received_s"] - self.latest["received_s"] <= .3,
                        "Odometry sample gap exceeded 0.3 seconds")
            self.latest = sample
            self.samples.append(sample)
        except Exception as error:
            self.error = str(error)

    def parameters(self):
        client = AsyncParameterClient(self.node, "/lekiwi_node")
        require(client.wait_for_services(timeout_sec=5.), "Base parameter service unavailable")
        names = ["motor_backend", "enable_motor_write", "torque_enable", "odom_source",
                 "encoder_feedback_source", "use_sim_time", "cmd_vel_timeout", "max_linear_x",
                 "max_linear_y", "max_linear_speed", "max_wheel_speed", "wheel_radius",
                 "base_radius", "speed_tick_scale", "speed_tick_limit", "wheel_ids",
                 "wheel_directions", "encoder_odom_scale", "encoder_position_ticks_per_revolution",
                 "kinematics_frame_x", "kinematics_frame_y", "kinematics_frame_yaw",
                 "odom_frame", "base_frame"]
        future = client.get_parameters(names)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.)
        require(future.done() and future.result() is not None, "Parameter query timed out")
        result = dict(zip(names, map(parameter_value_to_python, future.result().values)))
        require(all(value is not None for value in result.values()), "Missing base parameters")
        self.expected_frames = (result["odom_frame"], result["base_frame"])
        return result

    def check_graph(self):
        def names(endpoints):
            return [e.node_namespace.rstrip("/") + "/" + e.node_name for e in endpoints]
        require(names(self.node.get_publishers_info_by_topic("/cmd_vel")) ==
                [self.node.get_fully_qualified_name()], "Another cmd_vel publisher exists")
        require(names(self.node.get_subscriptions_info_by_topic("/cmd_vel")) == ["/lekiwi_node"],
                "Expected one base cmd_vel subscriber")
        for topic in ("/odom", "/motor_ready"):
            require(names(self.node.get_publishers_info_by_topic(topic)) == ["/lekiwi_node"],
                    f"Unexpected publisher on {topic}")

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            require(rclpy.ok(), "ROS context stopped")
            rclpy.spin_once(self.node, timeout_sec=max(0., deadline - time.monotonic()))

    def publish(self, x, y):
        self.publish_motion(Motion(linear_x=x, linear_y=y))

    def arm(self):
        self.begin_ready_observation()
        self.set_motor_power(True)
        require(self.wait_for_motor_ready(5.), "No fresh motor_ready after enabling power")

    def stop_and_disarm(self):
        self.begin_ready_observation()
        error = cleanup_session(self, 20)
        if error is not None:
            raise error
        self.spin(.2)
        require(self.ready is False, "No motor_ready=false after torque-off acknowledgement")

    def destroy(self):
        try:
            super().destroy()
        finally:
            rclpy.try_shutdown()
