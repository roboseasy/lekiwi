"""Keep wheel power enabled only during a supervised Nav2 session."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

from lekiwi_teleop.motor_session import RosMotorEndpoint


def positive_duration(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError("a finite, positive duration is required")
    return duration


def names(endpoints) -> set[str]:
    return {
        f"{info.node_namespace.rstrip('/')}/{info.node_name}"
        for info in endpoints
    }


def check_command_graph(node) -> None:
    publisher_infos = node.get_publishers_info_by_topic("/cmd_vel")
    subscriber_infos = node.get_subscriptions_info_by_topic("/cmd_vel")
    publishers = names(publisher_infos)
    subscribers = names(subscriber_infos)
    if len(publisher_infos) != 1 or publishers != {"/collision_monitor"}:
        raise RuntimeError(f"expected only /collision_monitor on /cmd_vel; found {sorted(publishers)}")
    if len(subscriber_infos) != 1 or subscribers != {"/lekiwi_node"}:
        raise RuntimeError(f"expected only /lekiwi_node on /cmd_vel; found {sorted(subscribers)}")


def check_hardware_parameters(values) -> None:
    backend, writes, odom, initial_torque = values
    if backend.string_value != "feetech" or not writes.bool_value:
        raise RuntimeError("base is not using a writable Feetech motor backend")
    if odom.string_value != "encoder":
        raise RuntimeError("Nav2 requires encoder odometry on the physical base")
    if initial_torque.bool_value:
        raise RuntimeError("base started with torque enabled; start it with torque_enable:=false")


def read_hardware_parameters(node) -> None:
    client = AsyncParameterClient(node, "/lekiwi_node")
    if not client.wait_for_services(timeout_sec=3.0):
        raise RuntimeError("/lekiwi_node parameter services are unavailable")
    future = client.get_parameters([
        "motor_backend", "enable_motor_write", "odom_source", "torque_enable",
    ])
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    if not future.done() or future.result() is None:
        raise RuntimeError("could not read base hardware parameters")
    check_hardware_parameters(future.result().values)


class SensorObserver:
    def __init__(self, node, clock=time.monotonic):
        self.clock = clock
        self.scan_time = None
        self.odom_time = None
        self.scan_subscription = node.create_subscription(
            LaserScan, "/scan_navigation", self._on_scan, qos_profile_sensor_data
        )
        self.odom_subscription = node.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )

    def _on_scan(self, _message):
        self.scan_time = self.clock()

    def _on_odom(self, _message):
        self.odom_time = self.clock()

    def fresh(self, max_age=1.0):
        now = self.clock()
        return (
            self.scan_time is not None
            and self.odom_time is not None
            and now - self.scan_time <= max_age
            and now - self.odom_time <= max_age
        )


def publish_zero(node) -> None:
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    try:
        for _ in range(20):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_publisher(publisher)


def supervise(endpoint, duration, startup_timeout=5.0, clock=time.monotonic) -> None:
    node = endpoint.node
    observer = SensorObserver(node, clock)
    deadline = clock() + startup_timeout
    graph_error = None
    while clock() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if not observer.fresh() or endpoint.motor_ready is None:
            continue
        if endpoint.motor_ready is True:
            raise RuntimeError("/motor_ready must be false before this guard arms the motors")
        try:
            check_command_graph(node)
            break
        except RuntimeError as error:
            graph_error = error
    else:
        if graph_error is not None and observer.fresh():
            raise graph_error
    if not observer.fresh():
        raise RuntimeError("fresh /scan_navigation and /odom were not observed")
    if endpoint.motor_ready is not False:
        raise RuntimeError("/motor_ready must be false before this guard arms the motors")
    check_command_graph(node)
    read_hardware_parameters(node)
    publish_zero(node)
    graph_deadline = clock() + 2.0
    while True:
        try:
            check_command_graph(node)
            break
        except RuntimeError:
            if clock() >= graph_deadline:
                raise
            rclpy.spin_once(node, timeout_sec=0.05)
    if not observer.fresh():
        raise RuntimeError("/scan_navigation or /odom became stale before motor power on")

    endpoint.begin_ready_observation()
    endpoint.set_motor_power(True)
    if not endpoint.wait_for_motor_ready(5.0):
        raise RuntimeError("/motor_ready did not become true after motor power on")
    print(
        "Nav2 motor guard active. Completed goals keep motor power on; "
        "close RViz or stop the launch to disarm.",
        flush=True,
    )

    end = clock() + duration
    next_graph_check = clock()
    while rclpy.ok() and clock() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if endpoint.motor_ready is not True:
            raise RuntimeError("/motor_ready became false; Nav2 motor guard is disarming")
        if not observer.fresh():
            raise RuntimeError("/scan_navigation or /odom stopped; Nav2 motor guard is disarming")
        if clock() >= next_graph_check:
            check_command_graph(node)
            next_graph_check = clock() + 0.5
    if clock() >= end:
        print("Nav2 motor guard time limit reached; disarming motors.", flush=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yes-i-confirm-motion", action="store_true", required=True)
    parser.add_argument("--max-session-duration", type=positive_duration, default=600.0)
    parser.add_argument("--startup-timeout", type=positive_duration, default=5.0)
    process_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = parser.parse_args(remove_ros_args(args=process_args)[1:])
    endpoint = RosMotorEndpoint(
        publish_commands=False, node_name="lekiwi_nav2_motor_guard"
    )
    rclpy.init(args=process_args, signal_handler_options=SignalHandlerOptions.NO)
    result = 0
    try:
        endpoint.initialize()
        supervise(endpoint, args.max_session_duration, args.startup_timeout)
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        print(f"Nav2 motor guard stopped: {error}", file=sys.stderr)
        result = 1
    finally:
        if endpoint.node is not None:
            try:
                endpoint.set_motor_power(False)
            except Exception as error:
                print(f"Motor power off was not acknowledged: {error}", file=sys.stderr)
                result = 1
            try:
                publish_zero(endpoint.node)
            except Exception as error:
                print(f"Zero velocity publish failed: {error}", file=sys.stderr)
                result = 1
            endpoint.destroy()
        rclpy.try_shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
