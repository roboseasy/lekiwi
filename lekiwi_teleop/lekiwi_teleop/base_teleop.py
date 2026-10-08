import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class BaseTeleopMapper:
    linear_speed: float = 0.05
    angular_speed: float = 0.3
    speed_scale_step: float = 1.2
    min_linear_speed: float = 0.02
    max_linear_speed: float = 0.10
    min_angular_speed: float = 0.2
    max_angular_speed: float = 0.3

    def __post_init__(self):
        if self.speed_scale_step <= 1.0:
            raise ValueError("speed_scale_step must be greater than 1.0")
        if self.min_linear_speed <= 0.0 or self.max_linear_speed < self.min_linear_speed:
            raise ValueError("linear speed limits must be positive and ordered")
        if self.min_angular_speed <= 0.0 or self.max_angular_speed < self.min_angular_speed:
            raise ValueError("angular speed limits must be positive and ordered")

        object.__setattr__(
            self,
            "linear_speed",
            self._clamp(self.linear_speed, self.min_linear_speed, self.max_linear_speed),
        )
        object.__setattr__(
            self,
            "angular_speed",
            self._clamp(self.angular_speed, self.min_angular_speed, self.max_angular_speed),
        )

    def twist_for_key(self, key: str) -> Twist | None:
        if key == "w":
            return self._twist(linear_x=self.linear_speed)
        if key == "s":
            return self._twist(linear_x=-self.linear_speed)
        if key == "a":
            return self._twist(linear_y=self.linear_speed)
        if key == "d":
            return self._twist(linear_y=-self.linear_speed)
        if key == "q":
            return self._twist(angular_z=self.angular_speed)
        if key == "e":
            return self._twist(angular_z=-self.angular_speed)
        if key == " ":
            return self._twist()
        return None

    def adjust_speed_for_key(self, key: str) -> "BaseTeleopMapper | None":
        if key in {"+", "="}:
            factor = self.speed_scale_step
        elif key in {"-", "_"}:
            factor = 1.0 / self.speed_scale_step
        else:
            return None

        return BaseTeleopMapper(
            linear_speed=self.linear_speed * factor,
            angular_speed=self.angular_speed * factor,
            speed_scale_step=self.speed_scale_step,
            min_linear_speed=self.min_linear_speed,
            max_linear_speed=self.max_linear_speed,
            min_angular_speed=self.min_angular_speed,
            max_angular_speed=self.max_angular_speed,
        )

    def is_exit_key(self, key: str) -> bool:
        return key in {"x", "\x03", "\x1b"}

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return min(max(value, minimum), maximum)

    @staticmethod
    def _twist(linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0) -> Twist:
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        return twist


def publish_stop_command(
    publisher,
    publish_count: int = 3,
    sleep_sec: float = 0.02,
    sleeper=time.sleep,
):
    for _ in range(publish_count):
        publisher.publish(BaseTeleopMapper._twist())
        sleeper(sleep_sec)


class LekiwiBaseTeleop(Node):
    def __init__(self):
        super().__init__("lekiwi_base_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("angular_speed", 0.3)
        self.declare_parameter("speed_scale_step", 1.2)
        self.declare_parameter("min_linear_speed", 0.02)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("min_angular_speed", 0.2)
        self.declare_parameter("max_angular_speed", 0.3)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        linear_speed = float(self.get_parameter("linear_speed").value)
        angular_speed = float(self.get_parameter("angular_speed").value)
        speed_scale_step = float(self.get_parameter("speed_scale_step").value)
        min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        min_angular_speed = float(self.get_parameter("min_angular_speed").value)
        max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.mapper = BaseTeleopMapper(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
            speed_scale_step=speed_scale_step,
            min_linear_speed=min_linear_speed,
            max_linear_speed=max_linear_speed,
            min_angular_speed=min_angular_speed,
            max_angular_speed=max_angular_speed,
        )
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(
            "LeKiwi base teleop 시작: w/s 전후, a/d 좌우, q/e 회전, +/- 속도 조절, space 정지, x 종료"
        )
        self.get_logger().info(
            f"publish topic={cmd_vel_topic}, linear={self.mapper.linear_speed:.3f} m/s, "
            f"angular={self.mapper.angular_speed:.3f} rad/s"
        )

    def publish_stop(self):
        publish_stop_command(self.publisher)

    def publish_for_key(self, key: str) -> bool:
        if self.mapper.is_exit_key(key):
            self.publish_stop()
            return False

        adjusted_mapper = self.mapper.adjust_speed_for_key(key)
        if adjusted_mapper is not None:
            self.mapper = adjusted_mapper
            self.get_logger().info(
                f"teleop speed updated: linear={self.mapper.linear_speed:.3f} m/s, "
                f"angular={self.mapper.angular_speed:.3f} rad/s"
            )
            return True

        twist = self.mapper.twist_for_key(key)
        if twist is None:
            return True

        self.publisher.publish(twist)
        return True


class RawTerminal:
    def __enter__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def read_key(self) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not readable:
            return None
        return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LekiwiBaseTeleop()
        with RawTerminal() as terminal:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                key = terminal.read_key()
                if key is None:
                    continue
                if not node.publish_for_key(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
