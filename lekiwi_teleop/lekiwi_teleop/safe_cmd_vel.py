import argparse
import math
import time
from dataclasses import dataclass
from typing import Callable, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from lekiwi_teleop.motor_session import request_motor_power as set_motor_power


@dataclass(frozen=True)
class MotionCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    duration: float = 1.0
    rate: float = 20.0


@dataclass(frozen=True)
class MotionLimits:
    max_linear: float = 0.05
    max_angular: float = 0.3
    max_duration: float = 2.0
    max_rate: float = 50.0


def finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"finite float 값이 필요합니다: {value}")
    return parsed


def positive_float(value: str) -> float:
    parsed = finite_float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"0보다 큰 값이 필요합니다: {value}")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError(f"0보다 큰 정수가 필요합니다: {value}")
    return parsed


def make_twist(linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0) -> Twist:
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.angular.z = angular_z
    return twist


def has_motion_command(command: MotionCommand) -> bool:
    return any(
        abs(value) > 0.0
        for value in (command.linear_x, command.linear_y, command.angular_z)
    )


def validate_motion_request(
    command: MotionCommand,
    *,
    confirmed: bool,
    arm_motors: bool = False,
    limits: MotionLimits = MotionLimits(),
) -> None:
    if command.duration <= 0.0:
        raise ValueError("duration은 0보다 커야 합니다.")
    if command.rate <= 0.0:
        raise ValueError("rate는 0보다 커야 합니다.")
    linear_speed = math.hypot(command.linear_x, command.linear_y)
    if linear_speed > limits.max_linear:
        raise ValueError(
            "linear 속도가 안전 상한을 초과했습니다: "
            f"{linear_speed:g} > {limits.max_linear:g}. "
            "--max-linear로 현장 상한을 명시적으로 조정하세요."
        )
    if abs(command.angular_z) > limits.max_angular:
        raise ValueError(
            "angular 속도가 안전 상한을 초과했습니다: "
            f"{abs(command.angular_z):g} > {limits.max_angular:g}. "
            "--max-angular로 현장 상한을 명시적으로 조정하세요."
        )
    if command.duration > limits.max_duration:
        raise ValueError(
            "duration이 안전 상한을 초과했습니다: "
            f"{command.duration:g} > {limits.max_duration:g}. "
            "--max-duration으로 현장 상한을 명시적으로 조정하세요."
        )
    if command.rate > limits.max_rate:
        raise ValueError(
            "rate가 안전 상한을 초과했습니다: "
            f"{command.rate:g} > {limits.max_rate:g}. "
            "--max-rate로 현장 상한을 명시적으로 조정하세요."
        )
    if (has_motion_command(command) or arm_motors) and not confirmed:
        raise ValueError(
            "실제 모터 출력 또는 non-zero /cmd_vel은 --yes-i-confirm-motion이 필요합니다."
        )


def build_publish_plan(command: MotionCommand) -> list[Twist]:
    publish_count = max(1, math.ceil(command.duration * command.rate))
    return [
        make_twist(command.linear_x, command.linear_y, command.angular_z)
        for _ in range(publish_count)
    ]


def publish_zero(publisher, cycles: int = 20) -> None:
    zero = make_twist()
    for _ in range(cycles):
        publisher.publish(zero)


def wait_for_subscribers(
    publisher,
    minimum: int,
    timeout: float,
    *,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    deadline = monotonic() + timeout
    while publisher.get_subscription_count() < minimum:
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            raise RuntimeError(
                f"/cmd_vel 구독자 {minimum}개를 {timeout:g}초 안에 확인하지 못했습니다."
            )
        sleep(min(0.05, remaining))


def wait_for_motor_ready(
    node: Node,
    states: list[bool],
    expected: bool,
    timeout: float,
    *,
    spin: Callable = rclpy.spin_once,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        spin(node, timeout_sec=min(0.05, max(0.0, deadline - monotonic())))
        if states and states[-1] is expected:
            return
    raise RuntimeError(f"/motor_ready={str(expected).lower()}를 {timeout:g}초 안에 확인하지 못했습니다.")


def publish_plan(
    publisher,
    plan: Sequence[Twist],
    *,
    zero_cycles: int,
    sleep_period: float | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    interval = 0.0 if sleep_period is None else sleep_period
    try:
        publish_zero(publisher, zero_cycles)
        for index, twist in enumerate(plan):
            publisher.publish(twist)
            if interval > 0.0 and index < len(plan) - 1:
                sleep(interval)
    finally:
        publish_zero(publisher, zero_cycles)


def cleanup_after_motion(publisher, *, zero_cycles: int, disarm: Callable[[bool], None]) -> None:
    try:
        publish_zero(publisher, zero_cycles)
    finally:
        disarm(False)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "LeKiwi base 실물 검증용 /cmd_vel publisher. "
            "실제 모터 구동 전 현장 사용자 컨펌이 필요합니다."
        )
    )
    parser.add_argument("--linear-x", type=finite_float, default=0.0)
    parser.add_argument("--linear-y", type=finite_float, default=0.0)
    parser.add_argument("--angular-z", type=finite_float, default=0.0)
    parser.add_argument("--duration", type=positive_float, default=1.0)
    parser.add_argument("--rate", type=positive_float, default=20.0)
    parser.add_argument("--max-linear", type=positive_float, default=0.05)
    parser.add_argument("--max-angular", type=positive_float, default=0.3)
    parser.add_argument("--max-duration", type=positive_float, default=2.0)
    parser.add_argument("--max-rate", type=positive_float, default=50.0)
    parser.add_argument("--zero-cycles", type=positive_int, default=20)
    parser.add_argument("--min-subscribers", type=positive_int, default=1)
    parser.add_argument("--discovery-timeout", type=positive_float, default=8.0)
    parser.add_argument(
        "--arm-motors",
        action="store_true",
        help="/motor_power true를 먼저 호출하고 종료 시 false로 되돌립니다.",
    )
    parser.add_argument(
        "--yes-i-confirm-motion",
        action="store_true",
        help="현장 안전 확인과 사용자 컨펌이 끝난 motion임을 명시합니다.",
    )
    return parser


def command_from_args(args: argparse.Namespace) -> MotionCommand:
    return MotionCommand(
        linear_x=args.linear_x,
        linear_y=args.linear_y,
        angular_z=args.angular_z,
        duration=args.duration,
        rate=args.rate,
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    command = command_from_args(args)
    validate_motion_request(
        command,
        confirmed=args.yes_i_confirm_motion,
        arm_motors=args.arm_motors,
        limits=MotionLimits(
            max_linear=args.max_linear,
            max_angular=args.max_angular,
            max_duration=args.max_duration,
            max_rate=args.max_rate,
        ),
    )

    plan = build_publish_plan(command)
    sleep_period = 1.0 / command.rate

    rclpy.init()
    node = rclpy.create_node("lekiwi_safe_cmd_vel")
    publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    motor_power_client = node.create_client(SetBool, "/motor_power") if args.arm_motors else None
    ready_states: list[bool] = []
    if args.arm_motors:
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            Bool, "/motor_ready", lambda message: ready_states.append(message.data), ready_qos
        )

    try:
        wait_for_subscribers(
            publisher, args.min_subscribers, args.discovery_timeout
        )
        print(
            f"/cmd_vel 구독자 {publisher.get_subscription_count()}개 연결 확인",
            flush=True,
        )
        publish_zero(publisher, args.zero_cycles)
        if motor_power_client is not None:
            wait_for_motor_ready(node, ready_states, False, 2.0)
            set_motor_power(node, motor_power_client, True)
            wait_for_motor_ready(node, ready_states, True, 2.0)
        publish_plan(
            publisher,
            plan,
            zero_cycles=args.zero_cycles,
            sleep_period=sleep_period,
        )
    finally:
        try:
            cleanup_after_motion(
                publisher,
                zero_cycles=args.zero_cycles,
                disarm=(
                    (lambda enabled: set_motor_power(node, motor_power_client, enabled))
                    if motor_power_client is not None
                    else (lambda enabled: None)
                ),
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
