import argparse
import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Callable, Sequence

import rclpy
from rclpy.signals import SignalHandlerOptions

from lekiwi_teleop.motor_session import Motion, RosMotorEndpoint, cleanup_session


@dataclass(frozen=True)
class GuardedTeleopConfig:
    max_session_duration: float = 3600.0
    ready_timeout: float = 5.0
    key_poll_timeout: float = 0.1
    zero_cycles: int = 20

    def __post_init__(self):
        positive_values = {
            "max_session_duration": self.max_session_duration,
            "ready_timeout": self.ready_timeout,
            "key_poll_timeout": self.key_poll_timeout,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if self.zero_cycles <= 0:
            raise ValueError("zero_cycles must be greater than zero")


class DirectionalSpeedProfile:
    LINEAR_PROFILE_STEP = 0.005
    ANGULAR_PROFILE_STEP = 0.05
    MAX_SPEED_STAGE = 3
    DIRECTIONS = {
        "w": Motion(linear_x=0.10),
        "s": Motion(linear_x=-0.10),
        "a": Motion(linear_y=0.10),
        "d": Motion(linear_y=-0.10),
        "q": Motion(angular_z=0.30),
        "e": Motion(angular_z=-0.30),
    }

    def __init__(self):
        self.speed_stage = 1
        self.direction = Motion()
        self.target = Motion()
        self.control = Motion()
        self._pending_direction_change_zero = False

    def _update_target(self) -> None:
        scale = self.speed_stage / self.MAX_SPEED_STAGE
        self.target = Motion(
            linear_x=self.direction.linear_x * scale,
            linear_y=self.direction.linear_y * scale,
            angular_z=self.direction.angular_z * scale,
        )

    def handle_key(self, key: str) -> bool:
        if key in self.DIRECTIONS:
            next_direction = self.DIRECTIONS[key]
            direction_changed = (
                self.target != Motion() and next_direction != self.direction
            )
            self.direction = next_direction
            self._update_target()
            if direction_changed:
                self.control = Motion()
                self._pending_direction_change_zero = True
        elif key == "r":
            self.speed_stage = min(self.MAX_SPEED_STAGE, self.speed_stage + 1)
            self._update_target()
        elif key == "f":
            self.speed_stage = max(0, self.speed_stage - 1)
            self._update_target()
            if self.speed_stage == 0:
                self.control = Motion()
                self._pending_direction_change_zero = False
        elif key == " ":
            self.speed_stage = 0
            self._update_target()
            self.control = Motion()
            self._pending_direction_change_zero = False
        else:
            return False
        return True

    @staticmethod
    def _make_simple_profile(output: float, target: float, step: float) -> float:
        if target > output:
            return min(target, output + step)
        if target < output:
            return max(target, output - step)
        return target

    def next_motion(self) -> Motion:
        if self._pending_direction_change_zero:
            self._pending_direction_change_zero = False
            return self.control
        self.control = Motion(
            linear_x=self._make_simple_profile(
                self.control.linear_x,
                self.target.linear_x,
                self.LINEAR_PROFILE_STEP,
            ),
            linear_y=self._make_simple_profile(
                self.control.linear_y,
                self.target.linear_y,
                self.LINEAR_PROFILE_STEP,
            ),
            angular_z=self._make_simple_profile(
                self.control.angular_z,
                self.target.angular_z,
                self.ANGULAR_PROFILE_STEP,
            ),
        )
        return self.control


class HeldKeySpeedProfile:
    """One-axis motion only while a direction key remains pressed."""

    SPEEDS = {
        1: (0.04, 0.23),
        2: (0.05, 0.29),
        3: (0.06, 0.35),
    }
    DIRECTIONS = {
        "w": Motion(linear_x=1.0),
        "s": Motion(linear_x=-1.0),
        "a": Motion(linear_y=1.0),
        "d": Motion(linear_y=-1.0),
        "q": Motion(angular_z=1.0),
        "e": Motion(angular_z=-1.0),
    }

    def __init__(self):
        self.speed_stage = 1
        self.held_keys: list[str] = []
        self.control = Motion()
        self._pending_zero = False

    @property
    def active_key(self) -> str | None:
        return self.held_keys[-1] if self.held_keys else None

    def press(self, key: str) -> bool:
        if key in ("1", "2", "3"):
            self.speed_stage = int(key)
            return True
        if key == "space":
            self.stop()
            return True
        if key not in self.DIRECTIONS:
            return False
        if key in self.held_keys:
            return False
        previous = self.active_key
        self.held_keys.append(key)
        if previous is not None and previous != key:
            self.control = Motion()
            self._pending_zero = True
        return True

    def release(self, key: str) -> bool:
        if key not in self.held_keys:
            return False
        previous = self.active_key
        self.held_keys.remove(key)
        if self.active_key != previous:
            self.control = Motion()
            self._pending_zero = self.active_key is not None
            return True
        return False

    def stop(self) -> None:
        self.held_keys.clear()
        self.control = Motion()
        self._pending_zero = False

    def next_motion(self) -> Motion:
        key = self.active_key
        if key is None:
            self.control = Motion()
            return self.control
        if self._pending_zero:
            self._pending_zero = False
            return Motion()
        linear_speed, angular_speed = self.SPEEDS[self.speed_stage]
        direction = self.DIRECTIONS[key]
        target = Motion(
            direction.linear_x * linear_speed,
            direction.linear_y * linear_speed,
            direction.angular_z * angular_speed,
        )
        self.control = Motion(
            linear_x=DirectionalSpeedProfile._make_simple_profile(
                self.control.linear_x, target.linear_x, DirectionalSpeedProfile.LINEAR_PROFILE_STEP
            ),
            linear_y=DirectionalSpeedProfile._make_simple_profile(
                self.control.linear_y, target.linear_y, DirectionalSpeedProfile.LINEAR_PROFILE_STEP
            ),
            angular_z=DirectionalSpeedProfile._make_simple_profile(
                self.control.angular_z, target.angular_z, DirectionalSpeedProfile.ANGULAR_PROFILE_STEP
            ),
        )
        return self.control


def positive_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"finite value greater than zero required: {value}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Guarded three-stage hold-to-move desktop control for LeKiwi."
    )
    parser.add_argument(
        "--yes-i-confirm-motion",
        action="store_true",
        required=True,
        help="Acknowledge that fresh on-site motion authorization was obtained.",
    )
    parser.add_argument(
        "--max-session-duration",
        type=positive_finite_float,
        default=3600.0,
        help="Maximum total guarded session duration in seconds (default: 3600).",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> GuardedTeleopConfig:
    if not args.yes_i_confirm_motion:
        raise ValueError("--yes-i-confirm-motion is required")
    return GuardedTeleopConfig(max_session_duration=args.max_session_duration)


def _handle_cleanup_failure(
    cleanup_error: BaseException,
    primary_error: BaseException | None,
    report_cleanup_error: Callable[[str], None],
) -> None:
    message = f"guarded teleop cleanup failed: {cleanup_error}"
    if primary_error is not None:
        primary_error.add_note(message)
    try:
        report_cleanup_error(message)
    except BaseException as reporting_error:
        reporting_message = f"cleanup failure reporting also failed: {reporting_error}"
        if primary_error is not None:
            primary_error.add_note(reporting_message)
        else:
            cleanup_error.add_note(reporting_message)
        try:
            print(f"{message}; {reporting_message}", file=sys.stderr)
        except BaseException:
            pass
    if primary_error is None:
        raise cleanup_error


def run_guarded_session(
    endpoint,
    read_key: Callable[[float], str | None],
    *,
    config: GuardedTeleopConfig = GuardedTeleopConfig(),
    clock: Callable[[], float] = time.monotonic,
    report_cleanup_error: Callable[[str], None] = lambda message: print(
        message, file=sys.stderr
    ),
) -> str:
    deadline = clock() + config.max_session_duration
    profile = DirectionalSpeedProfile()
    primary_error: BaseException | None = None
    try:
        for _ in range(config.zero_cycles):
            endpoint.publish_motion(Motion())
        remaining = deadline - clock()
        if remaining <= 0.0:
            raise RuntimeError("maximum session duration expired during preparation")
        endpoint.set_motor_power(True, timeout_sec=remaining)
        endpoint.begin_ready_observation()
        remaining = deadline - clock()
        if remaining <= 0.0:
            raise RuntimeError("maximum session duration expired during preparation")
        ready_timeout = min(config.ready_timeout, remaining)
        if not endpoint.wait_for_motor_ready(ready_timeout):
            raise RuntimeError(
                f"/motor_ready did not produce a true sample within {ready_timeout:g}s"
            )

        while True:
            remaining = deadline - clock()
            if remaining <= 0.0:
                return "timeout"
            key = read_key(min(config.key_poll_timeout, remaining))
            if deadline - clock() <= 0.0:
                return "timeout"
            if key == "\x03":
                raise KeyboardInterrupt
            if key is not None:
                profile.handle_key(key)
            endpoint.publish_motion(profile.next_motion())
    except BaseException as error:
        primary_error = error
        raise
    finally:
        cleanup_error = cleanup_session(endpoint, config.zero_cycles)
        if cleanup_error is not None:
            _handle_cleanup_failure(
                cleanup_error, primary_error, report_cleanup_error
            )


class RawTerminal:
    def __enter__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def read_key(self, timeout_sec: float) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not readable:
            return None
        return sys.stdin.read(1)


class HoldTeleopWindow:
    """Handle key release and window focus as physical stop events."""

    RELEASE_DEBOUNCE_MS = 35
    PUBLISH_PERIOD_MS = 50

    def __init__(self, root, endpoint, deadline, clock=time.monotonic, status=None):
        self.root = root
        self.endpoint = endpoint
        self.deadline = deadline
        self.clock = clock
        self.status = status or (lambda message: None)
        self.profile = HeldKeySpeedProfile()
        self.pending_releases = {}
        self.blocked_until_release = set()
        self.closed = False
        self.error = None
        root.bind("<KeyPress>", self.on_press)
        root.bind("<KeyRelease>", self.on_release)
        root.bind("<FocusOut>", self.on_focus_out)
        root.bind("<Control-c>", self.on_close)
        root.bind("<Escape>", self.on_close)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    @staticmethod
    def key_name(event):
        return event.keysym.lower()

    def update_status(self):
        key = self.profile.active_key or "정지"
        self.status(f"속도 {self.profile.speed_stage} · {key}")

    def on_press(self, event):
        key = self.key_name(event)
        if key in self.pending_releases:
            self.root.after_cancel(self.pending_releases.pop(key))
        if key in self.blocked_until_release:
            return
        if key == "space":
            self.blocked_until_release.update(self.profile.held_keys)
        if not self.closed and self.profile.press(key):
            if key == "space":
                self.endpoint.publish_motion(Motion())
            self.update_status()

    def on_release(self, event):
        key = self.key_name(event)
        if self.closed or (key not in self.profile.DIRECTIONS and
                           key not in self.blocked_until_release):
            return
        if key in self.pending_releases:
            self.root.after_cancel(self.pending_releases.pop(key))
        self.pending_releases[key] = self.root.after(
            self.RELEASE_DEBOUNCE_MS, lambda: self.finish_release(key)
        )

    def finish_release(self, key):
        self.pending_releases.pop(key, None)
        if key in self.blocked_until_release:
            self.blocked_until_release.remove(key)
            return
        if not self.closed and self.profile.release(key):
            self.endpoint.publish_motion(Motion())
            self.update_status()

    def on_focus_out(self, _event):
        self.blocked_until_release.update(self.profile.held_keys)
        self.stop_now()

    def stop_now(self):
        for timer in self.pending_releases.values():
            self.root.after_cancel(timer)
        self.pending_releases.clear()
        self.profile.stop()
        self.endpoint.publish_motion(Motion())
        self.update_status()

    def on_close(self, _event=None):
        if self.closed:
            return
        try:
            self.stop_now()
        finally:
            self.closed = True
            self.root.quit()

    def tick(self):
        if self.closed:
            return
        try:
            rclpy.spin_once(self.endpoint.node, timeout_sec=0.0)
            if self.endpoint.motor_ready is False:
                raise RuntimeError("/motor_ready became false during teleop")
            if self.clock() >= self.deadline:
                self.on_close()
                return
            self.endpoint.publish_motion(self.profile.next_motion())
            self.root.after(self.PUBLISH_PERIOD_MS, self.tick)
        except BaseException as error:
            self.error = error
            self.on_close()


def run_hold_gui_session(endpoint, root, *, config=GuardedTeleopConfig(),
                         clock=time.monotonic, status=None):
    deadline = clock() + config.max_session_duration
    primary_error = None
    try:
        for _ in range(config.zero_cycles):
            endpoint.publish_motion(Motion())
        remaining = deadline - clock()
        if remaining <= 0:
            raise RuntimeError("maximum session duration expired during preparation")
        endpoint.set_motor_power(True, timeout_sec=remaining)
        endpoint.begin_ready_observation()
        remaining = deadline - clock()
        if remaining <= 0 or not endpoint.wait_for_motor_ready(
            min(config.ready_timeout, remaining)
        ):
            raise RuntimeError("/motor_ready true not observed before teleop")
        controller = HoldTeleopWindow(root, endpoint, deadline, clock, status)
        controller.update_status()
        root.after(controller.PUBLISH_PERIOD_MS, controller.tick)
        root.focus_force()
        root.mainloop()
        if controller.error is not None:
            raise controller.error
    except BaseException as error:
        primary_error = error
        raise
    finally:
        cleanup_error = cleanup_session(endpoint, config.zero_cycles)
        if cleanup_error is not None:
            _handle_cleanup_failure(cleanup_error, primary_error,
                                    endpoint.report_cleanup_error)


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = config_from_args(args)
    endpoint = None
    root = None
    session_started = False
    primary_error: BaseException | None = None
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        import tkinter as tk

        root = tk.Tk()
        root.title("LeKiwi 텔레옵 · 키를 누르는 동안만 이동")
        root.geometry("460x200")
        root.resizable(False, False)
        label = tk.Label(root, text="W/S 전후 · A/D 좌우 · Q/E 회전", font=("Sans", 15))
        label.pack(pady=(22, 8))
        tk.Label(root, text="방향 키를 놓으면 정지 · 1/2/3 속도 · Space 정지 · Esc 종료",
                 font=("Sans", 10)).pack(pady=5)
        status_text = tk.StringVar(value="모터 준비 중")
        tk.Label(root, textvariable=status_text, font=("Sans", 13)).pack(pady=12)
        root.update()
        endpoint = RosMotorEndpoint()
        endpoint.initialize()
        endpoint.node.get_logger().warning(
            f"Hold-to-move teleop for at most {config.max_session_duration:g}s: "
            "w/s x, a/d y, q/e yaw, 1/2/3 speed, Space stop, Esc exit"
        )
        session_started = True
        run_hold_gui_session(endpoint, root, config=config, status=status_text.set)
        endpoint.node.get_logger().info("hold-to-move teleop ended")
        return 0
    except KeyboardInterrupt as error:
        primary_error = error
        return 130
    except Exception as error:
        primary_error = error
        print(f"guarded teleop failed: {error}", file=sys.stderr)
        return 1
    finally:
        try:
            if endpoint is not None:
                if not session_started:
                    cleanup_error = cleanup_session(endpoint, config.zero_cycles)
                    if cleanup_error is not None:
                        _handle_cleanup_failure(cleanup_error, primary_error,
                                                endpoint.report_cleanup_error)
                endpoint.destroy()
            if root is not None:
                root.destroy()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
