"""One timed, low-speed translation; compare odometry with a later floor measurement.

Planning and comparison need no ROS. A run connects to an existing base controller;
it never launches hardware, resets odometry, or changes calibration parameters.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import signal
import time


DIRECTIONS = {"forward": (1., 0.), "backward": (-1., 0.),
              "left": (0., 1.), "right": (0., -1.)}
PERIOD = .05
MAX_GAP = .15
ODOM_MAX_AGE = .3


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


@dataclass(frozen=True)
class Trial:
    distance_m: float = .2
    speed_mps: float = .02
    direction: str = "forward"

    def __post_init__(self):
        if self.direction not in DIRECTIONS:
            raise ValueError("Unknown translation direction")
        if not math.isfinite(self.distance_m) or not 0 < self.distance_m <= .3:
            raise ValueError("distance must be in (0, 0.3] m")
        if not math.isfinite(self.speed_mps) or not 0 < self.speed_mps <= .03:
            raise ValueError("speed must be in (0, 0.03] m/s")
        if not .5 <= self.duration_s <= 30:
            raise ValueError("distance / speed must be in [0.5, 30] seconds")

    @property
    def duration_s(self):
        return self.distance_m / self.speed_mps

    @property
    def velocity(self):
        return tuple(v * self.speed_mps for v in DIRECTIONS[self.direction])

    def plan(self):
        return {**asdict(self), "duration_s": self.duration_s, "rate_hz": 20,
                "velocity_xy_mps": self.velocity, "stop_rule": "elapsed_time",
                "actual_distance_guaranteed": False}


def displacement(start, end, direction):
    """Project endpoint displacement onto the commanded axis at the start pose."""
    dx, dy = end["x"] - start["x"], end["y"] - start["y"]
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    x, y = c * dx + s * dy, -s * dx + c * dy
    ux, uy = DIRECTIONS[direction]
    yaw = end["yaw"] - start["yaw"]
    return {"along_m": ux * x + uy * y, "cross_m": -uy * x + ux * y,
            "net_distance_m": math.hypot(dx, dy),
            "yaw_rad": math.atan2(math.sin(yaw), math.cos(yaw))}


def validate_parameters(p, trial, mock):
    expected = {"motor_backend": "mock" if mock else "feetech",
                "enable_motor_write": not mock, "torque_enable": False,
                "odom_source": "command" if mock else "encoder",
                "use_sim_time": False}
    if not mock:
        expected["encoder_feedback_source"] = "position"
    for key, value in expected.items():
        require(p.get(key) == value, f"Unexpected {key}: {p.get(key)!r}; expected {value!r}")
    for key in ("cmd_vel_timeout", "max_linear_x", "max_linear_y", "max_linear_speed",
                "wheel_radius", "max_wheel_speed", "speed_tick_limit", "speed_tick_scale"):
        require(isinstance(p.get(key), (int, float)) and math.isfinite(p[key]) and p[key] > 0,
                f"Invalid numeric parameter: {key}")
    require(MAX_GAP < p["cmd_vel_timeout"] <= .5, "Unsuitable cmd_vel_timeout")
    axis = "max_linear_x" if trial.velocity[0] else "max_linear_y"
    require(min(p[axis], p["max_linear_speed"]) >= trial.speed_mps,
            "Controller would clamp requested body speed")
    # Pure translation wheel speeds cannot exceed |v| / r, regardless of frame yaw.
    require(p["wheel_radius"] > 0 and p["speed_tick_scale"] > 0,
            "Invalid wheel radius or speed scale")
    wheel_limit = min(p["max_wheel_speed"], p["speed_tick_limit"] / p["speed_tick_scale"])
    require(wheel_limit >= trial.speed_mps / p["wheel_radius"],
            "Controller would clamp requested wheel speed")


def check_sample(endpoint, now, *, ready=None):
    require(endpoint.error is None, f"Invalid odometry: {endpoint.error}")
    sample = endpoint.latest
    require(sample is not None and 0 <= now - sample["received_s"] <= ODOM_MAX_AGE,
            "Missing or stale odometry")
    if ready is not None:
        require(endpoint.ready is ready, f"motor_ready must be {ready}")
    endpoint.check_graph()
    return sample


def stationary(endpoint, *, clock, seconds, ready, timeout=5.):
    """Require a continuous fresh, stable window, not merely a zero twist message."""
    deadline = clock() + timeout
    window = []
    while clock() < deadline:
        endpoint.publish(0., 0.)
        endpoint.spin(PERIOD)
        sample = check_sample(endpoint, clock(), ready=ready)
        if not window or sample["stamp_ns"] != window[-1]["stamp_ns"]:
            window.append(sample.copy())
        while len(window) > 1 and window[-1]["received_s"] - window[1]["received_s"] >= seconds:
            window.pop(0)
        if len(window) < 5 or window[-1]["received_s"] - window[0]["received_s"] < seconds:
            continue
        deltas = [displacement(window[0], item, "forward") for item in window]
        if (max(d["net_distance_m"] for d in deltas) <= .0005
                and max(abs(d["yaw_rad"]) for d in deltas) <= .003
                and all(math.hypot(s["vx"], s["vy"]) <= .003 and abs(s["wz"]) <= .02
                        for s in window)):
            return sample.copy()
    raise RuntimeError("Odometry did not become stationary within the stop deadline")


def run_trial(endpoint, trial, report, *, mock=False, clock=time.monotonic):
    """Dependency-injected motion lifecycle, also exercised with fake time/feedback."""
    report.update(schema=1, mode="mock" if mock else "hardware", plan=trial.plan(),
                  completed=False, physical_stop_confirmed=False,
                  torque_off_register_verified=False, errors=[])
    owns_motion = False
    try:
        report["parameters"] = endpoint.parameters()
        validate_parameters(report["parameters"], trial, mock)
        endpoint.spin(.5)  # Allow local graph and latched readiness discovery.
        check_sample(endpoint, clock(), ready=False)
        owns_motion = True
        stationary(endpoint, clock=clock, seconds=1., ready=False)
        if not mock:
            endpoint.arm()
        expected_ready = not mock
        start = stationary(endpoint, clock=clock, seconds=1., ready=expected_ready)
        report["start"] = start
        started = deadline = None
        last_publish = None
        report["max_publish_gap_s"] = 0.
        while deadline is None or clock() < deadline:
            now = clock()
            if last_publish is not None:
                gap = now - last_publish
                report["max_publish_gap_s"] = max(report["max_publish_gap_s"], gap)
                require(gap <= MAX_GAP, "Command publishing stalled")
            current = check_sample(endpoint, now, ready=expected_ready)
            d = displacement(start, current, trial.direction)
            require(-.02 <= d["along_m"] <= trial.distance_m + .05,
                    "Odometry exceeded the travel guard")
            require(abs(d["cross_m"]) <= .03 and abs(d["yaw_rad"]) <= .2,
                    "Excessive sideways or angular odometry drift")
            # Do not issue a late nonzero command after slow graph checks.
            if deadline is not None and clock() >= deadline:
                break
            sent = clock()
            endpoint.publish(*trial.velocity)
            last_publish = sent
            if started is None:
                started = sent
                deadline = started + trial.duration_s
                report["command_started_s"] = started
            endpoint.spin(min(PERIOD, max(0., deadline - clock())))
        ended = clock()
        endpoint.publish(0., 0.)
        report["command_ended_s"] = ended
        report["command_duration_s"] = ended - started
        report["published_command_integral_m"] = trial.speed_mps * (ended - started)
        require(abs(ended - deadline) <= .1, "Motion duration exceeded timing tolerance")
        # Retain encoder readiness through the stopping tail; disarm in finally.
        end = stationary(endpoint, clock=clock, seconds=2., ready=expected_ready)
        report["end"] = end
        report["odom"] = displacement(start, end, trial.direction)
        require(-.02 <= report["odom"]["along_m"] <= trial.distance_m + .05
                and abs(report["odom"]["cross_m"]) <= .03
                and abs(report["odom"]["yaw_rad"]) <= .2,
                "Stopping tail exceeded the travel guard")
        report["odom_minus_requested_m"] = report["odom"]["along_m"] - trial.distance_m
        report["odom_stop_window_passed"] = True
        report["completed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report["completed"] = False
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        if owns_motion:
            try:
                endpoint.stop_and_disarm()
                report["torque_off_service_acknowledged"] = True
            except (Exception, KeyboardInterrupt) as error:
                report["completed"] = False
                report["torque_off_service_acknowledged"] = False
                report["errors"].append(f"Cleanup failed: {error}")
        report["samples"] = endpoint.samples
    return report


def compare(report, measured_m, tolerance_m=.01):
    require(report.get("schema") == 1 and report.get("completed") is True,
            "Only a completed distance trial can be compared")
    require(report.get("mode") == "hardware", "Mock odometry is not a physical measurement")
    require(math.isfinite(measured_m), "Measured displacement must be finite")
    require(math.isfinite(tolerance_m) and tolerance_m > 0, "Tolerance must be positive")
    requested = report["plan"]["distance_m"]
    odom = report["odom"]["along_m"]
    return {"requested_m": requested, "measured_along_m": measured_m,
            "odom_along_m": odom, "tolerance_m": tolerance_m,
            "actual_minus_requested_m": measured_m - requested,
            "odom_minus_actual_m": odom - measured_m,
            "actual_within_tolerance": abs(measured_m - requested) <= tolerance_m,
            "odom_within_tolerance": abs(odom - measured_m) <= tolerance_m,
            "odom_cross_m": report["odom"]["cross_m"],
            "odom_yaw_rad": report["odom"]["yaw_rad"],
            "scope": "one trial, signed displacement along the initial command axis"}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    for name in ("plan", "run"):
        sub = commands.add_parser(name)
        sub.add_argument("--distance", type=float, default=.2, help="Requested metres")
        sub.add_argument("--speed", type=float, default=.02, help="Metres per second")
        sub.add_argument("--direction", choices=DIRECTIONS, default="forward")
        if name == "run":
            sub.add_argument("--output", type=Path, required=True, help="New JSON report path")
            sub.add_argument("--mock", action="store_true", help="Require a mock base")
            sub.add_argument("--yes-i-confirm-motion", action="store_true")
    sub = commands.add_parser("compare")
    sub.add_argument("--report", type=Path, required=True)
    sub.add_argument("--measured", type=float, required=True,
                     help="Signed metres along the initial commanded direction; zero is valid")
    sub.add_argument("--tolerance", type=float, default=.01, help="Absolute tolerance, metres")
    sub.add_argument("--output", type=Path, required=True, help="New comparison JSON path")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.action == "compare":
            result = compare(json.loads(args.report.read_text()), args.measured, args.tolerance)
            with args.output.open("x") as output:
                json.dump(result, output, indent=2, allow_nan=False)
            print(json.dumps(result, indent=2))
            return 0
        trial = Trial(args.distance, args.speed, args.direction)
        if args.action == "plan":
            print(json.dumps(trial.plan(), indent=2))
            return 0
        if not args.mock and not args.yes_i_confirm_motion:
            parser.error("Hardware run requires --yes-i-confirm-motion after on-site preparation")
        # Reserve the output before any ROS or hardware operation; never overwrite a trial.
        with args.output.open("x") as output:
            report = {"schema": 1, "completed": False, "plan": trial.plan(), "errors": []}
            json.dump(report, output, indent=2)
            output.flush()
            endpoint = None
            handlers = {}
            try:
                from lekiwi_teleop.distance_trial_ros import DistanceEndpoint
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                    handlers[sig] = signal.signal(sig, interrupt)
                endpoint = DistanceEndpoint()
                endpoint.initialize()
                run_trial(endpoint, trial, report, mock=args.mock)
            except (Exception, KeyboardInterrupt) as error:
                report["completed"] = False
                report["errors"].append(f"{type(error).__name__}: {error}")
            finally:
                try:
                    if endpoint is not None:
                        endpoint.destroy()
                except Exception as error:
                    report["completed"] = False
                    report["errors"].append(f"ROS shutdown failed: {error}")
                finally:
                    for sig, handler in handlers.items():
                        signal.signal(sig, handler)
                    output.seek(0)
                    json.dump(report, output, indent=2, allow_nan=False)
                    output.truncate()
            print(json.dumps({key: value for key, value in report.items() if key != "samples"},
                             indent=2))
            return 0 if report["completed"] else 1
    except (ValueError, OSError, RuntimeError) as error:
        parser.error(str(error))


def interrupt(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


if __name__ == "__main__":
    raise SystemExit(main())
