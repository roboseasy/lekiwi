import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

from lekiwi_teleop.distance_trial import Trial, compare, displacement, main, run_trial


class FakeEndpoint:
    """Independent planar plant with controllable faults and a monotonic fake clock."""
    def __init__(self, *, mock=False, gain=.9, fault=None):
        self.now = 100.
        self.mock, self.gain, self.fault = mock, gain, fault
        self.ready = False
        self.error = None
        self.x, self.y, self.yaw = 4., 7., math.pi / 2
        self.velocity = (0., 0.)
        self.samples, self.commands, self.powers = [], [], []
        self.cleanup_count = 0
        self.latest = None
        self.motion_started = None
        self.spin(.01)

    def parameters(self):
        return {"motor_backend": "mock" if self.mock else "feetech",
                "enable_motor_write": not self.mock, "torque_enable": False,
                "odom_source": "command" if self.mock else "encoder",
                "encoder_feedback_source": "position", "use_sim_time": False,
                "cmd_vel_timeout": .3, "max_linear_x": .03, "max_linear_y": .03,
                "max_linear_speed": .03, "wheel_radius": .0508,
                "max_wheel_speed": 1., "speed_tick_limit": 600., "speed_tick_scale": 600.}

    def check_graph(self):
        if self.fault == "publisher" and self.motion_started is not None:
            raise RuntimeError("Another cmd_vel publisher exists")

    def spin(self, duration):
        if self.fault == "interrupt" and self.motion_started is not None:
            self.fault = None
            raise KeyboardInterrupt()
        if self.fault == "stall" and self.motion_started is not None:
            duration += .2
        self.now += duration
        vx, vy = (v * self.gain for v in self.velocity)
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * duration
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * duration
        if self.motion_started is not None:
            if self.fault == "stale":
                return
            if self.fault == "ready":
                self.ready = False
            if self.fault == "drift":
                self.x += .04
        self.latest = {"x": self.x, "y": self.y, "yaw": self.yaw,
                       "vx": vx, "vy": vy, "wz": 0., "received_s": self.now,
                       "stamp_ns": round(self.now * 1e9)}
        self.samples.append(self.latest)

    def publish(self, x, y):
        self.velocity = (x, y)
        self.commands.append((self.now, x, y))
        if (x or y) and self.motion_started is None:
            self.motion_started = self.now

    def arm(self):
        self.powers.append(True)
        if self.fault == "arm":
            raise RuntimeError("arm response timed out")
        self.ready = True

    def stop_and_disarm(self):
        self.cleanup_count += 1
        self.publish(0., 0.)
        self.powers.append(False)
        self.ready = False
        if self.fault == "cleanup":
            raise RuntimeError("torque-off failed")


class TestDistanceTrial(unittest.TestCase):
    def run_fake(self, **kwargs):
        endpoint = FakeEndpoint(**kwargs)
        report = run_trial(endpoint, Trial(), {}, mock=endpoint.mock, clock=lambda: endpoint.now)
        return endpoint, report

    def test_default_plan_is_twenty_cm_in_ten_seconds(self):
        self.assertEqual(Trial().duration_s, 10.)
        self.assertEqual(Trial().velocity, (.02, 0.))

    def test_invalid_or_excessive_motion_is_rejected(self):
        for options in ({"speed_mps": float("nan")}, {"distance_m": float("inf")},
                        {"distance_m": 0}, {"speed_mps": -.01}, {"speed_mps": .04},
                        {"distance_m": .31}, {"speed_mps": .001}, {"direction": "cw"}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                Trial(**options)

    def test_rotated_start_and_all_translation_directions(self):
        start = {"x": 3., "y": -7., "yaw": math.pi / 2}
        for direction, offset in (("forward", (0., .2)), ("backward", (0., -.2)),
                                  ("left", (-.2, 0.)), ("right", (.2, 0.))):
            end = dict(start, x=start["x"] + offset[0], y=start["y"] + offset[1])
            actual = displacement(start, end, direction)
            self.assertAlmostEqual(actual["along_m"], .2)
            self.assertAlmostEqual(actual["cross_m"], 0.)

    def test_yaw_wrap_and_cross_track_are_preserved(self):
        start = {"x": 0., "y": 0., "yaw": math.pi - .01}
        end = {"x": -.2, "y": -.03, "yaw": -math.pi + .01}
        result = displacement(start, end, "forward")
        self.assertAlmostEqual(result["yaw_rad"], .02)
        self.assertGreater(result["cross_m"], .03)

    def test_time_based_run_does_not_drive_until_odom_target(self):
        endpoint, report = self.run_fake()
        self.assertTrue(report["completed"], report["errors"])
        self.assertAlmostEqual(report["command_duration_s"], 10.)
        self.assertAlmostEqual(report["odom"]["along_m"], .18)
        self.assertAlmostEqual(report["published_command_integral_m"], .2)
        self.assertEqual(endpoint.powers, [True, False])
        self.assertEqual(endpoint.commands[-1][1:], (0., 0.))
        self.assertFalse(report["physical_stop_confirmed"])
        self.assertFalse(report["torque_off_register_verified"])

    def test_stopping_tail_is_included(self):
        class CoastingEndpoint(FakeEndpoint):
            coast_left = .25

            def spin(self, duration):
                if self.motion_started is not None and self.velocity == (0., 0.) and self.coast_left > 0:
                    dt = min(duration, self.coast_left)
                    self.y += .01 * dt
                    self.coast_left -= dt
                super().spin(duration)

        endpoint = CoastingEndpoint(gain=1.)
        report = run_trial(endpoint, Trial(), {}, clock=lambda: endpoint.now)
        self.assertTrue(report["completed"], report["errors"])
        self.assertAlmostEqual(report["odom"]["along_m"], .2025)

    def test_faults_and_interruptions_always_attempt_cleanup(self):
        for fault in ("stale", "ready", "drift", "publisher", "arm", "interrupt", "stall", "cleanup"):
            with self.subTest(fault=fault):
                endpoint, report = self.run_fake(fault=fault)
                self.assertFalse(report["completed"])
                self.assertTrue(report["errors"])
                self.assertEqual(endpoint.cleanup_count, 1)
                self.assertEqual(endpoint.commands[-1][1:], (0., 0.))
                self.assertEqual(endpoint.powers[-1], False)

    def test_preflight_refusal_does_not_touch_motors(self):
        endpoint = FakeEndpoint()
        report = run_trial(endpoint, Trial(), {}, mock=True, clock=lambda: endpoint.now)
        self.assertFalse(report["completed"])
        self.assertEqual(endpoint.commands, [])
        self.assertEqual(endpoint.powers, [])

    def test_already_armed_controller_is_rejected(self):
        endpoint = FakeEndpoint()
        endpoint.ready = True
        report = run_trial(endpoint, Trial(), {}, clock=lambda: endpoint.now)
        self.assertFalse(report["completed"])
        self.assertEqual(endpoint.powers, [])

    def test_clipped_speed_is_rejected_before_arming(self):
        endpoint = FakeEndpoint()
        original = endpoint.parameters
        endpoint.parameters = lambda: dict(original(), max_linear_x=.01)
        report = run_trial(endpoint, Trial(), {}, clock=lambda: endpoint.now)
        self.assertFalse(report["completed"])
        self.assertEqual(endpoint.powers, [])

    def test_non_finite_controller_limit_is_rejected_before_arming(self):
        endpoint = FakeEndpoint()
        original = endpoint.parameters
        endpoint.parameters = lambda: dict(original(), max_linear_x=float("nan"))
        report = run_trial(endpoint, Trial(), {}, clock=lambda: endpoint.now)
        self.assertFalse(report["completed"])
        self.assertEqual(endpoint.powers, [])

    def test_mock_requires_mock_controller_and_never_arms(self):
        endpoint, report = self.run_fake(mock=True, gain=1.)
        self.assertTrue(report["completed"], report["errors"])
        self.assertNotIn(True, endpoint.powers)
        with self.assertRaisesRegex(RuntimeError, "Mock"):
            compare(report, .2)

    def test_command_and_odometry_errors_are_separate(self):
        _, report = self.run_fake()
        result = compare(report, .18)
        self.assertFalse(result["actual_within_tolerance"])
        self.assertTrue(result["odom_within_tolerance"])
        self.assertAlmostEqual(result["actual_minus_requested_m"], -.02)
        report["odom"]["along_m"] = .2
        self.assertFalse(compare(report, .18)["odom_within_tolerance"])

    def test_zero_or_wrong_direction_measurement_is_valid_failure(self):
        _, report = self.run_fake()
        for measured in (0., -.2):
            result = compare(report, measured)
            self.assertFalse(result["actual_within_tolerance"])
            self.assertFalse(result["odom_within_tolerance"])

    def test_failed_trial_and_non_finite_measurements_cannot_pass(self):
        _, report = self.run_fake()
        for value in (float("nan"), float("inf")):
            with self.assertRaises(RuntimeError):
                compare(report, value)
        report["completed"] = False
        with self.assertRaises(RuntimeError):
            compare(report, .2)

    def test_plan_has_no_ros_import_or_motion_requirement(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main(["plan"]), 0)
        self.assertEqual(json.loads(output.getvalue())["stop_rule"], "elapsed_time")

    def test_hardware_run_without_confirmation_does_not_create_report(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "trial.json"
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(["run", "--output", str(path)])
            self.assertFalse(path.exists())

    def test_compare_writes_new_file_and_preserves_source(self):
        _, report = self.run_fake()
        with tempfile.TemporaryDirectory() as folder:
            source, output = Path(folder) / "trial.json", Path(folder) / "comparison.json"
            source.write_text(json.dumps(report))
            before = source.read_bytes()
            args = ["compare", "--report", str(source), "--measured", ".18", "--output", str(output)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            self.assertTrue(json.loads(output.read_text())["odom_within_tolerance"])
            self.assertEqual(source.read_bytes(), before)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args)


if __name__ == "__main__":
    unittest.main()
