import argparse
import contextlib
import io
import re
import subprocess
import unittest
from pathlib import Path

from lekiwi_teleop.odom_calibration import (
    CalibrationSample,
    build_parser,
    calculate_scale_recommendation,
    finite_non_zero_float,
    finite_positive_float,
    main,
)


class TestOdomCalibration(unittest.TestCase):
    def test_calculate_scale_recommendation_scales_current_calibration(self):
        sample = CalibrationSample(
            current_scale=1.0,
            odom_delta=0.6,
            measured_delta=0.5,
        )

        recommendation = calculate_scale_recommendation(sample)

        self.assertAlmostEqual(recommendation.recommended_scale, 5.0 / 6.0)
        self.assertAlmostEqual(recommendation.odom_to_measured_ratio, 1.2)

    def test_calculate_scale_recommendation_uses_absolute_odom_delta_for_reverse_motion(self):
        sample = CalibrationSample(
            current_scale=1.0,
            odom_delta=-0.48,
            measured_delta=0.5,
        )

        recommendation = calculate_scale_recommendation(sample)

        self.assertAlmostEqual(recommendation.recommended_scale, 0.5 / 0.48)
        self.assertAlmostEqual(recommendation.odom_to_measured_ratio, 0.96)

    def test_calculate_scale_recommendation_rejects_invalid_inputs(self):
        with self.assertRaisesRegex(ValueError, "current_scale"):
            calculate_scale_recommendation(
                CalibrationSample(
                    current_scale=0.0,
                    odom_delta=0.5,
                    measured_delta=0.5,
                )
            )

        with self.assertRaisesRegex(ValueError, "odom_delta"):
            calculate_scale_recommendation(
                CalibrationSample(
                    current_scale=1.0,
                    odom_delta=0.0,
                    measured_delta=0.5,
                )
            )

        with self.assertRaisesRegex(ValueError, "measured_delta"):
            calculate_scale_recommendation(
                CalibrationSample(
                    current_scale=1.0,
                    odom_delta=0.5,
                    measured_delta=0.0,
                )
            )

    def test_parser_feeds_calculation_contract(self):
        parser = build_parser()
        args = parser.parse_args(
            [
                "--current-scale",
                "1.0",
                "--odom-delta",
                "0.45",
                "--measured-delta",
                "0.5",
            ]
        )

        recommendation = calculate_scale_recommendation(CalibrationSample.from_args(args))

        self.assertAlmostEqual(recommendation.recommended_scale, 0.5 / 0.45)

    def test_main_prints_recommended_launch_argument(self):
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            exit_code = main(
                [
                    "--current-scale",
                    "1.0",
                    "--odom-delta",
                    "0.6",
                    "--measured-delta",
                    "0.5",
                ]
            )

        self.assertEqual(exit_code, 0)
        output = stdout.getvalue()
        self.assertIn("recommended encoder_odom_scale: 0.833333", output)
        self.assertIn("launch argument: encoder_odom_scale:=0.833333", output)

    def test_finite_positive_float_rejects_non_positive_or_non_finite_values(self):
        with self.assertRaisesRegex(argparse.ArgumentTypeError, "0보다 큰 유한한 값"):
            finite_positive_float("0")
        with self.assertRaisesRegex(argparse.ArgumentTypeError, "0보다 큰 유한한 값"):
            finite_positive_float("nan")

    def test_finite_non_zero_float_rejects_zero_or_non_finite_values(self):
        self.assertEqual(finite_non_zero_float("-0.5"), -0.5)

        with self.assertRaisesRegex(argparse.ArgumentTypeError, "0이 아닌 유한한 값"):
            finite_non_zero_float("0")
        with self.assertRaisesRegex(argparse.ArgumentTypeError, "0이 아닌 유한한 값"):
            finite_non_zero_float("inf")

    def test_parser_rejects_invalid_odom_delta_before_calculation(self):
        parser = build_parser()

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as zero_context:
                parser.parse_args(
                    [
                        "--current-scale",
                        "1.0",
                        "--odom-delta",
                        "0",
                        "--measured-delta",
                        "0.5",
                    ]
                )

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as nan_context:
                parser.parse_args(
                    [
                        "--current-scale",
                        "1.0",
                        "--odom-delta",
                        "nan",
                        "--measured-delta",
                        "0.5",
                    ]
                )

        self.assertEqual(zero_context.exception.code, 2)
        self.assertEqual(nan_context.exception.code, 2)

    def test_floor_calibration_documents_scale_and_repeatability_acceptance(self):
        repo_root = Path(__file__).resolve().parents[2]
        procedure = (
            repo_root / "docs" / "hardware" / "encoder_odom_floor_calibration.md"
        ).read_text(encoding="utf-8")

        self.assertIn("encoder_odom_scale", procedure)
        self.assertIn(">=3", procedure)
        self.assertIn("<=5%", procedure)
        self.assertIn("<=3%", procedure)
        for token in (
            "translation trials per direction >=3",
            "yaw trials per direction >=3",
            "d_odom = hypot(x_end - x_start, y_end - y_start)",
            "translation_error_pct = 100 * abs(d_odom - d_measured) / d_measured",
            "yaw_delta_error = atan2(sin(yaw_odom - yaw_measured), cos(yaw_odom - yaw_measured))",
            "yaw_error_pct = 100 * abs(yaw_delta_error) / abs(yaw_measured)",
            "translation_ratio_i = d_measured_i / d_odom_i",
            "yaw_ratio_i = abs(yaw_measured_i) / abs(yaw_odom_i)",
            "STOP: fresh user confirmation required immediately before first non-zero command",
            "calibration set: forward translation only",
            "forward_ratio_i = d_measured_i / d_odom_i",
            "forward_ratio_mean = sum(forward_ratio_i) / n_forward",
            "forward_repeat_variation_pct = 100 * (max(forward_ratio_i) - min(forward_ratio_i)) / forward_ratio_mean",
            "recommended_scale = current_scale * forward_ratio_mean",
            "exactly one recommended scale",
            "do not compute a second scale from backward/left/right/yaw validation",
            "abort without a recommended scale",
            "validation_direction_mean_d = sum(translation_ratio_i for direction d) / n_d",
            "translation_direction_spread_pct = 100 * (max(validation_direction_mean_d) - min(validation_direction_mean_d)) / mean(validation_direction_mean_d)",
            "pre-arm /motor_ready false",
            "Airborne power/readiness transition already accepted",
        ):
            self.assertIn(token, procedure)

        self.assertEqual(
            procedure.count("recommended_scale = current_scale * forward_ratio_mean"), 1
        )
        self.assertLess(
            procedure.index("calibration set: forward translation only"),
            procedure.index("recommended_scale = current_scale * forward_ratio_mean"),
        )
        self.assertLess(
            procedure.index("recommended_scale = current_scale * forward_ratio_mean"),
            procedure.index("do not compute a second scale from backward/left/right/yaw validation"),
        )

        marker_index = procedure.index(
            "STOP: fresh user confirmation required immediately before first non-zero command"
        )
        command_index = procedure.index(
            "ros2 run lekiwi_teleop lekiwi_safe_cmd_vel \\\n"
        )
        self.assertLess(marker_index, command_index)
        confirmation_index = procedure.index(
            'test "$FRESH_CONFIRMATION" = "$EXPECTED_CONFIRMATION"'
        )
        self.assertLess(confirmation_index, command_index)

    def test_floor_trial_matrix_is_executable_and_records_only_after_physical_stop(self):
        repo_root = Path(__file__).resolve().parents[2]
        procedure = (
            repo_root / "docs" / "hardware" / "encoder_odom_floor_calibration.md"
        ).read_text(encoding="utf-8")
        match = re.search(
            r"```bash\n(?P<block>[^`]*# FLOOR_TRIAL_MATRIX_START[\s\S]*?)\n```",
            procedure,
        )
        self.assertIsNotNone(match)
        block = match.group("block")
        syntax = subprocess.run(
            ["bash", "-n"], input=block, text=True, capture_output=True, check=False
        )
        self.assertEqual(syntax.returncode, 0, syntax.stderr)

        trial = block[
            block.index("# RUN_FLOOR_TRIAL_BEGIN") : block.index("# RUN_FLOOR_TRIAL_END")
        ]
        confirmation_index = trial.index("EXPECTED_CONFIRMATION=")
        self.assertLess(trial.index("grep -Eq '^data: false$'"), confirmation_index)
        reset_index = trial.index("/reset_odometry")
        command_index = trial.index("lekiwi_safe_cmd_vel")
        disable_index = trial.index('/motor_power std_srvs/srv/SetBool "{data: false}"')
        ready_false_index = trial.index("grep -Eq '^data: false$'", disable_index)
        physical_stop_index = trial.index("Physical stop confirmation", ready_false_index)
        record_index = trial.index("/odom --once", physical_stop_index)
        self.assertLess(reset_index, command_index)
        self.assertLess(command_index, disable_index)
        self.assertLess(disable_index, ready_false_index)
        self.assertLess(ready_false_index, physical_stop_index)
        self.assertLess(physical_stop_index, record_index)


if __name__ == "__main__":
    unittest.main()
