from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util

from launch import LaunchContext
from launch_contract_helpers import forwarded_base_arguments, launch_argument_defaults


def read_launch(name: str) -> str:
    return Path("launch", name).read_text(encoding="utf-8")


def test_factory_profile_calibration_is_applied_to_default_launch_values() -> None:
    spec = importlib.util.spec_from_file_location(
        'lekiwi_base_launch', Path(__file__).resolve().parents[1] / 'launch/base.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with TemporaryDirectory() as directory:
        profile = Path(directory) / 'base_profile.yaml'
        profile.write_text(
            'lekiwi_node:\n  ros__parameters:\n'
            '    speed_tick_scale: 700.0\n    encoder_odom_scale: 0.82\n'
            '    encoder_tick_deadband: 55\n',
            encoding='utf-8',
        )
        context = LaunchContext()
        context.launch_configurations.update({
            'base_params_file': str(profile),
            **module.PROFILE_CALIBRATION_DEFAULTS,
            'encoder_tick_deadband': '25',
        })
        module.apply_profile_calibration(context)
        assert context.launch_configurations['speed_tick_scale'] == '700.0'
        assert context.launch_configurations['encoder_odom_scale'] == '0.82'
        assert context.launch_configurations['encoder_tick_deadband'] == '25'


def test_robot_launch_forwards_hardware_arguments() -> None:
    overrides = {"base_params_file": "/tmp/base_profile.yaml", "motor_backend": "mock",
                 "enable_motor_write": "false", "serial_port": "/tmp/mock_bus",
                 "encoder_odom_scale": "0.82"}
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    forwarded = forwarded_base_arguments()
    for name, value in overrides.items():
        assert ''.join(item.perform(context) for item in forwarded[name]) == value
    assert launch_argument_defaults("robot.launch.py")["encoder_odom_scale"] == "1.0"


def test_robot_launch_forwards_nav2_velocity_envelope() -> None:
    expected_defaults = {
        "max_linear_x": "0.11", "max_linear_y": "0.1", "max_linear_speed": "0.11",
        "max_angular_z": "0.75", "max_wheel_speed": "2.3009711818284617",
        "speed_tick_limit": "1500",
    }
    defaults = launch_argument_defaults("robot.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({name: "0.03" for name in expected_defaults})
    forwarded = forwarded_base_arguments()
    for name, default in expected_defaults.items():
        assert defaults[name] == default
        assert ''.join(item.perform(context) for item in forwarded[name]) == "0.03"


def test_standard_launch_connects_hardware_with_torque_disabled() -> None:
    for name in ("robot.launch.py", "base.launch.py"):
        defaults = launch_argument_defaults(name)
        for argument, default in (
            ("use_imu", "true"), ("motor_backend", "feetech"), ("enable_motor_write", "true"),
            ("torque_enable", "false"), ("serial_port", "/dev/ttyACM0"),
            ("encoder_odom_scale", "1.0"),
        ):
            assert defaults[argument] == default


def test_base_launch_exposes_nav2_velocity_envelope() -> None:
    defaults = launch_argument_defaults("base.launch.py")
    expected_defaults = {
        "max_linear_x": "0.11", "max_linear_y": "0.1", "max_linear_speed": "0.11",
        "max_angular_z": "0.75", "max_wheel_speed": "2.3009711818284617",
        "speed_tick_limit": "1500",
    }
    for name, default in expected_defaults.items():
        assert defaults[name] == default


def test_base_launch_keeps_file_defaults_before_explicit_overrides() -> None:
    launch = read_launch("base.launch.py")

    assert "base_params_file = LaunchConfiguration('base_params_file')" in launch
    assert "parameters=[base_params_file, geometry_params_file, {" in launch
    assert "'motor_backend': motor_backend" in launch
    assert "'enable_motor_write': ParameterValue(enable_motor_write, value_type=bool)" in launch
    assert "'serial_port': serial_port" in launch
    for name in (
        "max_linear_x",
        "max_linear_y",
        "max_linear_speed",
        "max_angular_z",
        "max_wheel_speed",
    ):
        assert f"'{name}': ParameterValue({name}, value_type=float)" in launch
    assert "'speed_tick_limit': ParameterValue(speed_tick_limit, value_type=int)" in launch
    assert "'encoder_odom_scale': ParameterValue(encoder_odom_scale, value_type=float)" in launch


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-o", f"pythonpath={Path(__file__).resolve().parents[1]}"]))
