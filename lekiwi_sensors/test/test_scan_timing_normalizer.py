from pathlib import Path
import importlib.machinery
import importlib.util
from types import SimpleNamespace
import sys
import re
import subprocess
import tempfile

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[0]


def load_normalizer_module():
    script_path = PACKAGE_ROOT / "scripts" / "scan_timing_normalizer"
    loader = importlib.machinery.SourceFileLoader("scan_timing_normalizer", str(script_path))
    spec = importlib.util.spec_from_loader("scan_timing_normalizer", loader)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_normalize_scan_timing_zeroes_time_increment_without_mutating_input() -> None:
    normalizer = load_normalizer_module()
    scan = SimpleNamespace(
        header=SimpleNamespace(frame_id="lidar_link"),
        ranges=[1.0, 2.0, 3.0],
        scan_time=0.1,
        time_increment=0.00025,
    )

    normalized = normalizer.normalize_scan_timing(scan)

    assert normalized is not scan
    assert normalized.header.frame_id == "lidar_link"
    assert normalized.ranges == [1.0, 2.0, 3.0]
    assert normalized.scan_time == 0.1
    assert normalized.time_increment == 0.0
    assert scan.time_increment == 0.00025


def test_normalize_scan_timing_can_preserve_time_increment_when_disabled() -> None:
    normalizer = load_normalizer_module()
    scan = SimpleNamespace(time_increment=0.00025)

    normalized = normalizer.normalize_scan_timing(scan, zero_time_increment=False)

    assert normalized.time_increment == 0.00025


def test_topic_pair_rejects_resolved_self_feedback_topics() -> None:
    normalizer = load_normalizer_module()
    resolve = lambda name: "/" + name.lstrip("/")
    for input_topic, output_topic in (("/scan", "/scan"), ("scan", "/scan")):
        try:
            normalizer.validate_distinct_topics(input_topic, output_topic, resolve)
        except ValueError:
            pass
        else:
            raise AssertionError((input_topic, output_topic))


def test_topic_pair_rejects_empty_topics_before_resolution() -> None:
    normalizer = load_normalizer_module()

    def reject_if_resolved(_name: str) -> str:
        raise AssertionError("empty topics must be rejected before resolution")

    for input_topic, output_topic in (
        ("", "/scan"),
        ("   ", "/scan"),
        ("/scan_raw", ""),
        ("/scan_raw", "   "),
    ):
        try:
            normalizer.validate_distinct_topics(
                input_topic,
                output_topic,
                reject_if_resolved,
            )
        except ValueError:
            pass
        else:
            raise AssertionError((input_topic, output_topic))


def test_topic_pair_accepts_distinct_resolved_topics() -> None:
    normalizer = load_normalizer_module()
    resolved_names = []

    def resolve(name: str) -> str:
        resolved = "/robot/" + name.lstrip("/")
        resolved_names.append(resolved)
        return resolved

    normalizer.validate_distinct_topics("scan_raw", "scan", resolve)

    assert resolved_names == ["/robot/scan_raw", "/robot/scan"]


def test_node_validates_topics_before_creating_ros_endpoints() -> None:
    script = (PACKAGE_ROOT / "scripts" / "scan_timing_normalizer").read_text(
        encoding="utf-8"
    )
    node_constructor = script.split("class _NormalizerNode", maxsplit=1)[1]

    validation_position = node_constructor.index("validate_distinct_topics(")
    publisher_position = node_constructor.index("self.create_publisher(")
    subscription_position = node_constructor.index("self.create_subscription(")

    assert validation_position < publisher_position
    assert validation_position < subscription_position


def test_script_declares_expected_parameters() -> None:
    script = (PACKAGE_ROOT / "scripts" / "scan_timing_normalizer").read_text(encoding="utf-8")

    for expected in (
        "declare_parameter('input_topic', '/scan_raw')",
        "declare_parameter('output_topic', '/scan')",
        "declare_parameter('zero_time_increment', True)",
    ):
        assert expected in script


def test_cmake_installs_scan_timing_normalizer() -> None:
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "scripts/scan_timing_normalizer" in cmake


def test_ydlidar_launch_routes_driver_scan_through_normalizer() -> None:
    launch = (PACKAGE_ROOT / "launch" / "ydlidar.launch.py").read_text(encoding="utf-8")

    assert "scan_timing_normalizer" in launch
    assert "driver_scan_topic" in launch
    assert "normalized_scan_topic" in launch
    assert "use_scan_timing_normalizer" in launch
    assert "zero_time_increment" in launch
    assert "ParameterValue(zero_time_increment, value_type=bool)" in launch
    assert "('/scan', driver_scan_topic)" in launch
    assert "condition=IfCondition(use_scan_timing_normalizer)" in launch


def test_readme_documents_scan_timing_normalizer_contract() -> None:
    readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    airborne = (
        REPO_ROOT / "docs" / "hardware" / "encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")

    assert "/scan_raw" in readme
    assert "scan_timing_normalizer" in readme
    assert "time_increment" in readme
    assert "60-second drift check" in airborne
    assert "/motor_ready" in airborne
    for token in (
        'CAPTURE_DIR="$(mktemp -d)"',
        "timeout 65 ros2 topic echo /odom",
        "timeout 65 ros2 topic echo /joint_states",
        "timeout 65 ros2 topic echo /motor_ready",
        "timeout 65 ros2 topic echo /rosout",
        "translation drift <=0.01 m",
        "yaw drift <=0.02 rad",
        "max absolute wheel velocity <=0.05 rad/s",
        "1500 × 2π / 4096 = 2.3009711818284617 rad/s",
        "(2.3009711818284617, 1500)",
        "Encoder feedback loss with write path available",
        "Serial disconnect at zero/torque-off",
        "stop/disable packets are attempts, not observed physical stop",
        "physical power cutoff is mandatory on serial disconnect",
    ):
        assert token in airborne, token

    assert airborne.index("Encoder feedback loss with write path available") < airborne.index(
        "Serial disconnect at zero/torque-off"
    )


def test_airborne_capture_has_executable_metric_extraction_formulas() -> None:
    airborne = (
        REPO_ROOT / "docs" / "hardware" / "encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    match = re.search(
        r'python3 - "\$CAPTURE_DIR" <<\'PY\'\n(.*?)\nPY', airborne, flags=re.DOTALL
    )
    assert match is not None
    script = match.group(1)
    compile(script, "<airborne_capture_analysis>", "exec")

    for token in (
        "yaml.safe_load_all",
        "math.hypot(x1 - x0, y1 - y0)",
        "math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))",
        "math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))",
        "max(abs(value)",
        'message.get("data") is True',
        'print(f"translation_drift_m={translation_drift:.6f}")',
        'print(f"yaw_drift_rad={yaw_drift:.6f}")',
        'print(f"max_abs_wheel_velocity_rad_s={max_wheel_velocity:.6f}")',
        'print(f"motor_ready_all_true={motor_ready_all_true}")',
    ):
        assert token in script, token

    capture_block = next(
        block
        for block in re.findall(r"```bash\n(.*?)\n```", airborne, flags=re.DOTALL)
        if 'CAPTURE_DIR="$(mktemp -d)"' in block
    )
    assert 'if wait "$PID"; then' in capture_block
    assert "WAIT_STATUS=0" in capture_block
    assert "WAIT_STATUS=$?" in capture_block
    assert 'test "$WAIT_STATUS" -eq 124' in capture_block


def test_airborne_analyzer_requires_joint_state_span_of_59_seconds() -> None:
    airborne = (
        REPO_ROOT / "docs" / "hardware" / "encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    match = re.search(
        r'python3 - "\$CAPTURE_DIR" <<\'PY\'\n(.*?)\nPY', airborne, flags=re.DOTALL
    )
    assert match is not None
    script = match.group(1)
    assert "stamp_seconds(joint_window[-1]) - stamp_seconds(joint_window[0]) < 59.0" in script

    def stamped(stamp: int, payload: dict) -> dict:
        return {"header": {"stamp": {"sec": stamp, "nanosec": 0}}, **payload}

    odom = [
        stamped(
            stamp,
            {
                "pose": {
                    "pose": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    }
                }
            },
        )
        for stamp in (0, 60)
    ]

    with tempfile.TemporaryDirectory() as temp_dir:
        capture_dir = Path(temp_dir)
        for name, messages in (
            ("odom.txt", odom),
            ("motor_ready.txt", [{"data": True}]),
            ("rosout.txt", []),
        ):
            (capture_dir / name).write_text(
                yaml.safe_dump_all(messages, explicit_end=True), encoding="utf-8"
            )

        def run_with_joint_stamps(stamps: tuple[int, ...]) -> subprocess.CompletedProcess[str]:
            joints = [stamped(stamp, {"velocity": [0.0, 0.0, 0.0]}) for stamp in stamps]
            (capture_dir / "joint_states.txt").write_text(
                yaml.safe_dump_all(joints, explicit_end=True), encoding="utf-8"
            )
            return subprocess.run(
                [sys.executable, "-c", script, str(capture_dir)],
                text=True,
                capture_output=True,
                check=False,
            )

        one_sample = run_with_joint_stamps((0,))
        assert one_sample.returncode != 0
        assert "joint_states does not cover the 60-second window" in one_sample.stderr
        assert run_with_joint_stamps((0, 60)).returncode == 0


if __name__ == "__main__":
    test_normalize_scan_timing_zeroes_time_increment_without_mutating_input()
    test_normalize_scan_timing_can_preserve_time_increment_when_disabled()
    test_topic_pair_rejects_resolved_self_feedback_topics()
    test_topic_pair_rejects_empty_topics_before_resolution()
    test_topic_pair_accepts_distinct_resolved_topics()
    test_node_validates_topics_before_creating_ros_endpoints()
    test_script_declares_expected_parameters()
    test_cmake_installs_scan_timing_normalizer()
    test_ydlidar_launch_routes_driver_scan_through_normalizer()
    test_readme_documents_scan_timing_normalizer_contract()
    test_airborne_capture_has_executable_metric_extraction_formulas()
    test_airborne_analyzer_requires_joint_state_span_of_59_seconds()
