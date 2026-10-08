import ast
import math
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory

from launch import LaunchContext
from launch_contract_helpers import (declared_launch_arguments, forwarded_base_arguments,
                                     launch_argument_defaults)


REPO_ROOT = Path(__file__).resolve().parents[2]


REQUIRED_ROBOT_ARGS = {
    "mode",
    "use_lidar",
    "use_imu",
    "imu_i2c_device",
    "imu_i2c_address",
    "imu_frame_id",
    "lidar_port",
    "lidar_params_file",
    "driver_scan_topic",
    "normalized_scan_topic",
    "use_scan_timing_normalizer",
    "zero_time_increment",
    "motor_backend",
    "enable_motor_write",
    "serial_port",
    "odom_source",
    "max_wheel_speed",
    "speed_tick_scale",
    "speed_tick_limit",
    "encoder_tick_deadband",
    "encoder_feedback_source",
    "encoder_position_ticks_per_revolution",
    "torque_enable",
    "log_encoder_reads",
    "monitor_motor_health",
}

MOTOR_DIAGNOSTIC_DEFAULTS = {
    "odom_source": "encoder",
    "max_wheel_speed": "2.3009711818284617",
    "speed_tick_scale": "651.8986469044033",
    "speed_tick_limit": "1500",
    "encoder_tick_deadband": "50",
    "encoder_feedback_source": "position",
    "encoder_position_ticks_per_revolution": "4096.0",
    "torque_enable": "false",
    "log_encoder_reads": "false",
    "monitor_motor_health": "false",
}


def read_launch(name: str) -> str:
    return Path("launch", name).read_text(encoding="utf-8")


def parse_launch(name: str) -> ast.Module:
    return ast.parse(read_launch(name), filename=f"launch/{name}")


def string_constants(tree: ast.AST) -> set[str]:
    return {node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and isinstance(node.value, str)}


def call_names(tree: ast.AST, name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == name
    ]


def keyword_value(call: ast.Call, name: str) -> ast.AST | None:
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def fenced_bash_blocks(content: str) -> list[str]:
    return re.findall(r"```bash\n(.*?)\n```", content, flags=re.DOTALL)


def fenced_bash_block_containing(content: str, marker: str) -> str:
    matches = [block for block in fenced_bash_blocks(content) if marker in block]
    assert len(matches) == 1, (
        f"expected exactly one fenced bash block containing {marker!r}, "
        f"found {len(matches)}"
    )
    return matches[0]


def staged_hardware_launch_examples(repo_root: Path) -> list[tuple[str, str]]:
    examples = []
    markdown_paths = [repo_root / "README.md"]
    markdown_paths.extend(sorted((repo_root / "docs").glob("**/*.md")))
    for path in markdown_paths:
        if not path.is_file():
            continue
        relative_path = path.relative_to(repo_root).as_posix()
        document = path.read_text(encoding="utf-8")
        examples.extend(
            (relative_path, block)
            for block in fenced_bash_blocks(document)
            if "ros2 launch lekiwi_bringup robot.launch.py" in block
            and "max_wheel_speed:=1.0" in block
        )
    return examples


def launch_numeric_overrides(block: str) -> dict[str, float]:
    return {
        name: float(value)
        for name, value in re.findall(r"\b([a-z_]+):=([0-9]+(?:\.[0-9]+)?)", block)
    }


def motion_case_body(block: str, selector: str) -> str:
    matches = re.findall(
        rf'^\s*case "\${re.escape(selector)}" in\s*$\n(.*?)^\s*esac\s*$',
        block,
        flags=re.DOTALL | re.MULTILINE,
    )
    assert len(matches) == 1, selector
    return matches[0]


def case_non_default_arm_names(case_body: str) -> list[str]:
    return [
        name
        for name in re.findall(r"^\s*([^\s)]+)\)", case_body, flags=re.MULTILINE)
        if name != "*"
    ]


def case_motion_arguments(
    case_body: str,
    expected_assignment: str | None = None,
) -> list[tuple[str, str, float]]:
    assignment_pattern = (
        re.escape(expected_assignment)
        if expected_assignment is not None
        else r"(?:motion_args|COMMAND_ARGS)"
    )
    matches = re.findall(
        rf"^\s*([a-z]+)\)\s+({assignment_pattern})="
        r"\(--(linear-x|linear-y|angular-z)\s+(-?[0-9]+(?:\.[0-9]+)?)\)\s+;;\s*$",
        case_body,
        flags=re.MULTILINE,
    )
    arm_names = case_non_default_arm_names(case_body)
    if [direction for direction, _, _, _ in matches] != arm_names:
        raise ValueError("every non-default case arm must be one strict single-axis assignment")
    if expected_assignment is None and len({assignment for _, assignment, _, _ in matches}) > 1:
        raise ValueError("case arms must use one consistent motion assignment")
    return [
        (direction, axis, float(value))
        for direction, _, axis, value in matches
    ]


def test_robot_launch_exposes_base_mode_arguments() -> None:
    tree = parse_launch("robot.launch.py")

    assert REQUIRED_ROBOT_ARGS <= declared_launch_arguments("robot.launch.py").keys()


def test_robot_launch_rejects_non_base_modes() -> None:
    launch = read_launch("robot.launch.py")

    assert "mode_value != 'base'" in launch
    assert "Unsupported LeKiwi mode" in launch


def test_robot_launch_forwards_encoder_feedback_overrides_to_base_launch() -> None:
    context = LaunchContext()
    overrides = {"encoder_tick_deadband": "25", "encoder_feedback_source": "speed",
                 "monitor_motor_health": "true"}
    context.launch_configurations.update(overrides)
    forwarded = forwarded_base_arguments()
    for name, value in overrides.items():
        assert ''.join(item.perform(context) for item in forwarded[name]) == value


def test_robot_launch_forwards_every_base_argument_override() -> None:
    declared = declared_launch_arguments("base.launch.py")
    forwarded = forwarded_base_arguments()
    assert declared.keys() == forwarded.keys()
    context = LaunchContext()
    context.launch_configurations.update({name: "override_" + name for name in declared})
    for name, substitutions in forwarded.items():
        assert ''.join(item.perform(context) for item in substitutions) == "override_" + name


def test_base_launch_includes_ydlidar_launch_under_use_lidar_condition() -> None:
    tree = parse_launch("base.launch.py")
    constants = string_constants(tree)
    include_calls = call_names(tree, "IncludeLaunchDescription")

    assert "lekiwi_sensors" in constants
    assert "launch" in constants
    assert "ydlidar.launch.py" in constants

    lidar_includes = [
        call
        for call in include_calls
        if "ydlidar.launch.py" in {
            node.value
            for node in ast.walk(call)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
    ]
    assert len(lidar_includes) == 1

    condition = keyword_value(lidar_includes[0], "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == "IfCondition"
    assert condition.args
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "use_lidar"

    launch_arguments = keyword_value(lidar_includes[0], "launch_arguments")
    assert isinstance(launch_arguments, ast.Call)
    assert isinstance(launch_arguments.func, ast.Attribute)
    assert launch_arguments.func.attr == "items"
    assert isinstance(launch_arguments.func.value, ast.Dict)

    forwarded_arguments = {}
    for key, value in zip(launch_arguments.func.value.keys, launch_arguments.func.value.values, strict=True):
        assert isinstance(key, ast.Constant)
        assert isinstance(key.value, str)
        assert isinstance(value, ast.Name)
        forwarded_arguments[key.value] = value.id

    assert forwarded_arguments["params_file"] == "lidar_params_file"
    assert forwarded_arguments["lidar_port"] == "lidar_port"
    assert forwarded_arguments["driver_scan_topic"] == "driver_scan_topic"
    assert forwarded_arguments["normalized_scan_topic"] == "normalized_scan_topic"
    assert forwarded_arguments["use_scan_timing_normalizer"] == "use_scan_timing_normalizer"
    assert forwarded_arguments["zero_time_increment"] == "zero_time_increment"


def test_base_launch_includes_bmi160_by_default() -> None:
    tree = parse_launch("base.launch.py")
    include_calls = call_names(tree, "IncludeLaunchDescription")
    imu_includes = [
        call
        for call in include_calls
        if "bmi160.launch.py" in {
            node.value
            for node in ast.walk(call)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
    ]
    assert len(imu_includes) == 1

    condition = keyword_value(imu_includes[0], "condition")
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == "IfCondition"
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == "use_imu"

    defaults = launch_argument_defaults("base.launch.py")
    assert defaults["use_imu"] == "true"
    assert defaults["imu_i2c_device"] == "/dev/i2c-1"
    assert defaults["imu_i2c_address"] == "104"
    assert defaults["imu_frame_id"] == "imu_link"


def test_base_launch_exposes_lidar_normalizer_arguments() -> None:
    tree = parse_launch("base.launch.py")

    assert {
        "driver_scan_topic",
        "normalized_scan_topic",
        "use_scan_timing_normalizer",
        "zero_time_increment",
    } <= declared_launch_arguments("base.launch.py").keys()


def test_base_launch_passes_motor_diagnostic_overrides_to_lekiwi_node() -> None:
    tree = parse_launch("base.launch.py")
    constants = string_constants(tree)

    assert "odom_source" in constants
    assert "max_wheel_speed" in constants
    assert "speed_tick_scale" in constants
    assert "speed_tick_limit" in constants
    assert "encoder_tick_deadband" in constants
    assert "encoder_feedback_source" in constants
    assert "encoder_position_ticks_per_revolution" in constants
    assert "torque_enable" in constants
    assert "log_encoder_reads" in constants
    assert "monitor_motor_health" in constants

    node_calls = call_names(tree, "Node")
    lekiwi_nodes = [
        call
        for call in node_calls
        if "lekiwi_node" in {
            arg.value for arg in ast.walk(call) if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
        }
    ]
    assert len(lekiwi_nodes) == 1

    parameters = keyword_value(lekiwi_nodes[0], "parameters")
    assert isinstance(parameters, ast.List)
    assert len(parameters.elts) == 3
    assert isinstance(parameters.elts[1], ast.Name)
    assert parameters.elts[1].id == "geometry_params_file"
    overrides = parameters.elts[2]
    assert isinstance(overrides, ast.Dict)

    override_names = {}
    for key, value in zip(overrides.keys, overrides.values, strict=True):
        assert isinstance(key, ast.Constant)
        assert isinstance(key.value, str)
        assert isinstance(value, (ast.Name, ast.Call))
        override_names[key.value] = value

    assert "odom_source" in override_names
    assert isinstance(override_names["odom_source"], ast.Name)
    assert override_names["odom_source"].id == "odom_source"
    assert "max_wheel_speed" in override_names
    assert isinstance(override_names["max_wheel_speed"], ast.Call)
    assert "speed_tick_scale" in override_names
    assert isinstance(override_names["speed_tick_scale"], ast.Call)
    assert "speed_tick_limit" in override_names
    assert isinstance(override_names["speed_tick_limit"], ast.Call)
    assert "encoder_tick_deadband" in override_names
    assert isinstance(override_names["encoder_tick_deadband"], ast.Call)
    assert "encoder_feedback_source" in override_names
    assert isinstance(override_names["encoder_feedback_source"], ast.Name)
    assert "encoder_position_ticks_per_revolution" in override_names
    assert isinstance(override_names["encoder_position_ticks_per_revolution"], ast.Call)
    assert "torque_enable" in override_names
    assert isinstance(override_names["torque_enable"], ast.Call)
    assert "log_encoder_reads" in override_names
    assert isinstance(override_names["log_encoder_reads"], ast.Call)
    assert "monitor_motor_health" in override_names
    assert isinstance(override_names["monitor_motor_health"], ast.Call)


def test_motor_diagnostic_launch_defaults_match_calibrated_base_defaults() -> None:
    for launch_file in ("base.launch.py", "robot.launch.py"):
        defaults = launch_argument_defaults(launch_file)

        for name, value in MOTOR_DIAGNOSTIC_DEFAULTS.items():
            assert defaults[name] == value


def test_readme_documents_staged_hardware_acceptance() -> None:
    assert "docs/hardware/base_operation.md" in (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    documents = {
        relative_path: (REPO_ROOT / relative_path).read_text(encoding="utf-8")
        for relative_path in (
            "docs/hardware/base_operation.md",
            "docs/hardware/encoder_odom_airborne_check.md",
            "docs/hardware/encoder_odom_floor_calibration.md",
        )
    }
    readme = documents["docs/hardware/base_operation.md"]
    airborne = documents["docs/hardware/encoder_odom_airborne_check.md"]
    floor = documents["docs/hardware/encoder_odom_floor_calibration.md"]

    for token in (
        "/motor_ready",
        "encoder_odom_scale",
        "(1.0, 600)",
        "(2.0, 1200)",
        "(3.5, 2100)",
        "(5.4, 3240)",
        "60-second drift check",
        "/cmd_vel",
        "/odom",
        "/scan",
    ):
        assert token in readme, token

    assert "수동 wheel-direction non-zero 명령" in readme
    assert "지도 저장과 정지 상태의 localization 입력·TF를 확인했다." in readme
    assert "Nav2 목표 도달은 아직 확인하지 못했다." in readme

    assert "Encoder feedback loss with write path available" in airborne
    assert "Serial disconnect at zero/torque-off" in airborne
    assert "translation_error_pct" in floor
    assert "yaw_error_pct" in floor

    ordered_gates = (
        "1. **Offline complete**",
        "2. **Airborne supported robot**",
        "3. **Teleop / odom / scan**",
    )
    positions = [readme.index(heading) for heading in ordered_gates]
    assert positions == sorted(positions)


def test_staged_hardware_launch_examples_fit_body_velocity_envelope() -> None:
    expected_example_counts = {
        "docs/hardware/base_operation.md": 1,
        "docs/hardware/encoder_odom_floor_calibration.md": 2,
    }
    staged_launch_examples = staged_hardware_launch_examples(REPO_ROOT)
    assert len(staged_launch_examples) >= sum(expected_example_counts.values())
    for relative_path, expected_count in expected_example_counts.items():
        assert sum(
            example_path == relative_path
            for example_path, _ in staged_launch_examples
        ) == expected_count
    required_limits = {
        "max_linear_x",
        "max_linear_y",
        "max_linear_speed",
        "max_angular_z",
        "max_wheel_speed",
    }

    for relative_path, example in staged_launch_examples:
        overrides = launch_numeric_overrides(example)
        assert required_limits <= overrides.keys(), relative_path
        assert overrides["max_linear_x"] == overrides["max_linear_speed"] == 0.03
        assert overrides["max_linear_y"] == overrides["max_linear_speed"] == 0.03
        assert overrides["max_angular_z"] == 0.1
        required_wheel_speed = (
            overrides["max_linear_speed"] + 0.115 * overrides["max_angular_z"]
        ) / 0.0508
        assert required_wheel_speed <= overrides["max_wheel_speed"], relative_path

    airborne = (
        REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    relaunch_guidance = next(
        line for line in airborne.splitlines() if "Relaunch hardware bringup" in line
    )
    for override in (
        "max_linear_x:=0.03",
        "max_linear_y:=0.03",
        "max_linear_speed:=0.03",
        "max_angular_z:=0.1",
    ):
        assert override in relaunch_guidance

    trial_matrix = fenced_bash_block_containing(airborne, "# AIRBORNE_TRIAL_MATRIX_START")
    assert "motion_args=(--linear-x 0.03)" in trial_matrix
    assert "motion_args=(--linear-y 0.03)" in trial_matrix
    assert "motion_args=(--angular-z 0.1)" in trial_matrix


def test_staged_hardware_launch_collection_discovers_nested_docs() -> None:
    with TemporaryDirectory() as temporary_directory:
        repo_root = Path(temporary_directory)
        for relative_path in (
            "README.md",
            "docs/hardware/encoder_odom_floor_calibration.md",
        ):
            path = repo_root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("", encoding="utf-8")

        nested_doc = repo_root / "docs/new/hardware/staged.md"
        nested_doc.parent.mkdir(parents=True)
        nested_doc.write_text(
            """\
```bash
ros2 launch lekiwi_bringup robot.launch.py \\
  max_wheel_speed:=1.0
```
""",
            encoding="utf-8",
        )

        examples = staged_hardware_launch_examples(repo_root)
        assert [relative_path for relative_path, _ in examples] == [
            "docs/new/hardware/staged.md"
        ]


def test_physical_trial_matrix_commands_fit_staged_body_limits() -> None:
    airborne = (
        REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    floor = (
        REPO_ROOT / "docs/hardware/encoder_odom_floor_calibration.md"
    ).read_text(encoding="utf-8")
    airborne_relaunch = next(
        line for line in airborne.splitlines() if "Relaunch hardware bringup" in line
    )
    floor_launch = next(
        block
        for block in fenced_bash_blocks(floor)
        if "ros2 launch lekiwi_bringup robot.launch.py" in block
        and "max_wheel_speed:=1.0" in block
    )
    matrix_contracts = {
        "airborne": (
            motion_case_body(
                fenced_bash_block_containing(airborne, "# AIRBORNE_TRIAL_MATRIX_START"),
                "direction",
            ),
            launch_numeric_overrides(airborne_relaunch),
            "motion_args",
            {
                "forward": ("linear-x", 1),
                "reverse": ("linear-x", -1),
                "left": ("linear-y", 1),
                "right": ("linear-y", -1),
                "ccw": ("angular-z", 1),
                "cw": ("angular-z", -1),
            },
        ),
        "floor": (
            motion_case_body(
                fenced_bash_block_containing(floor, "# FLOOR_TRIAL_MATRIX_START"),
                "DIRECTION",
            ),
            launch_numeric_overrides(floor_launch),
            "COMMAND_ARGS",
            {
                "forward": ("linear-x", 1),
                "backward": ("linear-x", -1),
                "left": ("linear-y", 1),
                "right": ("linear-y", -1),
                "ccw": ("angular-z", 1),
                "cw": ("angular-z", -1),
            },
        ),
    }
    axis_limits = {
        "linear-x": "max_linear_x",
        "linear-y": "max_linear_y",
        "angular-z": "max_angular_z",
    }

    for name, (case_body, limits, assignment, expected_directions) in matrix_contracts.items():
        arm_names = case_non_default_arm_names(case_body)
        assert len(arm_names) == 6, name
        assert len(set(arm_names)) == 6, name
        assert set(arm_names) == set(expected_directions), name

        commands = case_motion_arguments(case_body, assignment)
        assert len(commands) == len(arm_names), name
        for axis in axis_limits:
            axis_values = [value for _, command_axis, value in commands if command_axis == axis]
            assert len(axis_values) == 2, (name, axis)
            assert any(value > 0.0 for value in axis_values), (name, axis)
            assert any(value < 0.0 for value in axis_values), (name, axis)

        actual_directions = {}
        for direction, axis, value in commands:
            assert value != 0.0, (name, direction)
            actual_directions[direction] = (axis, 1 if value > 0.0 else -1)
            assert abs(value) <= limits[axis_limits[axis]], (name, direction)
            if axis.startswith("linear-"):
                assert abs(value) <= limits["max_linear_speed"], (name, direction)

        assert actual_directions == expected_directions, name


def test_case_motion_parser_rejects_hidden_or_non_strict_arms() -> None:
    valid_arms = """\
forward) motion_args=(--linear-x 0.03) ;;
reverse) motion_args=(--linear-x -0.03) ;;
left) motion_args=(--linear-y 0.03) ;;
right) motion_args=(--linear-y -0.03) ;;
ccw) motion_args=(--angular-z 0.1) ;;
cw) motion_args=(--angular-z -0.1) ;;
*) return 2 ;;
"""
    adversarial_mutations = {
        "trailing-comment duplicate": (
            valid_arms + "forward) motion_args=(--linear-x 9.0) ;; # hidden duplicate\n"
        ),
        "multi-axis arm": valid_arms.replace(
            "forward) motion_args=(--linear-x 0.03) ;;",
            "forward) motion_args=(--linear-x 0.03 --linear-y 0.03) ;;",
        ),
        "alternate assignment extra": (
            valid_arms + "forward) motion_args+=(--linear-x 9.0) ;;\n"
        ),
    }

    for name, mutation in adversarial_mutations.items():
        rejected = False
        try:
            case_motion_arguments(mutation)
        except ValueError:
            rejected = True
        assert rejected, name


def test_motion_procedures_put_fresh_confirmation_before_first_nonzero_command() -> None:
    marker = "STOP: fresh user confirmation required immediately before first non-zero command"
    command = "ros2 run lekiwi_teleop lekiwi_safe_cmd_vel \\\n"
    paths = ("docs/hardware/encoder_odom_floor_calibration.md",)

    readme = (REPO_ROOT / "docs/hardware/base_operation.md").read_text(encoding="utf-8")
    short_test = readme[readme.index("### 짧은 실물 구동 테스트") : readme.index("### Base Teleop")]
    assert "authoritative guarded Airborne blocks" in short_test
    assert "ros2 run lekiwi_teleop lekiwi_safe_cmd_vel" not in "\n".join(
        fenced_bash_blocks(short_test)
    )

    for relative_path in paths:
        document = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
        marker_index = document.index(marker)
        command_index = document.index(command)
        assert marker_index < command_index, relative_path
        assert "`--yes-i-confirm-motion` is a CLI acknowledgement, not authorization." in document

        floor_block = fenced_bash_block_containing(document, "# FLOOR_TRIAL_MATRIX_START")
        ordered_positions = (
            floor_block.index("EXPECTED_CONFIRMATION="),
            floor_block.index("read -r -p"),
            floor_block.index('test "$FRESH_CONFIRMATION" = "$EXPECTED_CONFIRMATION"'),
            floor_block.index(command),
        )
        assert list(ordered_positions) == sorted(ordered_positions)

    airborne = (
        REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    assert airborne.index(marker) < airborne.index("# AIRBORNE_TRIAL_MATRIX_START")
    airborne_trial = fenced_bash_block_containing(airborne, "# AIRBORNE_TRIAL_MATRIX_START")
    assert airborne_trial.index("Fresh motion confirmation") < airborne_trial.index(command)
    assert "`--yes-i-confirm-motion` is a CLI acknowledgement, not authorization." in airborne


def test_readme_scopes_keyboard_teleop_to_mock_write_disabled_demo() -> None:
    readme = (REPO_ROOT / "docs/hardware/base_operation.md").read_text(encoding="utf-8")
    teleop_section = readme[readme.index("### Base Teleop") : readme.index("## YDLIDAR 설치와 빌드")]

    for token in (
        "mock/write-disabled demonstration only",
        "motor_backend:=mock",
        "enable_motor_write:=false",
        "must not be used in physical hardware acceptance",
        "separate guarded bounded wrapper",
    ):
        assert token in teleop_section, token

    assert teleop_section.index("motor_backend:=mock") < teleop_section.index(
        "ros2 run lekiwi_teleop lekiwi_base_teleop"
    )


def test_physical_acceptance_has_no_unguarded_cmd_vel_publishers() -> None:
    documents = {
        "docs/hardware/base_operation.md": (REPO_ROOT / "docs/hardware/base_operation.md").read_text(encoding="utf-8"),
        "airborne": (REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md").read_text(
            encoding="utf-8"
        ),
        "floor": (REPO_ROOT / "docs/hardware/encoder_odom_floor_calibration.md").read_text(
            encoding="utf-8"
        ),
    }
    physical_sections = {
        "docs/hardware/base_operation.md": documents["docs/hardware/base_operation.md"][
            documents["docs/hardware/base_operation.md"].index("### 짧은 실물 구동 테스트") :
            documents["docs/hardware/base_operation.md"].index("### Base Teleop")
        ],
        "airborne": documents["airborne"],
        "floor": documents["floor"],
    }

    for name, section in physical_sections.items():
        bash = "\n".join(fenced_bash_blocks(section))
        assert "lekiwi_base_teleop" not in bash, name
        assert not re.search(r"ros2\s+topic\s+pub[\s\S]{0,200}/cmd_vel", bash), name
        for match in re.finditer(r"ros2 run lekiwi_teleop (\S+)", bash):
            assert match.group(1) in {"lekiwi_safe_cmd_vel", "lekiwi_odom_calibration"}, (
                name,
                match.group(1),
            )

    motion_marker = "STOP: fresh user confirmation required immediately before first non-zero command"
    for name in ("airborne", "floor"):
        section = physical_sections[name]
        for match in re.finditer(r"ros2 run lekiwi_teleop lekiwi_safe_cmd_vel", section):
            assert section.index(motion_marker) < match.start(), name


def test_airborne_power_on_is_guarded_and_readiness_is_observed_separately() -> None:
    airborne = (
        REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    block = fenced_bash_block_containing(airborne, "# AIRBORNE_POWER_ON_START")

    def motor_power_succeeded(response: str) -> bool:
        matches = re.findall(
            r"^motor_power_succeeded\(\) \{\n.*?^\}",
            block,
            flags=re.DOTALL | re.MULTILINE,
        )
        assert len(matches) == 1, "expected one motor_power_succeeded shell function"
        result = subprocess.run(
            ["bash", "-c", f"{matches[0]}\nmotor_power_succeeded"],
            input=response,
            text=True,
            capture_output=True,
            check=False,
        )
        return result.returncode == 0

    assert subprocess.run(
        ["bash", "-n"], input=block, text=True, capture_output=True, check=False
    ).returncode == 0
    power_stop = "STOP: fresh user confirmation required immediately before Airborne motor power-on"
    assert airborne.index(power_stop) < airborne.index("# AIRBORNE_POWER_ON_START")
    assert block.index('timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"') < block.index(
        'timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local'
    )
    assert motor_power_succeeded("success=True\n")
    assert motor_power_succeeded("success: true\n")
    assert not motor_power_succeeded("success=False\n")
    assert not motor_power_succeeded("success: false\n")
    assert not motor_power_succeeded("success=true\n")
    assert not motor_power_succeeded("success: True\n")
    assert not motor_power_succeeded("success=False success=True\n")
    assert not motor_power_succeeded("success: false\nsuccess: true\n")
    assert motor_power_succeeded(
        "response:\n"
        "std_srvs.srv.SetBool_Response(success=True, message='motor power enabled')\n"
    )
    assert 'motor_power_succeeded <<<"$POWER_RESPONSE"' in block
    assert 'grep -q "success: true" <<<"$POWER_RESPONSE"' not in block
    drift_disable_block = fenced_bash_block_containing(
        airborne, "DRIFT_DISABLE_RESPONSE="
    )
    assert 'motor_power_succeeded <<<"$DRIFT_DISABLE_RESPONSE"' in drift_disable_block
    assert (
        'grep -q "success: true" <<<"$DRIFT_DISABLE_RESPONSE"'
        not in drift_disable_block
    )
    assert "grep -Eq '^data: true$'" in block
    assert 'timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"' in block
    assert "exit 1" in block
    assert "motor power service success does not imply /motor_ready true" in airborne


def test_floor_trial_arms_only_inside_guarded_command() -> None:
    power_on = 'ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"'
    readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    operation = (REPO_ROOT / "docs/hardware/base_operation.md").read_text(encoding="utf-8")
    floor = (REPO_ROOT / "docs/hardware/encoder_odom_floor_calibration.md").read_text(
        encoding="utf-8"
    )

    assert power_on not in "\n".join(fenced_bash_blocks(readme))
    assert power_on not in "\n".join(fenced_bash_blocks(operation))
    floor_trial = fenced_bash_block_containing(floor, "# FLOOR_TRIAL_MATRIX_START")
    assert floor_trial.count(power_on) == 0
    assert floor_trial.count("--arm-motors") == 1
    assert power_on not in "\n".join(
        block
        for block in fenced_bash_blocks(floor)
        if "# FLOOR_TRIAL_MATRIX_START" not in block
    )


def test_airborne_trial_matrix_guards_every_stage_and_direction() -> None:
    airborne = (
        REPO_ROOT / "docs/hardware/encoder_odom_airborne_check.md"
    ).read_text(encoding="utf-8")
    block = fenced_bash_block_containing(airborne, "# AIRBORNE_TRIAL_MATRIX_START")
    syntax = subprocess.run(
        ["bash", "-n"], input=block, text=True, capture_output=True, check=False
    )
    assert syntax.returncode == 0, syntax.stderr

    stages_match = re.search(r'^STAGES=\(([^\n]+)\)$', block, flags=re.MULTILINE)
    directions_match = re.search(r'^DIRECTIONS=\(([^\n]+)\)$', block, flags=re.MULTILINE)
    assert stages_match is not None
    assert directions_match is not None
    stages = re.findall(r'"([^"]+)"', stages_match.group(1))
    directions = re.findall(r'"([^"]+)"', directions_match.group(1))
    assert stages == ["1.0:652", "2.0:1304", "2.3009711818284617:1500"]
    assert directions == ["forward", "reverse", "left", "right", "ccw", "cw"]
    assert len({(stage, direction) for stage in stages for direction in directions}) == 18

    trial_start = block.index("# RUN_AIRBORNE_TRIAL_BEGIN")
    trial_end = block.index("# RUN_AIRBORNE_TRIAL_END")
    trial = block[trial_start:trial_end]
    ordered_tokens = (
        "Fresh motion confirmation",
        "trap best_effort_disable_airborne_trial EXIT",
        "trap interrupt_airborne_trial INT TERM",
        "timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local",
        "grep -Eq '^data: false$'",
        "timeout --signal=INT --kill-after=2 20 ros2 run lekiwi_teleop lekiwi_safe_cmd_vel",
        "--arm-motors",
        'timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"',
        "grep -q \"success: true\"",
        "timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local",
        "grep -Eq '^data: false$'",
        "Physical stop confirmation",
        "trap - EXIT INT TERM",
    )
    cursor = 0
    for token in ordered_tokens:
        cursor = trial.index(token, cursor) + len(token)
    assert trial.count("lekiwi_safe_cmd_vel") == 1
    assert '--yes-i-confirm-motion' in trial

    assert "for STAGE in \"${STAGES[@]}\"; do" in block
    assert "for DIRECTION in \"${DIRECTIONS[@]}\"; do" in block
    assert 'run_airborne_trial "$STAGE_MAX" "$STAGE_TICK" "$DIRECTION"' in block
    assert block.index("ros2 param get /lekiwi_node max_wheel_speed") < block.index(
        "for DIRECTION in \"${DIRECTIONS[@]}\"; do"
    )
    assert block.index("ros2 param get /lekiwi_node speed_tick_limit") < block.index(
        "for DIRECTION in \"${DIRECTIONS[@]}\"; do"
    )


def test_floor_launch_uses_current_first_airborne_wheel_limit() -> None:
    floor = (REPO_ROOT / "docs/hardware/encoder_odom_floor_calibration.md").read_text(
        encoding="utf-8"
    )
    assert all(stage in floor for stage in (
        "(1.0, 652)", "(2.0, 1304)", "(2.3009711818284617, 1500)"
    ))
    assert "(1.0, 600)" not in floor
    launches = [
        block for block in fenced_bash_blocks(floor)
        if "ros2 launch lekiwi_bringup robot.launch.py" in block
    ]
    assert len(launches) == 2
    for launch in launches:
        overrides = launch_numeric_overrides(launch)
        assert overrides["max_wheel_speed"] == 1.0
        assert overrides["speed_tick_limit"] == 652
        assert "use_imu:=false" in launch

        def yaml_float(relative_path: str, key: str) -> float:
            content = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
            match = re.search(rf"^\s*{re.escape(key)}:\s*([-+0-9.eE]+)\s*$", content, re.MULTILINE)
            assert match is not None
            return float(match.group(1))

        base = "lekiwi_bringup/param/base.yaml"
        frame = "lekiwi_description/config/base_frame.yaml"
        frame_offset = math.hypot(
            yaml_float(frame, "kinematics_frame_x"),
            yaml_float(frame, "kinematics_frame_y"),
        )
        wheel_bound = (
            overrides["max_linear_speed"] +
            (yaml_float(base, "base_radius") + frame_offset) * overrides["max_angular_z"]
        ) / yaml_float(base, "wheel_radius")
        assert wheel_bound < overrides["max_wheel_speed"]


def test_floor_trial_wrapper_guards_every_calibration_and_validation_trial() -> None:
    floor = (REPO_ROOT / "docs/hardware/encoder_odom_floor_calibration.md").read_text(
        encoding="utf-8"
    )
    block = fenced_bash_block_containing(floor, "# FLOOR_TRIAL_MATRIX_START")
    syntax = subprocess.run(
        ["bash", "-n"], input=block, text=True, capture_output=True, check=False
    )
    assert syntax.returncode == 0, syntax.stderr

    trial = block[
        block.index("# RUN_FLOOR_TRIAL_BEGIN") : block.index("# RUN_FLOOR_TRIAL_END")
    ]
    pre_ready = "timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local"
    assert trial.index(pre_ready) < trial.index("EXPECTED_CONFIRMATION=")
    assert trial.index("grep -Eq '^data: false$'") < trial.index("EXPECTED_CONFIRMATION=")
    ordered_tokens = (
        "EXPECTED_CONFIRMATION=",
        "read -r -p",
        'test "$FRESH_CONFIRMATION" = "$EXPECTED_CONFIRMATION"',
        "trap best_effort_disable_floor_trial EXIT",
        "trap interrupt_floor_trial INT TERM",
        'timeout 5 ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"',
        'grep -q "success: true"',
        "timeout --signal=INT --kill-after=2 30 ros2 run lekiwi_teleop lekiwi_safe_cmd_vel",
        "--arm-motors",
        'timeout 5 ros2 service call /motor_power std_srvs/srv/SetBool "{data: false}"',
        'grep -q "success: true"',
        "timeout 5 ros2 topic echo /motor_ready --once --qos-reliability reliable --qos-durability transient_local",
        "grep -Eq '^data: false$'",
        "Physical stop confirmation",
        'test "$PHYSICAL_STOP_CONFIRMATION" = "STOPPED"',
        "timeout 5 ros2 topic echo /odom --once",
        "trap - EXIT INT TERM",
    )
    cursor = 0
    for token in ordered_tokens:
        cursor = trial.index(token, cursor) + len(token)
    assert trial.count("lekiwi_safe_cmd_vel") == 1
    assert '--yes-i-confirm-motion' in trial
    assert 'TRIALS_PER_DIRECTION=3' in block
    assert 'SAFETY_DIRECTIONS=("forward")' in block
    assert 'safety) DIRECTIONS=("${SAFETY_DIRECTIONS[@]}"); TRIALS_PER_DIRECTION=1' in block
    assert 'if test "$PHASE" = safety; then MOTION_DURATION=1.0; else MOTION_DURATION=5.0; fi' in trial
    assert '--max-duration "$MOTION_DURATION"' in trial
    assert 'CALIBRATION_DIRECTIONS=("forward")' in block
    assert 'VALIDATION_DIRECTIONS=("forward" "backward" "left" "right" "ccw" "cw")' in block
    assert 'for TRIAL_NUMBER in $(seq 1 "$TRIALS_PER_DIRECTION"); do' in block
    assert 'run_floor_trial "$FLOOR_PHASE" "$DIRECTION" "$TRIAL_NUMBER"' in block


def test_combined_test_launch_keeps_motor_writes_disabled() -> None:
    tree = ast.parse((REPO_ROOT / "lekiwi_bringup/tools/diagnostics/topic_test.launch.py").read_text())
    motor_arguments = []
    for call in call_names(tree, "IncludeLaunchDescription"):
        arguments = keyword_value(call, "launch_arguments")
        if not isinstance(arguments, ast.Call) or not isinstance(arguments.func, ast.Attribute):
            continue
        mapping = arguments.func.value
        if not isinstance(mapping, ast.Dict):
            continue
        values = {key.value: value.value for key, value in zip(mapping.keys, mapping.values)
                  if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)}
        if "motor_backend" in values:
            motor_arguments.append(values)
    assert len(motor_arguments) == 1
    assert {key: motor_arguments[0][key] for key in (
        "motor_backend", "enable_motor_write", "torque_enable", "odom_source")} == {
        "motor_backend": "mock", "enable_motor_write": "false",
        "torque_enable": "false", "odom_source": "command"}


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-o", f"pythonpath={REPO_ROOT / 'lekiwi_bringup'}"]))
