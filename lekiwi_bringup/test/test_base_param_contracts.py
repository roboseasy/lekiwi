import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]


def load_node_params(name: str) -> dict:
    data = yaml.safe_load(Path("param", name).read_text(encoding="utf-8"))
    return data["lekiwi_node"]["ros__parameters"]


def test_base_params_keep_safe_defaults() -> None:
    params = load_node_params("base.yaml")

    assert params["wheel_radius"] == 0.05
    assert params["base_radius"] == 0.13647
    assert params["motor_backend"] == "mock"
    assert params["enable_motor_write"] is False
    assert params["tf_time_offset"] == 0.0
    assert params["odom_source"] == "encoder"
    assert params["encoder_read_period"] == 0.1
    assert params["encoder_sample_timeout"] == 0.5
    assert params["max_linear_x"] == 0.11
    assert params["max_linear_y"] == 0.1
    assert params["max_linear_speed"] == 0.11
    assert params["max_angular_z"] == 0.75
    assert params["max_wheel_speed"] == 2.3009711818284617
    assert params["speed_tick_scale"] == 651.8986469044033
    assert params["speed_tick_limit"] == 1500
    assert params["encoder_tick_deadband"] == 50
    assert params["encoder_feedback_source"] == "position"
    assert params["encoder_position_ticks_per_revolution"] == 4096.0
    assert params["encoder_odom_scale"] == 1.0
    assert params["encoder_position_tick_deadband"] == 0
    assert params["log_encoder_reads"] is False
    assert params["wheel_ids"] == [8, 9, 7]
    assert params["torque_enable"] is False


def test_base_profile_example_keeps_explicit_hardware_defaults() -> None:
    params = load_node_params("base_profile.example.yaml")

    assert params["wheel_radius"] == 0.05
    assert params["base_radius"] == 0.13647
    assert params["motor_backend"] == "feetech"
    assert params["enable_motor_write"] is True
    assert params["tf_time_offset"] == 0.0
    assert params["odom_source"] == "encoder"
    assert params["encoder_read_period"] == 0.1
    assert params["encoder_sample_timeout"] == 0.5
    assert params["max_linear_x"] == 0.11
    assert params["max_linear_y"] == 0.1
    assert params["max_linear_speed"] == 0.11
    assert params["max_angular_z"] == 0.75
    assert params["max_wheel_speed"] == 2.3009711818284617
    assert params["speed_tick_scale"] == 651.8986469044033
    assert params["speed_tick_limit"] == 1500
    assert params["encoder_tick_deadband"] == 50
    assert params["encoder_feedback_source"] == "position"
    assert params["encoder_position_ticks_per_revolution"] == 4096.0
    assert params["encoder_odom_scale"] == 1.0
    assert params["encoder_position_tick_deadband"] == 0
    assert params["log_encoder_reads"] is False
    assert params["wheel_ids"] == [8, 9, 7]
    assert params["torque_enable"] is False


def test_guarded_teleop_stays_within_base_and_wheel_capacity() -> None:
    params = load_node_params("base.yaml")
    teleop = (
        REPO_ROOT / "lekiwi_teleop" / "lekiwi_teleop" / "guarded_base_teleop.py"
    ).read_text(encoding="utf-8")

    assert '"w": Motion(linear_x=0.10)' in teleop
    assert '"a": Motion(linear_y=0.10)' in teleop
    assert '"q": Motion(angular_z=0.30)' in teleop
    assert 0.10 <= params["max_linear_x"]
    assert 0.10 <= params["max_linear_y"]
    assert 0.30 <= params["max_angular_z"]
    assert params["max_wheel_speed"] <= 52.0 * 2.0 * math.pi / 60.0
    assert params["speed_tick_limit"] == round(
        params["max_wheel_speed"] * params["speed_tick_scale"]
    )
    assert params["max_linear_speed"] / params["wheel_radius"] <= params["max_wheel_speed"]
    assert (
        params["base_radius"] * params["max_angular_z"] / params["wheel_radius"]
        <= params["max_wheel_speed"]
    )


def test_motor_ids_match_observed_physical_positions_in_current_model() -> None:
    params = load_node_params("base.yaml")
    node_params = yaml.safe_load(
        (REPO_ROOT / "lekiwi_node/param/base_controller.yaml").read_text()
    )["lekiwi_node"]["ros__parameters"]
    assert node_params["wheel_ids"] == params["wheel_ids"]
    model = ET.parse(REPO_ROOT / "lekiwi_description/urdf/base_and_arm.urdf.xacro")
    mounts = {
        element.get("name"): (float(element.get("x")), float(element.get("y")))
        for element in model.getroot()
        if element.tag == "{http://www.ros.org/wiki/xacro}lekiwi_wheel"
    }
    by_id = dict(zip(params["wheel_ids"], [mounts[n] for n in ["left", "back", "right"]]))
    assert by_id[8][0] < 0 and abs(by_id[8][1]) < 0.001  # observed rear
    assert by_id[7][1] > 0.1  # observed left
    assert by_id[9][1] < -0.1  # remaining right


if __name__ == "__main__":
    test_base_params_keep_safe_defaults()
    test_base_profile_example_keeps_explicit_hardware_defaults()
    test_guarded_teleop_stays_within_base_and_wheel_capacity()
    test_motor_ids_match_observed_physical_positions_in_current_model()
