from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_ydlidar_params() -> dict:
    params_path = PACKAGE_ROOT / "param" / "ydlidar.yaml"
    params = yaml.safe_load(params_path.read_text(encoding="utf-8"))
    return params["ydlidar_ros2_driver_node"]["ros__parameters"]


def test_ydlidar_angle_mask_is_disabled_for_min_range_experiment() -> None:
    params = load_ydlidar_params()

    assert params["ignore_array"] == ""


def test_package_declares_yaml_test_dependency() -> None:
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")

    assert "<test_depend>python3-yaml</test_depend>" in package_xml


if __name__ == "__main__":
    test_ydlidar_angle_mask_is_disabled_for_min_range_experiment()
    test_package_declares_yaml_test_dependency()
