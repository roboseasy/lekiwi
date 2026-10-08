import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "vendor" / "bosch_bmi160"

EXPECTED_HASHES = {
    "bmi160.c": "df3ff5e019db827287be52619d30ee647bcabee65f9a0bd3ddacef2ea908b591",
    "bmi160.h": "c28edf703c23f4c25ee4a8a08ff06e4b705c106638ee62e2e9116bf01639628c",
    "bmi160_defs.h": "abcb11556b25f374f4f8da05ee5f7f80d758df93abc32129ecb63131dcf78960",
    "LICENSE": "b2f8040032341e9aed1224d0c72a3fbf57c24977b1462aafb4091dc2e691527e",
}


def test_bosch_vendor_files_match_pinned_upstream() -> None:
    for name, expected in EXPECTED_HASHES.items():
        actual = hashlib.sha256((VENDOR / name).read_bytes()).hexdigest()
        assert actual == expected, name

    source_record = (VENDOR / "VENDOR_SOURCE.md").read_text(encoding="utf-8")
    assert "252ac2859ac1d2010915b7a7cfcbf501d7adfb40" in source_record
    assert "Local modifications to imported files: none" in source_record


def test_bmi160_defaults_match_verified_hardware_and_cartographer_contract() -> None:
    params = yaml.safe_load((ROOT / "param" / "bmi160.yaml").read_text(encoding="utf-8"))
    values = params["bmi160_node"]["ros__parameters"]
    assert values["i2c_device"] == "/dev/i2c-1"
    assert values["i2c_address"] == 0x68
    assert values["frame_id"] == "imu_link"
    assert values["topic_name"] == "imu/data_raw"
    assert values["publish_rate_hz"] == 100.0
    assert values["gyro_bias_samples"] >= 200
    assert sorted(abs(axis) for axis in values["axis_map"]) == [1, 2, 3]


def test_package_installs_runtime_and_vendor_license() -> None:
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (ROOT / "package.xml").read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "bmi160.launch.py").read_text(encoding="utf-8")

    assert "add_executable(bmi160_node" in cmake
    assert "vendor/bosch_bmi160/bmi160.c" in cmake
    assert "vendor/bosch_bmi160/LICENSE" in cmake
    assert "VENDOR_SOURCE.md" in cmake
    assert "<license>BSD-3-Clause</license>" in package
    assert "<depend>rclcpp</depend>" in package
    assert "<depend>sensor_msgs</depend>" in package
    assert "DeclareLaunchArgument" in launch
    assert "ParameterValue(i2c_address, value_type=int)" in launch


if __name__ == "__main__":
    test_bosch_vendor_files_match_pinned_upstream()
    test_bmi160_defaults_match_verified_hardware_and_cartographer_contract()
    test_package_installs_runtime_and_vendor_license()
