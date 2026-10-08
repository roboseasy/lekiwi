from pathlib import Path


def read_base_rviz() -> str:
    return Path("rviz", "base.rviz").read_text(encoding="utf-8")


def test_base_rviz_config_exists() -> None:
    assert Path("rviz", "base.rviz").is_file()


def test_base_rviz_uses_odom_fixed_frame() -> None:
    rviz = read_base_rviz()

    assert "Fixed Frame: odom" in rviz


def test_base_rviz_shows_robot_model_tf_odometry_and_scan() -> None:
    rviz = read_base_rviz()

    assert "Class: rviz_default_plugins/RobotModel" in rviz
    assert "Value: /robot_description" in rviz
    assert "Class: rviz_default_plugins/TF" in rviz
    assert "Class: rviz_default_plugins/Odometry" in rviz
    assert "Covariance:" in rviz
    assert rviz.count("Value: false") >= 2
    assert "Topic:" in rviz
    assert "Value: /odom" in rviz
    assert "Class: rviz_default_plugins/LaserScan" in rviz
    assert "Reliability Policy: Best Effort" in rviz
    assert "Value: /scan" in rviz


if __name__ == "__main__":
    test_base_rviz_config_exists()
    test_base_rviz_uses_odom_fixed_frame()
    test_base_rviz_shows_robot_model_tf_odometry_and_scan()
