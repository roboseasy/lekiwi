from setuptools import setup

package_name = "lekiwi_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ysj",
    maintainer_email="ysj@roboseasy.ai",
    description="Teleoperation package for LeKiwi.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lekiwi_base_teleop = lekiwi_teleop.base_teleop:main",
            "lekiwi_guarded_base_teleop = lekiwi_teleop.guarded_base_teleop:main",
            "lekiwi_odom_calibration = lekiwi_teleop.odom_calibration:main",
            "lekiwi_distance_trial = lekiwi_teleop.distance_trial:main",
            "lekiwi_safe_cmd_vel = lekiwi_teleop.safe_cmd_vel:main",
            "lekiwi_nav2_motor_guard = lekiwi_teleop.nav2_motor_guard:main",
        ],
    },
)
