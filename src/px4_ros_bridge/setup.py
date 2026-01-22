from setuptools import find_packages, setup
import os
from glob import glob

package_name = "px4_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Developer",
    maintainer_email="dev@example.com",
    description="ROS2 bridge converting PX4 px4_msgs topics to NavSatFix and Odometry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_ros_bridge = px4_ros_bridge.px4_ros_bridge:main",
        ],
    },
)
