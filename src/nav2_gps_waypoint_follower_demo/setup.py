from setuptools import find_packages, setup
import os
from glob import glob

package_name = "nav2_gps_waypoint_follower_demo"

def generate_data_files():
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, package_name), glob(package_name + "/*.py")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ]
    ## parse recursively the models and urdf directories
    for root, _, files in os.walk("models"):
        for file in files:
            data_files.append((os.path.join("share", package_name, "models", root), glob(os.path.join(root, file))))
    for root, _, files in os.walk("urdf"):
        for file in files:
            data_files.append((os.path.join("share", package_name, "urdf", root), glob(os.path.join(root, file))))
    return data_files

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=generate_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="pedro.gonzalez@eia.edu.co",
    description="Demo package for following GPS waypoints with nav2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "logged_waypoint_follower = nav2_gps_waypoint_follower_demo.logged_waypoint_follower:main",
            "interactive_waypoint_follower = nav2_gps_waypoint_follower_demo.interactive_waypoint_follower:main",
            "gps_waypoint_logger = nav2_gps_waypoint_follower_demo.gps_waypoint_logger:main",
        ],
    },
)
