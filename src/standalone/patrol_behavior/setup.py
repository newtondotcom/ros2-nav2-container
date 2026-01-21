from setuptools import find_packages, setup

package_name = "patrol_behavior"

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
    author="Developer",
    author_email="dev@example.com",
    maintainer="Developer",
    maintainer_email="dev@example.com",
    url="https://github.com/example/patrol_behavior",
    download_url="https://github.com/example/patrol_behavior/releases",
    keywords=["ROS", "robotics", "patrol", "navigation", "autonomous"],
    classifiers=[
        "Environment :: Console",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
    ],
    description="Autonomous patrol behavior for security robots",
    long_description="Standalone package for autonomous security patrol with ROS 2 and Nav2.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_executor = patrol_behavior.patrol_executor:main",
        ],
    },
)
