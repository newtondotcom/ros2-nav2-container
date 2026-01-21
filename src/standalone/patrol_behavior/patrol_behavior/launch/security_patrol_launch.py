#!/usr/bin/env python3
# Copyright (c) 2024 Standalone Security Patrol
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_desc_dir = get_package_share_directory("robot_description")
    sim_dir = get_package_share_directory("simulation")
    nav_dir = get_package_share_directory("navigation")

    robot_urdf = os.path.join(
        robot_desc_dir, "urdf", "standard", "turtlebot4.urdf.xacro"
    )
    world = os.path.join(sim_dir, "worlds", "patrol_world.sdf")
    map_yaml_file = os.path.join(nav_dir, "maps", "patrol_map.yaml")
    nav_params = os.path.join(nav_dir, "params", "patrol_nav2_params.yaml")

    # Launch configuration variables
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    # Start the simulation (generate a temporary SDF from xacro)
    world_sdf_fd, world_sdf_path = tempfile.mkstemp(prefix="patrol_", suffix=".sdf")
    os.close(world_sdf_fd)
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", world, "headless:=", headless, "-o", world_sdf_path]
    )

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r -s {world_sdf_path}"}.items(),
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda _: os.remove(world_sdf_path)
                    if os.path.exists(world_sdf_path)
                    else None
                )
            ]
        )
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "worlds")
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["not ", headless])),
        launch_arguments={"gz_args": "-v4 -g "}.items(),
    )

    # Spawn robot - using spawn_tb4.launch.py from simulation package
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb4.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "robot_sdf": robot_urdf,
            "x_pose": "-8.0",
            "y_pose": "0.0",
            "z_pose": "0.0",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
        }.items(),
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro", " ", robot_urdf]),
            }
        ],
    )

    # Start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={"namespace": ""}.items(),
    )

    # Start navigation with custom params
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={"map": map_yaml_file, "params_file": nav_params}.items(),
    )

    # Start the patrol behavior
    patrol_cmd = Node(
        package="patrol_behavior",
        executable="patrol_executor.py",
        emulate_tty=True,
        output="screen",
    )

    set_env_vars_resources2 = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(Path(os.path.join(robot_desc_dir)).parent.resolve())
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(patrol_cmd)
    return ld
