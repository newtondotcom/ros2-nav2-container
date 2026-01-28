nav2:
	git clone https://github.com/ros-navigation/navigation2.git --branch main src/navigation2
	git clone https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation.git --branch main src/nav2_minimal_turtlebot_simulation
	rosdep install -r -y \
	--from-paths src \
	--ignore-src
	colcon build \
	--symlink-install

clean:
	rm -rf src/navigation2

auto:
	ros2 launch nav2_simple_commander security_demo_launch.py

manual:
	# Terminal 1: launch your robot navigation and simulation (or physical robot). For example
	ros2 launch nav2_bringup tb3_simulation_launch.py world:=/path/to/aws_robomaker_small_warehouse_world/.world map:=/path/to/aws_robomaker_small_warehouse_world/.yaml

	# Terminal 2: launch your autonomy / application demo or example. For example
	ros2 run nav2_simple_commander demo_security

kill:
	pkill -f gz

test:
	bash scripts/build.sh
	source install/setup.bash
	ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py

lint:
	docker run -v .:/code --rm pipelinecomponents/ruff ruff format .