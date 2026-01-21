map:
	git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git src/aws-robomaker-small-warehouse-world

nav2:
	git clone https://github.com/ros-navigation/navigation2.git src/navigation2

sim:
	git clone https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation src/nav2_minimal_turtlebot_simulation

clean:
	rm -rf src/aws-robomaker-small-warehouse-world
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
	colcon --log-level info build --symlink-install --event-handlers "console_direct+"
	source install/setup.bash
	ros2 launch patrol_behavior security_patrol_launch.py