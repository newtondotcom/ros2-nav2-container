first:
	gz sim -v4 /workspaces/ros2-devcontainer/src/nav2_gps_waypoint_follower_demo/worlds/tb3_sonoma_raceway.sdf.xacro
pre:
	ros2 launch nav2_gps_waypoint_follower_demo gazebo_gps_world.launch.py

launch:
	ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py

teleop:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --param stamped:=True

rviz:
	ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py use_rviz:=True

mapviz:
	ros2 run nav2_gps_waypoint_follower_demo interactive_waypoint_follower

gen_file:
	ros2 run nav2_gps_waypoint_follower_demo gps_waypoint_logger config/temp.yaml

from_file:
	ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower config/temp.yaml