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

import time
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
import rclpy


def main():
    rclpy.init()
    
    # Create a simple node to bridge navigation commands
    nav = BasicNavigator()
    
    # Load patrol routes from config
    patrol_config_path = get_package_share_directory('patrol_behavior')
    routes_file = patrol_config_path + '/config/patrol_routes.yaml'
    
    try:
        with open(routes_file, 'r') as f:
            config = yaml.safe_load(f)
        routes = config.get('patrol_routes', {})
        behavior_config = config.get('behavior', {})
    except Exception as e:
        print(f"Error loading patrol routes: {e}")
        # Fallback to hardcoded routes
        routes = {
            'security_patrol': {
                'waypoints': [
                    {'position': [10.15, -0.77, 0.0]},
                    {'position': [17.86, -0.77, 0.0]},
                    {'position': [21.58, -3.5, 0.0]},
                    {'position': [19.4, -6.4, 0.0]},
                    {'position': [-6.0, 5.0, 0.0]},
                ]
            }
        }
        behavior_config = {
            'waypoint_timeout_seconds': 180,
            'enable_reverse_route': True,
            'loop_patrol': True
        }
    
    # Wait for navigation stack to be ready
    print("Waiting for nav2 to be ready...")
    nav.waitUntilNav2Active()
    
    # Initial robot pose (approximate, AMCL will refine)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -8.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    nav.setInitialPose(initial_pose)
    print(f"Initial pose set to: x={initial_pose.pose.position.x}, y={initial_pose.pose.position.y}")
    
    # Give Nav2 a moment to initialize
    time.sleep(2)
    
    # Main patrol loop
    route_name = 'security_patrol'
    if route_name in routes:
        route = routes[route_name]
        waypoints_config = route.get('waypoints', [])
        max_timeout = behavior_config.get('waypoint_timeout_seconds', 180)
        enable_reverse = behavior_config.get('enable_reverse_route', True)
        loop_patrol = behavior_config.get('loop_patrol', True)
        
        # Convert config to PoseStamped objects
        waypoints = []
        for wp in waypoints_config:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp['position'][0])
            pose.pose.position.y = float(wp['position'][1])
            pose.pose.position.z = float(wp['position'][2])
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
        
        print(f"Loaded {len(waypoints)} waypoints for route '{route_name}'")
        
        patrol_iteration = 0
        route_direction = 1  # 1 for forward, -1 for reverse
        
        while rclpy.ok():
            patrol_iteration += 1
            print(f"\n{'='*60}")
            print(f"Patrol Iteration {patrol_iteration}")
            print(f"Route direction: {'Forward' if route_direction == 1 else 'Reverse'}")
            print(f"{'='*60}")
            
            # Reorder waypoints if going in reverse
            current_waypoints = waypoints if route_direction == 1 else waypoints[::-1]
            
            # Navigate through waypoints
            start_time = time.time()
            for idx, waypoint in enumerate(current_waypoints):
                elapsed = time.time() - start_time
                print(f"\nWaypoint {idx+1}/{len(current_waypoints)}")
                print(f"Position: x={waypoint.pose.position.x:.2f}, y={waypoint.pose.position.y:.2f}")
                print(f"Elapsed: {elapsed:.0f}s")
                
                # Send waypoint goal
                nav.goToPose(waypoint)
                
                # Monitor progress with timeout
                start_waypoint_time = time.time()
                while not nav.isTaskComplete():
                    feedback = nav.getFeedback()
                    if feedback and hasattr(feedback, 'estimated_time_remaining'):
                        remaining_time = feedback.estimated_time_remaining.sec
                        print(f"  Distance remaining: {remaining_time}s", end='\r')
                    
                    # Check timeout
                    waypoint_elapsed = time.time() - start_waypoint_time
                    if waypoint_elapsed > max_timeout:
                        print(f"\n  Waypoint timeout ({max_timeout}s exceeded)!")
                        nav.cancelTask()
                        break
                    
                    time.sleep(0.1)
                
                # Check result
                result = nav.getResult()
                if result == nav.TaskResult.SUCCEEDED:
                    print(f"  ✓ Waypoint reached successfully")
                elif result == nav.TaskResult.CANCELED:
                    print(f"  ✗ Waypoint navigation canceled")
                elif result == nav.TaskResult.FAILED:
                    print(f"  ✗ Waypoint navigation failed")
                else:
                    print(f"  ? Unknown result: {result}")
            
            # Route completed
            print(f"\n✓ Route completed in {time.time() - start_time:.1f}s")
            
            if not loop_patrol:
                print("Patrol completed. Exiting.")
                break
            
            # Change direction if enabled
            if enable_reverse:
                route_direction *= -1
                print(f"Reversing route for next iteration")
            
            print("Starting next patrol iteration in 5 seconds...")
            time.sleep(5)
    
    else:
        print(f"Route '{route_name}' not found in configuration")
    
    # Clean up
    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
