# Autonomous Security Patrol - Standalone Project

This is a complete, self-contained ROS2 project for autonomous robot security patrol demonstration. It includes all necessary assets, configuration, and launchers without external dependencies on the `origins/` packages.

## Project Structure

```
src/standalone/
├── robot_description/        # Robot URDF models, meshes, and visualization
│   ├── urdf/                 # URDF/Xacro files
│   ├── meshes/               # 3D model files
│   ├── rviz/                 # RViz configuration
│   ├── launch/               # Launch files
│   ├── package.xml
│   └── CMakeLists.txt
│
├── simulation/               # Gazebo simulation environment
│   ├── worlds/               # SDF world files (patrol_world.sdf)
│   ├── configs/              # Gazebo bridge configurations
│   ├── launch/               # Simulation launch files
│   │   └── spawn_tb4.launch.py
│   ├── package.xml
│   └── CMakeLists.txt
│
├── navigation/               # Nav2 navigation stack configuration
│   ├── maps/                 # Navigation maps (patrol_map.yaml/pgm)
│   ├── params/               # Nav2 parameters (patrol_nav2_params.yaml)
│   ├── launch/               # Navigation launch files
│   ├── package.xml
│   └── CMakeLists.txt
│
└── patrol_behavior/          # Autonomous patrol behavior
    ├── patrol_behavior/
    │   ├── patrol_executor.py    # Main autonomous patrol node
    │   ├── launch/
    │   │   └── security_patrol_launch.py  # Main launcher
    │   └── config/
    │       └── patrol_routes.yaml         # Patrol waypoints configuration
    ├── resource/
    ├── setup.py
    ├── package.xml
    └── CMakeLists.txt
```

## Features

- **Fully Standalone**: No dependencies on `origins/` packages
- **Modular Design**: 4 independent packages for easy customization
- **Configurable Routes**: Patrol waypoints defined in YAML
- **Gazebo Integration**: Full simulation with physics and sensors
- **Nav2 Stack**: Complete navigation with AMCL localization and path planning
- **RViz Visualization**: Real-time robot and navigation visualization
- **Autonomous Patrol**: Security patrol with route reversal and continuous looping

## Assets

### Robot
- **URDF**: `turtlebot4.urdf.xacro` (renamed from origins)
- **Sensors**: RPLidar, OAK-D camera, IMU
- **Meshes**: Complete 3D models for visualization and collision

### Environment
- **World**: `patrol_world.sdf` (depot.sdf renamed)
- **Map**: `patrol_map.yaml/pgm` (depot map adapted)
- **Models**: Gazebo models for the warehouse environment

### Navigation
- **Nav2 Parameters**: `patrol_nav2_params.yaml`
- **Localization**: AMCL with initial pose (-8.0, 0.0)
- **Planning**: Global and local path planning

### Routes
- **File**: `patrol_routes.yaml`
- **Waypoints**: 5-point warehouse security patrol loop
- **Behavior**: 
  - Route reversal between iterations
  - Continuous looping
  - 180-second timeout per waypoint

## Installation & Building

### 1. Copy Assets
All assets are already copied. If you need to regenerate:

```bash
cd ~/ros2-nav2-container
# Assets are in src/standalone/ - ready to build
```

### 2. Build Packages

```bash
cd ~/ros2-nav2-container
colcon build --symlink-install --packages-select robot_description simulation navigation patrol_behavior
```

### 3. Source Environment

```bash
source ~/ros2-nav2-container/install/setup.bash
```

## Running the Demo

### Start the Complete Patrol Demo

```bash
ros2 launch patrol_behavior security_patrol_launch.py
```

**Optional Arguments**:
- `use_rviz:=True/False` - Enable/disable RViz visualization (default: True)
- `headless:=True/False` - Gazebo headless mode (default: False)

Example:
```bash
ros2 launch patrol_behavior security_patrol_launch.py headless:=True use_rviz:=False
```

### What Starts

1. **Gazebo Server** - Physics simulation engine
2. **Gazebo Client** - 3D visualization (if not headless)
3. **TurtleBot4 Robot** - Spawned at position (-8.0, 0.0)
4. **Robot State Publisher** - Publishes robot transforms
5. **Nav2 Stack** - Navigation and localization
   - AMCL (Monte Carlo localization)
   - Global path planner
   - Local path controller
   - Behavior tree navigator
6. **RViz** - ROS visualization (if enabled)
7. **Patrol Executor** - Autonomous patrol node

## Customizing the Patrol

### Edit Patrol Routes

Edit `src/standalone/patrol_behavior/patrol_behavior/config/patrol_routes.yaml`:

```yaml
patrol_routes:
  security_patrol:
    waypoints:
      - position: [10.15, -0.77, 0.0]
        description: "Waypoint description"
      - position: [17.86, -0.77, 0.0]
        description: "Next waypoint"
      # Add more waypoints...

behavior:
  waypoint_timeout_seconds: 180
  enable_reverse_route: true
  loop_patrol: true
```

### Change the Robot

1. Replace URDF in `robot_description/urdf/standard/`
2. Update mesh file paths in new URDF files
3. Ensure sensor configuration matches new robot
4. Rebuild: `colcon build --symlink-install`

### Change the Environment

1. Replace world file: `simulation/worlds/patrol_world.sdf`
2. Create new map: `navigation/maps/patrol_map.yaml/pgm`
3. Update map origin/resolution in YAML if needed
4. Rebuild: `colcon build --symlink-install`

### Modify Navigation Parameters

Edit `navigation/params/patrol_nav2_params.yaml`:
- AMCL localization tuning
- Costmap resolution and size
- Path planning algorithms
- Controller parameters

## Dependencies

### External ROS2 Packages (automatically installed)
- `nav2_bringup` - Navigation launch files and utilities
- `ros_gz_sim` - Gazebo-ROS2 bridge
- `ros_gz_bridge` - Sensor/actuator bridging
- `ros_gz_image` - Camera image bridging
- `robot_state_publisher` - TF publication
- `nav2_simple_commander` - High-level navigation API

### Python Dependencies
- `pyyaml` - YAML configuration loading
- `rclpy` - ROS2 Python client library

All are installed via `package.xml` dependencies.

## Network Configuration

### Gazebo Simulator Bridge

The `spawn_tb4.launch.py` configures bridges between Gazebo and ROS2:

**Topics Published** (Gazebo → ROS2):
- `/joint_states` - Robot joint angles/velocities
- `/odom` - Odometry from simulation
- `/scan` - Lidar point cloud
- `/tf`, `/tf_static` - Transform tree

**Topics Subscribed** (ROS2 → Gazebo):
- `/cmd_vel` - Velocity commands to robot motors

**Configuration File**: `simulation/configs/tb4_bridge.yaml`

## Troubleshooting

### 1. "Package not found" errors
```bash
# Ensure packages are built
colcon build --symlink-install

# Source install setup
source ~/ros2-nav2-container/install/setup.bash
```

### 2. Gazebo doesn't start
```bash
# Check Gazebo installation
gz sim --version

# Run with verbose output
ros2 launch patrol_behavior security_patrol_launch.py --log-level debug
```

### 3. Robot doesn't localize
- Check that AMCL is running: `ros2 node list | grep amcl`
- Verify map file exists: `ls -la src/standalone/navigation/maps/patrol_map.*`
- Check RViz: Map should load without errors

### 4. Navigation fails
```bash
# Check Nav2 stack status
ros2 service call /bt_navigator/get_state nav2_msgs/srv/GetState

# View planning debug info
ros2 run rviz2 rviz2 -d install/patrol_behavior/share/patrol_behavior/rviz/debug.rviz
```

### 5. URDF parsing errors
```bash
# Validate URDF/Xacro
xacro src/standalone/robot_description/urdf/standard/turtlebot4.urdf.xacro | head -20

# Check package references are correct
grep -r "find robot_description" src/standalone/robot_description/urdf/
```

## File Renaming Reference

Original → Standalone:
- `turtlebot4.urdf.xacro` → (renamed but same file)
- `depot.sdf` → `patrol_world.sdf`
- `depot.yaml/pgm` → `patrol_map.yaml/pgm`
- `nav2_params.yaml` → `patrol_nav2_params.yaml`
- Package ref: `nav2_minimal_tb4_description` → `robot_description`
- Package ref: `nav2_minimal_tb4_sim` → `simulation`
- Package ref: `nav2_bringup` → (external, used as-is)

## Development

### Adding New Routes

1. Edit `patrol_routes.yaml`
2. Define new route under `patrol_routes` section
3. Update `patrol_executor.py` to use new route name
4. Rebuild: `colcon build --symlink-install`

### Adding New Sensors

1. Add sensor definition to robot URDF
2. Add bridging configuration in `simulation/configs/tb4_bridge.yaml`
3. Update navigation parameters if needed
4. Rebuild

### Custom Autonomous Behaviors

The `patrol_executor.py` can be extended with:
- Threat detection integration
- Multi-robot coordination
- Dynamic route planning
- Real-time obstacle avoidance
- Custom recovery behaviors

See inline comments in `patrol_executor.py` for extension points.

## License

Apache License 2.0 - See LICENSE file in root directory

## References

- ROS 2 Documentation: https://docs.ros.org/
- Nav2 Documentation: https://navigation.ros.org/
- Gazebo Simulator: https://gazebosim.org/
- TurtleBot 4: https://turtlebot.github.io/
