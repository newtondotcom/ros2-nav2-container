# dionybot

```mermaid
graph TD
    %% Services sur chaque noeud
    subgraph RoverServices [dionybot : Rover on ROS2]
        PX4[PX4 Firmware]
        Nav2[Nav2 Navigation Stack]
        GNSS[Récepteur GNSS RTK]
        Lidar[Lidar]
        USS[Ultrasonic Sensor]
        Bridge[px4_ros_bridge]
        MissionClient[VDA5050-compatible mission client]
        Cameras[Cameras]
        DepthCamera[Depth Camera]
        Mux[Command Multiplexer]
        RowFollower[RowFollower]
        Supervisor[Supervisor FSM Node]
        DockingServer[Docking Server]
    end

    subgraph DockServices [Dock : Relay & Recharge]
        Mission[VDA5050-compatible mission controller]
        BaseRTK[Base GNSS-RTK]
        Panel[Control Panel: Emergency stop, RTH]
    end

    subgraph CloudServices [Cloud : Opérations Center]
        WebUI[Web Interface]
        Planner[Mission Planner]
    end

    %% Flèches de communication

    WebUI <-->|Mission Management| Planner
    Planner <--->|Fibre avec IP fixe| Mission

    GNSS <---|LoRaWAN / RF / WiF| BaseRTK   
    MissionClient <-->|LoRaWAN / RF / WiFi| Mission

    PX4 -->|/fmu/out/vehicle_odometry| Bridge
    PX4 -->|/fmu/out/vehicle_GNSS_position| Bridge

    Bridge  -->|/fmu/in/offboard_control_mode| PX4
    Bridge  -->|/fmu/in/trajectory_setpoint| PX4
    Bridge  -->|/fmu/in/vehicle_command| PX4
    Bridge -->|/gps/fix| Nav2
    Bridge -->|/odom| Nav2

    Cameras -->|sensor_msgs/Image| Nav2
    USS -->|sensor_msgs/Range| Nav2
    Nav2 -->|/cmd_vel_nav2| Mux
    RowFollower -->|/cmd_vel_row| Mux
    Panel -->|/cmd_vel_teleop| Mux
    Mux -->|/cmd_vel| Bridge 
    GNSS -->PX4      
    Lidar -->|/scan| Nav2 
    DepthCamera-->|PointCloud2| Nav2    
    DockingServer -->|/cmd_vel_dock| Mux
    Supervisor -->|NavigateToPose| Nav2

    MissionClient -->|/mission| Supervisor

    
```