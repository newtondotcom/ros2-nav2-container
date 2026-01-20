# TO add on build

> a copier dans [init-container.sh](../../../scripts/init-container.sh) dynamiquemernt

- https://github.com/PX4/PX4-gazebo-models/tree/main/models/OakD-Lite
- https://github.com/PX4/PX4-gazebo-models/tree/main/models/lidar_2d_v2
- https://github.com/PX4/PX4-gazebo-models/tree/main/models/mono_cam
- https://classic.gazebosim.org/tutorials?tut=wide_angle_camera&branch=wideanglecamera
- sonar for range :

```xml
<sensor name="front_left" type="gpu_lidar">
  <topic>ultrasonic</topic>
  <update_rate>20</update_rate>
  <lidar>
    <horizontal>
      <samples>1</samples>
      <min_angle>0</min_angle>
      <max_angle>0</max_angle>
    </horizontal>
    <range>
      <min>0.08</min>
      <max>30.0</max>
    </range>
  </lidar>
</sensor>
```

Example in bridge.yaml:

```yml
- ros_topic_name: "/mavros/rangefinder_sub"
  gz_topic_name: "/ultrasonic"
  ros_type_name: "sensor_msgs/msg/Range"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```