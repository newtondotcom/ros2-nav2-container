import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    VehicleGpsPosition,
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus


SQRT_HALF = math.sqrt(0.5)
Q_ROT_NED_TO_ENU = (0.0, SQRT_HALF, SQRT_HALF, 0.0)


def quat_multiply(q1, q2):
    """Hamilton product of two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


class Px4RosBridge(Node):
    def __init__(self):
        super().__init__("px4_ros_bridge_node")

        # QoS for PX4 subscribers (incoming from PX4)
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # QoS for PX4 publishers (outgoing to PX4)
        qos_profile_pub_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ROS standard publishers
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # PX4 offboard publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile_pub_px4
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile_pub_px4
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile_pub_px4
        )

        # PX4 subscribers
        self.gps_sub = self.create_subscription(
            VehicleGpsPosition,
            "/fmu/out/vehicle_gps_position",
            self._gps_callback,
            qos_profile_sub,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_callback,
            qos_profile_sub,
        )

        # ROS cmd_vel subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        # Offboard state
        self.last_cmd_vel: Optional[Twist] = None
        self.offboard_armed = False

        self.last_message_time: Optional[rclpy.time.Time] = None
        self.warned = False
        self.create_timer(5.0, self._warn_if_inactive)

        # Offboard timer (20 Hz)
        self.create_timer(0.05, self._offboard_timer_callback)

        # Send ARM + OFFBOARD commands after a short delay
        self.create_timer(2.0, self._send_arm_and_offboard_once)

        self.get_logger().info(
            "PX4 <-> ROS bridge started\n"
            "Subscribing to:\n"
            " - /fmu/out/vehicle_gps_position\n"
            " - /fmu/out/vehicle_odometry\n"
            " - /cmd_vel\n"
            "Publishing:\n"
            " - /gps/fix\n"
            " - /odom\n"
            " - /fmu/in/offboard_control_mode\n"
            " - /fmu/in/trajectory_setpoint\n"
            " - /fmu/in/vehicle_command"
        )

    def _touch_activity(self):
        self.last_message_time = self.get_clock().now()

    def _warn_if_inactive(self):
        now = self.get_clock().now()
        if self.last_message_time is None:
            if not self.warned:
                self.get_logger().warn("No PX4 messages received after 5s.")
                self.warned = True
            return

        inactive_duration = (now - self.last_message_time).nanoseconds / 1e9
        if inactive_duration > 5.0 and not self.warned:
            self.get_logger().warn(
                f"No PX4 messages received for {inactive_duration:.1f}s."
            )
            self.warned = True
        elif inactive_duration <= 5.0:
            self.warned = False

    def _gps_callback(self, msg: VehicleGpsPosition):
        self._touch_activity()

        nav_fix = NavSatFix()
        nav_fix.header.stamp = self.get_clock().now().to_msg()
        nav_fix.header.frame_id = "gps_link"

        nav_fix.latitude = msg.lat * 1e-7
        nav_fix.longitude = msg.lon * 1e-7
        nav_fix.altitude = msg.alt / 1000.0

        status = NavSatStatus()
        status.service = NavSatStatus.SERVICE_GPS
        status.status = NavSatStatus.STATUS_NO_FIX
        if msg.fix_type >= 3:
            status.status = NavSatStatus.STATUS_FIX
        if msg.fix_type >= 5:
            status.status = NavSatStatus.STATUS_SBAS_FIX
        if msg.fix_type == 6:
            status.status = NavSatStatus.STATUS_GBAS_FIX
        nav_fix.status = status

        if math.isfinite(msg.eph) and math.isfinite(msg.epv) and msg.eph > 0.0 and msg.epv > 0.0:
            nav_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            nav_fix.position_covariance[0] = msg.eph * msg.eph
            nav_fix.position_covariance[4] = msg.eph * msg.eph
            nav_fix.position_covariance[8] = msg.epv * msg.epv
        else:
            nav_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(nav_fix)

    def _odom_callback(self, msg: VehicleOdometry):
        self._touch_activity()

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = msg.position[1]
        odom.pose.pose.position.y = msg.position[0]
        odom.pose.pose.position.z = -msg.position[2]

        q_px4 = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        q_enu = quat_multiply(Q_ROT_NED_TO_ENU, q_px4)
        odom.pose.pose.orientation.w = q_enu[0]
        odom.pose.pose.orientation.x = q_enu[1]
        odom.pose.pose.orientation.y = q_enu[2]
        odom.pose.pose.orientation.z = q_enu[3]

        odom.twist.twist.linear.x = msg.velocity[1]
        odom.twist.twist.linear.y = msg.velocity[0]
        odom.twist.twist.linear.z = -msg.velocity[2]

        odom.twist.twist.angular.x = msg.angular_velocity[1]
        odom.twist.twist.angular.y = msg.angular_velocity[0]
        odom.twist.twist.angular.z = -msg.angular_velocity[2]

        self._set_pose_covariance(odom, msg)
        self._set_twist_covariance(odom, msg)

        self.odom_pub.publish(odom)

    def _set_pose_covariance(self, odom: Odometry, msg: VehicleOdometry):
        cov = [0.0] * 36
        if len(msg.position_variance) >= 3:
            cov[0] = msg.position_variance[1]
            cov[7] = msg.position_variance[0]
            cov[14] = msg.position_variance[2]
        if len(msg.orientation_variance) >= 3:
            cov[21] = msg.orientation_variance[0]
            cov[28] = msg.orientation_variance[1]
            cov[35] = msg.orientation_variance[2]
        odom.pose.covariance = cov

    def _set_twist_covariance(self, odom: Odometry, msg: VehicleOdometry):
        cov = [0.0] * 36
        if len(msg.velocity_variance) >= 3:
            cov[0] = msg.velocity_variance[1]
            cov[7] = msg.velocity_variance[0]
            cov[14] = msg.velocity_variance[2]
        odom.twist.covariance = cov

    def _cmd_vel_callback(self, msg: Twist):
        """Cache cmd_vel for offboard control. Forward speed only, no yaw."""
        self.last_cmd_vel = msg

    def _offboard_timer_callback(self):
        """Stream offboard control mode and trajectory setpoint at 20 Hz."""
        if self.last_cmd_vel is None:
            return

        now = self.get_clock().now().to_msg()

        # Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Publish TrajectorySetpoint
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position[0] = float("nan")
        setpoint.position[1] = float("nan")
        setpoint.position[2] = float("nan")
        setpoint.velocity[0] = self.last_cmd_vel.linear.x  # vx (forward)
        setpoint.velocity[1] = 0.0  # vy (lateral, zero for rover)
        setpoint.velocity[2] = 0.0  # vz (vertical, zero for rover)
        setpoint.yaw = float("nan")  # Hold current yaw
        setpoint.yawspeed = 0.0  # No yaw rate control
        self.trajectory_setpoint_pub.publish(setpoint)

    def _send_arm_and_offboard_once(self):
        """Send one-time ARM and OFFBOARD commands to PX4."""
        if self.offboard_armed:
            return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Command: ARM (CMD_COMPONENT_ARM_DISARM = 400)
        arm_cmd = VehicleCommand()
        arm_cmd.timestamp = timestamp
        arm_cmd.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0  # 1 = ARM
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.source_system = 1
        arm_cmd.source_component = 1
        arm_cmd.from_external = True
        self.vehicle_command_pub.publish(arm_cmd)

        # Command: Set Mode to OFFBOARD (MAV_CMD_DO_SET_MODE = 176)
        mode_cmd = VehicleCommand()
        mode_cmd.timestamp = timestamp
        mode_cmd.command = 176  # MAV_CMD_DO_SET_MODE
        mode_cmd.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        mode_cmd.param2 = 6.0  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        mode_cmd.target_system = 1
        mode_cmd.target_component = 1
        mode_cmd.source_system = 1
        mode_cmd.source_component = 1
        mode_cmd.from_external = True
        self.vehicle_command_pub.publish(mode_cmd)

        self.offboard_armed = True
        self.get_logger().info("Sent ARM + OFFBOARD commands to PX4")


def main(args=None):
    rclpy.init(args=args)
    node = Px4RosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
