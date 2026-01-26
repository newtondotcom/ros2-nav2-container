#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

from px4_msgs.msg import (
    VehicleGpsPosition,
    VehicleOdometry,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)

from tf2_ros import TransformBroadcaster


# ------------------------
# Utils
# ------------------------

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


# ------------------------
# Node
# ------------------------


class Px4RosBridge(Node):
    def __init__(self):
        super().__init__("px4_ros_bridge")

        # ---------------- QoS ----------------

        qos_px4_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        qos_px4_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # ---------------- Publishers ----------------

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_px4_pub
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_px4_pub
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_px4_pub
        )

        # ---------------- Subscribers ----------------

        self.create_subscription(
            VehicleGpsPosition,
            "/fmu/out/vehicle_gps_position",
            self._gps_callback,
            qos_px4_sub,
        )

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_callback,
            qos_px4_sub,
        )

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)

        # ---------------- TF ----------------

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- State ----------------

        self.last_cmd_vel: Optional[Twist] = None
        self.last_cmd_vel_time: Optional[rclpy.time.Time] = None

        self.cmd_vel_timeout = 0.5  # seconds

        self.offboard_counter = 0
        self.offboard_enabled = False

        # ---------------- Timers ----------------

        self.create_timer(0.05, self._offboard_timer)  # 20 Hz
        self.create_timer(1.0, self._try_arm_and_offboard)

        self.get_logger().info("PX4 ROS2 bridge started")

    # ============================================================
    # Callbacks
    # ============================================================

    def _cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    # ---------------- GPS ----------------

    def _gps_callback(self, msg: VehicleGpsPosition):
        nav_fix = NavSatFix()
        nav_fix.header.stamp = self.get_clock().now().to_msg()
        nav_fix.header.frame_id = "gps_link"

        nav_fix.latitude = msg.lat * 1e-7
        nav_fix.longitude = msg.lon * 1e-7
        nav_fix.altitude = msg.alt / 1000.0

        status = NavSatStatus()
        status.service = NavSatStatus.SERVICE_GPS

        if msg.fix_type < 3:
            status.status = NavSatStatus.STATUS_NO_FIX
        elif msg.fix_type < 5:
            status.status = NavSatStatus.STATUS_FIX
        elif msg.fix_type < 6:
            status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            status.status = NavSatStatus.STATUS_GBAS_FIX

        nav_fix.status = status

        if msg.eph > 0 and msg.epv > 0:
            nav_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            nav_fix.position_covariance[0] = msg.eph**2
            nav_fix.position_covariance[4] = msg.eph**2
            nav_fix.position_covariance[8] = msg.epv**2
        else:
            nav_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(nav_fix)

    # ---------------- Odometry ----------------

    def _odom_callback(self, msg: VehicleOdometry):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position NED -> ENU
        odom.pose.pose.position.x = msg.position[1]
        odom.pose.pose.position.y = msg.position[0]
        odom.pose.pose.position.z = -msg.position[2]

        # Orientation
        q_px4 = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        q_enu = quat_multiply(Q_ROT_NED_TO_ENU, q_px4)

        odom.pose.pose.orientation.w = q_enu[0]
        odom.pose.pose.orientation.x = q_enu[1]
        odom.pose.pose.orientation.y = q_enu[2]
        odom.pose.pose.orientation.z = q_enu[3]

        # Velocity
        odom.twist.twist.linear.x = msg.velocity[1]
        odom.twist.twist.linear.y = msg.velocity[0]
        odom.twist.twist.linear.z = -msg.velocity[2]

        odom.twist.twist.angular.x = msg.angular_velocity[1]
        odom.twist.twist.angular.y = msg.angular_velocity[0]
        odom.twist.twist.angular.z = -msg.angular_velocity[2]

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    # ============================================================
    # Offboard
    # ============================================================

    def _offboard_timer(self):
        self.offboard_counter += 1

        if self.last_cmd_vel_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        if dt > self.cmd_vel_timeout:
            vx = 0.0
            wz = 0.0
        else:
            vx = self.last_cmd_vel.linear.x
            wz = self.last_cmd_vel.angular.z

        # Offboard control mode
        offboard = OffboardControlMode()
        offboard.timestamp = int(now.nanoseconds / 1000)
        offboard.velocity = True
        self.offboard_control_mode_pub.publish(offboard)

        # Trajectory setpoint
        sp = TrajectorySetpoint()
        sp.timestamp = offboard.timestamp
        sp.velocity[0] = vx
        sp.velocity[1] = 0.0
        sp.velocity[2] = 0.0
        sp.yaw = float("nan")
        sp.yawspeed = wz

        self.trajectory_setpoint_pub.publish(sp)

    # ---------------- ARM + OFFBOARD ----------------

    def _try_arm_and_offboard(self):
        if self.offboard_enabled:
            return

        if self.offboard_counter < 20:
            return

        ts = int(self.get_clock().now().nanoseconds / 1000)

        # ARM
        arm = VehicleCommand()
        arm.timestamp = ts
        arm.command = 400
        arm.param1 = 1.0
        arm.target_system = 1
        arm.target_component = 1
        arm.from_external = True
        self.vehicle_command_pub.publish(arm)

        # OFFBOARD
        mode = VehicleCommand()
        mode.timestamp = ts
        mode.command = 176
        mode.param1 = 1.0
        mode.param2 = 6.0
        mode.target_system = 1
        mode.target_component = 1
        mode.from_external = True
        self.vehicle_command_pub.publish(mode)

        self.offboard_enabled = True
        self.get_logger().info("ARMED + OFFBOARD enabled")

    # ---------------- Cleanup ----------------

    def destroy_node(self):
        self._send_disarm()
        super().destroy_node()

    def _send_disarm(self):
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = 400
        cmd.param1 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.from_external = True
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("DISARM sent")


# ============================================================
# Main
# ============================================================


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
