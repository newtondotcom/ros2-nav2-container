import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleGpsPosition, VehicleOdometry
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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.gps_sub = self.create_subscription(
            VehicleGpsPosition,
            "/fmu/out/vehicle_gps_position",
            self._gps_callback,
            qos_profile,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._odom_callback,
            qos_profile,
        )

        self.last_message_time: Optional[rclpy.time.Time] = None
        self.warned = False
        self.create_timer(5.0, self._warn_if_inactive)

        self.get_logger().info(
            "PX4 -> ROS bridge started\n"
            "Subscribing to:\n"
            " - /fmu/out/vehicle_gps_position\n"
            " - /fmu/out/vehicle_odometry\n"
            "Publishing:\n"
            " - /gps/fix\n"
            " - /odom"
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
