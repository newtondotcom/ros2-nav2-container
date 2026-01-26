import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from px4_msgs.msg import (
    OffboardControlMode,
    RoverSpeedSetpoint,
    RoverSteeringSetpoint,
    VehicleCommand,
)

WHEELBASE = 0.6  # meters → adjust to your rover

class TwistToPX4Rover(Node):

    def __init__(self):
        super().__init__("twist_to_px4_rover")

        self.cmd_sub = self.create_subscription(
            TwistStamped, "/cmd_vel", self.cmd_cb, 10
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )

        self.speed_pub = self.create_publisher(
            RoverSpeedSetpoint, "/fmu/in/rover_speed_setpoint", 10
        )

        self.steer_pub = self.create_publisher(
            RoverSteeringSetpoint, "/fmu/in/rover_steering_setpoint", 10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.last_twist = None
        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.counter = 0

    # ------------------------

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def cmd_cb(self, msg):
        self.last_twist = msg

    # ------------------------

    def arm_and_offboard(self):
        ts = self.now_us()

        arm = VehicleCommand()
        arm.timestamp = ts
        arm.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm.param1 = 1.0
        arm.target_system = 1
        arm.target_component = 1
        self.cmd_pub.publish(arm)

        mode = VehicleCommand()
        mode.timestamp = ts
        mode.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        mode.param1 = 1.0
        mode.param2 = 6.0  # OFFBOARD
        mode.target_system = 1
        mode.target_component = 1
        self.cmd_pub.publish(mode)

    # ------------------------

    def timer_cb(self):

        if self.last_twist is None:
            return

        ts = self.now_us()

        # ---- Offboard mode ----
        off = OffboardControlMode()
        off.timestamp = ts
        off.rover_speed = True
        off.rover_steering = True
        self.offboard_pub.publish(off)

        v = self.last_twist.twist.linear.x
        w = self.last_twist.twist.angular.z

        # ---- Speed setpoint ----
        spd = RoverSpeedSetpoint()
        spd.timestamp = ts
        spd.speed = float(v)
        self.speed_pub.publish(spd)

        # ---- Steering setpoint ----
        steer = RoverSteeringSetpoint()
        steer.timestamp = ts

        if abs(v) < 0.05:
            steer.steering = 0.0
        else:
            delta = math.atan(w * WHEELBASE / v)
            steer.steering = float(delta)

        self.steer_pub.publish(steer)

        # ---- enable offboard ----
        if self.counter == 20:
            self.arm_and_offboard()
        self.counter += 1


def main():
    rclpy.init()
    node = TwistToPX4Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
