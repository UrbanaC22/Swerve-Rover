#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SwerveDriveController(Node):
    """
    Converts /cmd_vel (Vx, Vy, omega) into per-wheel steering angle +
    drive velocity commands for a 4-wheel independently-steered swerve
    drive.

    FL_wheel_joint=front-left, FR_wheel_joint=front-right, BL_wheel_joint=rear-left, BR_wheel_joint=rear-right. 
    Keep both files in sync -- if geometry or joint order changes in one, it must
    change in the other.

    IMPORTANT: JOINT_ORDER below is only correct if controller.yaml lists
    the joints for swerve_steering_controller / swerve_velocity_controller
    in that same order. Verify against your actual controller.yaml.
    """

    WHEEL_GEOMETRY = {
        'FL_wheel_joint': (0.445, -0.292),   # front-left
        'FR_wheel_joint': (0.445, 0.292),    # front-right
        'BL_wheel_joint': (-0.445, 0.292),   # rear-left
        'BR_wheel_joint': (-0.445, -0.292),  # rear-right
    }

    STEER_TO_DRIVE = {
        'FL_mount_joint': 'FL_wheel_joint',
        'FR_mount_joint': 'FR_wheel_joint',
        'BL_mount_joint': 'BL_wheel_joint',
        'BR_mount_joint': 'BR_wheel_joint',
    }

    JOINT_ORDER = ['FL_wheel_joint', 'FR_wheel_joint', 'BL_wheel_joint', 'BR_wheel_joint']

    WHEEL_RADIUS = 0.105  

    def __init__(self):
        super().__init__('swerve_drive_controller')

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/swerve_steering_controller/commands', 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/swerve_velocity_controller/commands', 10)

    @staticmethod
    def constrain_to_90(angle: float, speed: float):
        """
        Fold a steering angle into [-pi/2, +pi/2]. If the desired angle
        lies outside that range, rotate it by pi and reverse the drive
        speed instead -- identical resulting velocity vector, but the
        wheel never has to turn more than 90 deg from rest.
        """
        angle = normalize_angle(angle)
        if angle > math.pi / 2.0:
            angle -= math.pi
            speed = -speed
        elif angle < -math.pi / 2.0:
            angle += math.pi
            speed = -speed
        return angle, speed

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        steering_cmds = []
        velocity_cmds = []

        for joint in self.JOINT_ORDER:
            wx, wy = self.WHEEL_GEOMETRY[joint]

            # Standard swerve forward kinematics for a wheel at body-frame
            # offset (wx, wy): v_wheel = (Vx - omega*wy, Vy + omega*wx).
            v_x = vx - omega * wy
            v_y = vy + omega * wx

            speed = math.hypot(v_x, v_y)          # linear speed, m/s
            angle = math.atan2(v_y, v_x)          # desired physical angle

            angle, speed = self.constrain_to_90(angle, speed)

            # Axis-sign consistency with swerve_odometry.py: assembly
            # joint's rotation axis is -Z, so physical = -raw_joint =>
            # raw_joint = -physical.
            raw_joint_angle = -angle

            steering_cmds.append(raw_joint_angle)
            velocity_cmds.append(speed / self.WHEEL_RADIUS)  # m/s -> rad/s

        self.publish_commands(steering_cmds, velocity_cmds)

    def publish_commands(self, steering_cmds, velocity_cmds):
        steering_msg = Float64MultiArray()
        steering_msg.data = steering_cmds
        self.steering_pub.publish(steering_msg)

        velocity_msg = Float64MultiArray()
        velocity_msg.data = velocity_cmds
        self.velocity_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()