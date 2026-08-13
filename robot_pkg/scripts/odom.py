#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SwerveOdometry(Node):
    """
    Computes robot-frame (Vx, Vy, omega) from 4 independently steered/driven
    swerve wheels using a least-squares fit over all 4 wheels at once, then
    integrates into a world-frame /odom estimate + TF.

    Reads /dynamic_joint_states (control_msgs/DynamicJointState) rather than
    /joint_states. Why: joint_state_broadcaster only publishes a given field
    (position/velocity/effort) in /joint_states as a full array if EVERY
    joint it broadcasts exposes that interface. Our steering joints
    (a1_bl..a4_bl) only need position, our drive joints (w1_a1..w4_a4) only
    need velocity -- a genuinely heterogeneous interface set, matching real
    swerve hardware. /dynamic_joint_states reports each joint's own
    (interface_names, values) independently, so this works with no need to
    add fake interfaces to the URDF just to satisfy /joint_states' uniform
    array requirement.

    Wheel naming/order matches swerve_drive_controller.py exactly:
    FL_wheel_joint=front-left, FR_wheel_joint=front-right, BL_wheel_joint=rear-left, BR_wheel_joint=rear-right.
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

    def __init__(self):
        super().__init__('swerve_odometry')

        self.declare_parameter('wheel_radius', 0.105)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.joint_state_sub = self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self.joint_state_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Least-squares geometry matrix, built once. For each wheel i at
        # body-frame offset (x_i, y_i): vx_i = Vx - omega*y_i,
        # vy_i = Vy + omega*x_i. Stacked over 4 wheels -> 8 eqns, 3 unknowns.
        rows = []
        for _steer_joint, (wx, wy) in self.WHEEL_GEOMETRY.items():
            rows.append([1.0, 0.0, -wy])
            rows.append([0.0, 1.0, wx])
        self.J = np.array(rows)                 # 8x3
        self.J_pinv = np.linalg.pinv(self.J)    # 3x8, precomputed once

    def joint_state_callback(self, msg: DynamicJointState):
        # Per-joint interface lookup: {joint_name: {interface_name: value}}.
        # No array-length assumptions -- each joint only carries whatever
        # interfaces it actually has.
        joint_interfaces = {}
        for joint_name, iv in zip(msg.joint_names, msg.interface_values):
            joint_interfaces[joint_name] = dict(zip(iv.interface_names, iv.values))

        measured = []
        for steer_joint, drive_joint in self.STEER_TO_DRIVE.items():
            steer_ifaces = joint_interfaces.get(steer_joint)
            drive_ifaces = joint_interfaces.get(drive_joint)

            if steer_ifaces is None or drive_ifaces is None:
                self.get_logger().warning(
                    f"'{steer_joint}' or '{drive_joint}' not yet present in "
                    f"/dynamic_joint_states; skipping this update.",
                    throttle_duration_sec=5.0,
                )
                return

            if 'position' not in steer_ifaces or 'velocity' not in drive_ifaces:
                self.get_logger().warning(
                    f"'{steer_joint}' missing 'position' or '{drive_joint}' missing "
                    f"'velocity' in /dynamic_joint_states; skipping this update.",
                    throttle_duration_sec=5.0,
                )
                return

            steer_pos = steer_ifaces['position']
            wheel_ang_vel = drive_ifaces['velocity']

            if math.isnan(steer_pos) or math.isnan(wheel_ang_vel):
                self.get_logger().warning(
                    f"NaN in joint state for {steer_joint}/{drive_joint}; skipping."
                )
                return

            # The assembly joint axis in the URDF is -Z, so the physical
            # steering angle in the body frame is the negation of the raw
            # joint value. No angle folding needed here -- cos/sin of the
            # raw physical angle already recovers the correct (vx, vy)
            # contribution regardless of which of the two equivalent
            # (angle, speed) representations the controller commanded.
            physical_angle = -steer_pos

            wheel_linear_speed = wheel_ang_vel * self.wheel_radius

            measured.append(wheel_linear_speed * math.cos(physical_angle))
            measured.append(wheel_linear_speed * math.sin(physical_angle))

        measured = np.array(measured)  # [vx0, vy0, vx1, vy1, vx2, vy2, vx3, vy3]

        vx, vy, omega = self.J_pinv @ measured

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        dx = (vx * cos_t - vy * sin_t) * dt
        dy = (vx * sin_t + vy * cos_t) * dt
        dtheta = omega * dt

        self.x += dx
        self.y += dy
        self.theta = normalize_angle(self.theta + dtheta)

        self.publish_odometry(vx, vy, omega, now)

    def publish_odometry(self, vx, vy, omega, stamp):
        q = yaw_to_quaternion(self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        self.odom_pub.publish(odom_msg)

        if not self.publish_tf:
            return

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()