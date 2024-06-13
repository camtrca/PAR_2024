#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class AngleControl(Node):
    def __init__(self):
        super().__init__('angle_control_node')
        self.subscription = self.create_subscription(
            Odometry,
            '/rosbot_base_controller/odom',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        self.target_point = (1, 0, 0)
        self.max_turning_speed = 0.9  # Maximum turning speed
        self.min_turning_speed = 0.1  # Minimum turning speed

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion to euler angles
        yaw = self.quaternion_to_euler(orientation)

        # Calculate angle to target point
        angle_to_point, direction = self.calculate_angle_and_direction(position, yaw, self.target_point)

        self.get_logger().info(
            f'Position - x: {position.x}, y: {position.y}, z: {position.z}, Orientation w: {orientation.w}, '
            f'Angle to Point: {angle_to_point} degrees, Direction: {direction}')

        # Calculate turning speed based on the angle
        turning_speed = self.calculate_turning_speed(angle_to_point)

        # Publish twist message to turn the robot
        twist = Twist()
        if direction == "left":
            twist.angular.z = turning_speed
        else:
            twist.angular.z = -turning_speed

        # Stop turning if the angle is small enough (within 1 degree tolerance)
        if abs(angle_to_point) < 1:
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

    def quaternion_to_euler(self, orientation):
        """
        Convert a quaternion into euler angles (yaw only, assuming planar movement).
        """
        t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(t3, t4)
        return yaw

    def calculate_angle_and_direction(self, position, yaw, target_point):
        """
        Calculate the angle from the robot's current orientation to the target point.
        """
        dx = target_point[0] - position.x
        dy = target_point[1] - position.y

        angle_to_target = math.atan2(dy, dx)
        angle_relative = angle_to_target - yaw

        # Normalize angle to [-pi, pi]
        while angle_relative > math.pi:
            angle_relative -= 2 * math.pi
        while angle_relative < -math.pi:
            angle_relative += 2 * math.pi

        # Convert angle to degrees
        angle_relative_degrees = math.degrees(angle_relative)

        # Determine direction
        direction = "left" if angle_relative > 0 else "right"

        return angle_relative_degrees, direction

    def calculate_turning_speed(self, angle_to_point):
        """
        Calculate the turning speed based on the angle to the target point.
        """
        # Scale the turning speed based on the angle
        angle_ratio = abs(angle_to_point) / 180.0
        turning_speed = self.min_turning_speed + (self.max_turning_speed - self.min_turning_speed) * angle_ratio

        return turning_speed

def main(args=None):
    rclpy.init(args=args)
    node = AngleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
