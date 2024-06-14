#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import socket
import pickle
import threading


class DistanceReceiver(Node):
    def __init__(self):
        super().__init__('distance_receiver_node')
        # Create a TCP/IP socket for receiving messages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server on the leader robot, assumed to be running on localhost port 50000
        self.sock.connect(('localhost', 50000))
        self.get_logger().info("Connected to the server at localhost:50004")

        # Declare parameters with default values
        self.declare_parameter('leader_x', 0.0)
        self.declare_parameter('leader_y', 0.0)
        self.declare_parameter('leader_z', 0.0)

        self.initial_pos = {
            'x': self.get_parameter('leader_x').get_parameter_value().double_value,
            'y': self.get_parameter('leader_y').get_parameter_value().double_value,
            'z': self.get_parameter('leader_z').get_parameter_value().double_value
        }
        self.last_known_pos = None
        self.total_distance = 0.0

        # Angle control components
        self.subscription = self.create_subscription(
            Odometry,
            '/rosbot_base_controller/odom',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.max_turning_speed = 0.6  # Maximum turning speed
        self.min_turning_speed = 0.05  # Minimum turning speed
        self.safe_distance = 0.5
        self.moving_speed = 0.1

        # Start the thread to receive and process socket data
        threading.Thread(target=self.receive_and_process, daemon=True).start()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(f"Current Position: x: {
            position.x}, y: {position.y}, z: {position.z}")

        if self.last_known_pos is None:
            return

        orientation = msg.pose.pose.orientation

        # Convert quaternion to euler angles
        yaw = self.quaternion_to_euler(orientation)

        # Calculate angle to target point
        angle_to_point, direction = self.calculate_angle_and_direction(
            position, yaw, self.last_known_pos)

        # Calculate the distance to the target point
        distance = self.calculate_distance(position, self.last_known_pos)

        self.get_logger().info(
            f'Position - x: {position.x}, y: {position.y}, z: {
                position.z}, Orientation w: {orientation.w}, '
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
        if abs(angle_to_point) < 0.5:
            twist.angular.z = 0.0

        # Move the robot forward
        if distance > self.safe_distance:
            twist.linear.x = self.moving_speed
            self.get_logger().info("Go go go")
        else:
            twist.linear.x = 0.0

        self.publisher_.publish(twist)

    def quaternion_to_euler(self, orientation):
        """Convert a quaternion into euler angles (yaw only, assuming planar movement)."""
        t3 = 2.0 * (orientation.w * orientation.z +
                    orientation.x * orientation.y)
        t4 = 1.0 - 2.0 * (orientation.y * orientation.y +
                          orientation.z * orientation.z)
        return math.atan2(t3, t4)

    def calculate_angle_and_direction(self, position, yaw, target_point):
        """Calculate the angle and direction to the target point."""
        delta_x = target_point['x'] - position.x
        delta_y = target_point['y'] - position.y
        angle_to_point = math.degrees(math.atan2(delta_y, delta_x))
        angle_difference = angle_to_point - math.degrees(yaw)

        if angle_difference > 180:
            angle_difference -= 360
        elif angle_difference < -180:
            angle_difference += 360

        direction = "left" if angle_difference > 0 else "right"
        return abs(angle_difference), direction

    def calculate_turning_speed(self, angle):
        """Calculate turning speed based on the angle to the target point."""
        if angle > 10:
            return self.max_turning_speed
        else:
            return self.min_turning_speed + (self.max_turning_speed - self.min_turning_speed) * (angle / 10)

    def calculate_distance(self, follower_pos, target_point):
        """Calculate the distance to the target point."""
        distance = math.sqrt(
            (target_point['x'] - follower_pos.x) ** 2 +
            (target_point['y'] - follower_pos.y) ** 2
        )
        return distance

    def receive_and_process(self):
        try:
            while True:
                self.get_logger().info("Start----------------------------------")
                data = self.sock.recv(4096)  # Receive data from the server
                if data:
                    # Deserialize the received data using pickle
                    distance = pickle.loads(data)
                    # Log the received distance data
                    self.get_leader_pos(distance['position'])
                else:
                    # If no data is received, log and close the connection
                    self.get_logger().info("No more data received, closing connection...")
                    break
        except Exception as e:
            # Log any exceptions that occur during reception and processing
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            # Ensure the socket is closed when data reception is done
            self.sock.close()

    def get_leader_pos(self, leader_pos):
        # Extract the leader position and update the current position
        relative_pos = {
            'x': self.initial_pos['x'] + leader_pos['x'],
            'y': self.initial_pos['y'] + leader_pos['y'],
            'z': self.initial_pos['z'] + leader_pos['z']
        }

        # Update the last known position
        self.last_known_pos = relative_pos


def main(args=None):
    rclpy.init(args=args)
    node = DistanceReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown by KeyboardInterrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
