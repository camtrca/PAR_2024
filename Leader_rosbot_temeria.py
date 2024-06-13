import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import socket
import pickle
import select
import tf2_ros
import math
from rclpy.duration import Duration

class OdometryServer(Node):

    def __init__(self):
        super().__init__('odometry_server')
        self.total_distance = 0.0
        self.last_movement_data = None
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 50000))
        self.sock.listen(1)
        self.sock.setblocking(False)
        self.conn = None
        self.addr = None
        self.MIN_MOVEMENT_DIST = 0.01  # meters
        self.MIN_ROTATION_ANGLE = 0.01  # radians
        self.last_position = None
        self.last_orientation = None
        self.last_twist = Twist()
        self.timer = self.create_timer(0.3, self.timer_callback)
        self.get_logger().info("Server is running, waiting for connections.")

    def try_accept_connection(self):
        try:
            ready, _, _ = select.select([self.sock], [], [], 0.1)
            if ready:
                self.conn, self.addr = self.sock.accept()
                self.conn.setblocking(True)
                self.get_logger().info("Connected to a client.")
        except Exception as e:
            self.get_logger().error(f"Error accepting connections: {e}")

    def calculate_movement(self, current_position, current_orientation):
        if not self.last_position or not self.last_orientation:
            self.last_position = current_position
            self.last_orientation = current_orientation
            return None

        movement_distance = math.sqrt(
            (current_position.x - self.last_position.x) ** 2 +
            (current_position.y - self.last_position.y) ** 2 +
            (current_position.z - self.last_position.z) ** 2
        )

        angular_difference = math.sqrt(
            (current_orientation.x - self.last_orientation.x) ** 2 +
            (current_orientation.y - self.last_orientation.y) ** 2 +
            (current_orientation.z - self.last_orientation.z) ** 2 +
            (current_orientation.w - self.last_orientation.w) ** 2
        )

        if movement_distance >= self.MIN_MOVEMENT_DIST or angular_difference >= self.MIN_ROTATION_ANGLE:
            self.total_distance += movement_distance
            self.last_position = current_position
            self.last_orientation = current_orientation
            return {
                'message': f"Moved {movement_distance:.2f} meters; Rotated {angular_difference:.2f} radians.",
                'distance': movement_distance,
                'rotation': angular_difference,
                'total_distance': self.total_distance,
                'twist': {
                    'linear': {
                        'x': self.last_twist.linear.x,
                        'y': self.last_twist.linear.y,
                        'z': self.last_twist.linear.z
                    },
                    'angular': {
                        'x': self.last_twist.angular.x,
                        'y': self.last_twist.angular.y,
                        'z': self.last_twist.angular.z
                    }
                }
            }
        return None

    def odom_callback(self, msg):
        if not self.conn:
            self.try_accept_connection()
        if self.conn:
            try:
                trans = self.tf_buffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
                current_position = trans.transform.translation
                current_orientation = trans.transform.rotation
                movement_data = self.calculate_movement(current_position, current_orientation)
                if movement_data:
                    self.send_movement_data(movement_data)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"TF error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                self.cleanup_connection()

    def cmd_vel_callback(self, msg):
        self.last_twist = msg

    def send_error_message(self):
        error_message = "Transformation data unavailable. Check TF publishers or network conditions."
        try:
            if self.conn:
                serialized_message = pickle.dumps(error_message)
                message_length = len(serialized_message).to_bytes(4, byteorder='big')
                self.conn.sendall(message_length + serialized_message)
                self.get_logger().info("Sent error message to follower.")
        except BrokenPipeError:
            self.get_logger().error("Connection broken while sending error message. Attempting to reconnect...")
            self.cleanup_connection()
            self.try_accept_connection()

    def cleanup_connection(self):
        if self.conn:
            self.conn.close()
            self.conn = None
            self.get_logger().info("Connection closed and ready for a new client.")

    def destroy_node(self):
        if self.conn:
            self.conn.close()
        self.sock.close()
        super().destroy_node()

    def send_movement_data(self, movement_data):
        try:
            if isinstance(movement_data, dict):
                serialized_message = pickle.dumps(movement_data)
                message_length = len(serialized_message).to_bytes(4, byteorder='big')
                self.conn.sendall(message_length + serialized_message)
                self.get_logger().info(f"Sent movement description: {movement_data}")
            else:
                self.get_logger().error(f"Expected a dict, but got {type(movement_data)}: {movement_data}")
        except Exception as e:
            self.get_logger().error("Failed to send data: {}".format(e))
            self.cleanup_connection()

    def timer_callback(self):
        if not self.conn:
            return  # No connection, no operation.
        self.get_logger().info("Checking for significant movement...")
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
            current_position = trans.transform.translation
            current_orientation = trans.transform.rotation
            if self.last_position and self.last_orientation:
                movement_data = self.calculate_movement(current_position, current_orientation)
                if movement_data:
                    self.send_movement_data(movement_data)
                    self.last_position = current_position  # Update last positions only if sent
                    self.last_orientation = current_orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            self.cleanup_connection()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
