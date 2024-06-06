import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import pickle
import select

class OdometryServer(Node):
    def __init__(self):
        super().__init__('odometry_server')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('localhost', 50000))
        self.sock.listen(1)
        self.sock.setblocking(False)
        self.conn = None
        self.addr = None
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

    def odom_callback(self, msg):
        if not self.conn:
            self.try_accept_connection()
        if self.conn:
            try:
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                odom_data = {
                    "position": {"x": position.x, "y": position.y, "z": position.z},
                    "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}
                }
                serialized_data = pickle.dumps(odom_data)
                # Prefix each message with its length
                data_length = len(serialized_data).to_bytes(4, byteorder='big')
                self.conn.sendall(data_length + serialized_data)
                self.get_logger().info(f"Sent odometry data: {odom_data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send data: {e}")
                self.cleanup_connection()

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
