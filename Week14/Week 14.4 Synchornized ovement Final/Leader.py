import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pickle
import select
import socket

class Leader(Node):
    def __init__(self):
        super().__init__('leader')
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
    
    def cleanup_connection(self):
        if self.conn:
            self.conn.close()
            self.conn = None
            self.get_logger().info("Connection closed and ready for a new client.")

def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()