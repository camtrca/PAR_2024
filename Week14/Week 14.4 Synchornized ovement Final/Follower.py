import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pickle
import socket
import select

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10.0)
        try:
            self.sock.connect(('localhost', 50000))
            self.get_logger().info("Connected to the server at localhost:50000")
        except socket.timeout:
            self.get_logger().error("Connection to the server timed out")
            return
        
        self.last_position = None
        self.total_distance = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
