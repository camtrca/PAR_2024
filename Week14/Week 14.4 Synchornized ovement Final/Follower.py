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
        super().__init__('distance_receiver')
        self.sock = None
        self.setup_connection()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def setup_connection(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('localhost', 50000))
            self.get_logger().info("Connected to the server at localhost:50000")
        except socket.timeout:
            self.get_logger().error("Connection to the server timed out, trying again in 5 seconds...")
            rclpy.get_default_context().call_later(5, self.setup_connection)

    def receive_and_process(self):
        try:
            while rclpy.ok():
                # Read the length of the data first
                raw_length = self.sock.recv(4)
                if raw_length:
                    data_length = int.from_bytes(raw_length, byteorder='big')
                    data = b''
                    while len(data) < data_length:
                        more = self.sock.recv(data_length - len(data))
                        if not more:
                            raise Exception("Socket connection broken")
                        data += more
                    twist_data = pickle.loads(data)
                    if isinstance(twist_data, Twist):
                        self.get_logger().info(f"Publishing Twist data: {twist_data}")
                        self.publisher.publish(twist_data)
                    else:
                        self.get_logger().error("Received data is not of type Twist")

                else:
                    self.get_logger().info("No more data received, closing connection...")
                    break
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = Follower()
    try:
        node.receive_and_process()
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown by KeyboardInterrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
