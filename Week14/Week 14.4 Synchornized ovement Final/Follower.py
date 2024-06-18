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
        '''
        This function initializes the Follower node and sets up a connection to the server.
        '''
        super().__init__('distance_receiver')
        self.sock = None  # Socket for network communication is initially set to None.
        self.setup_connection()  # Call to setup the network connection.
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Setup ROS publisher for Twist messages.

    def setup_connection(self):
        '''
        This function sets up a socket connection to the server at localhost on port 50000.
        If a connection is already open, it is closed and a new connection is attempted.
        '''
        if self.sock:
            self.sock.close()  # Close the existing socket if it's open.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a new socket.
        try:
            self.sock.connect(('localhost', 50000))  # Attempt to connect to the server.
            self.get_logger().info("Connected to the server at localhost:50000")
        except socket.timeout:
            self.get_logger().error("Connection to the server timed out, trying again in 5 seconds...")
            rclpy.get_default_context().call_later(5, self.setup_connection)  # Retry after timeout.

    def receive_and_process(self):
        '''
        This function continuously receives data from the server, processes it, and publishes it.
        It reads a fixed-length header to determine the size of the incoming data, receives all
        bytes of the data, deserializes it, and if it is a Twist message, publishes it.
        '''
        try:
            while rclpy.ok():
                raw_length = self.sock.recv(4)  # Receive the first 4 bytes indicating the data length.
                if raw_length:
                    data_length = int.from_bytes(raw_length, byteorder='big')  # Convert bytes to integer.
                    data = b''
                    while len(data) < data_length:
                        more = self.sock.recv(data_length - len(data))  # Receive remaining data.
                        if not more:
                            raise Exception("Socket connection broken")
                        data += more
                    twist_data = pickle.loads(data)  # Deserialize data into a Twist object.
                    if isinstance(twist_data, Twist):
                        self.get_logger().info(f"Publishing Twist data: {twist_data}")
                        self.publisher.publish(twist_data)  # Publish the Twist data.
                    else:
                        self.get_logger().error("Received data is not of type Twist")
                else:
                    self.get_logger().info("No more data received, closing connection...")
                    break
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            self.sock.close()  # Ensure socket is closed on error or data end.

def main(args=None):
    '''
    This is the main function that initializes the ROS2 node and handles its lifecycle.
    It starts the node, calls the function to receive and process data, and handles shutdown.
    '''
    rclpy.init(args=args)  # Initialize the ROS client library.
    node = Follower()
    try:
        node.receive_and_process()  # Start the receive and process loop.
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown by KeyboardInterrupt')  # Graceful shutdown on interrupt.
    finally:
        node.destroy_node()  # Properly destroy the node.
        rclpy.shutdown()  # Shutdown the ROS client library.

if __name__ == '__main__':
    main()
