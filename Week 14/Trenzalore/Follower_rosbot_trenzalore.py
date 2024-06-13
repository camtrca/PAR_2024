import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pickle
import socket
import math
import tf2_ros


class DistanceReceiver(Node):
    def __init__(self):
        """
        Initialize the node and set up the TCP client connection to the server.
        """
        super().__init__('distance_receiver')
        # Create a TCP/IP socket for receiving messages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server on the leader robot, assumed to be running on localhost port 50000
        self.sock.connect(('localhost', 50000))
        self.get_logger().info("Connected to the server at localhost:50000")

        # Declare parameters with default values x, y, z to 0
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

    def receive_and_process(self):
        try:
            while True:
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
        self.get_logger().info(
            f"Received distance traveled: {self.last_known_pos}")
        angle_to_leader = math.atan2(
            self.last_known_pos['y'], self.last_known_pos['x'])


def main(args=None):
    rclpy.init(args=args)
    node = DistanceReceiver()
    try:
        node.receive_and_process()
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown by KeyboardInterrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
