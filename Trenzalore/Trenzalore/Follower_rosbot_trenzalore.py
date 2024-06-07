import rclpy
from rclpy.node import Node
import pickle
import socket

class DistanceReceiver(Node):
    """
    A ROS node that acts as a client to receive serialized distance data over TCP from a server.
    """
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

    def receive_and_process(self):
        """
        Continuously receive and process data from the server.
        """
        try:
            while True:
                data = self.sock.recv(4096)  # Receive data from the server
                if data:
                    # Deserialize the received data using pickle
                    distance = pickle.loads(data)
                    # Log the received distance data
                    self.get_logger().info(f"Received distance traveled: {distance}")
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

def main(args=None):
    """
    Main function to initialize the ROS node and handle its lifecycle.
    """
    rclpy.init(args=args)
    node = DistanceReceiver()
    try:
        # Keep the node running and processing data
        node.receive_and_process()
    except KeyboardInterrupt:
        # Handle case where the process is interrupted
        node.get_logger().info('Node shutdown by KeyboardInterrupt')
    finally:
        # Clean up the node resources
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
