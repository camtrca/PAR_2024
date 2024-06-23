import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pickle
import select
import socket

class Leader(Node):
    def __init__(self):
        '''
        This function initializes the Leader node, sets up a TCP server to listen for connections,
        and subscribes to Twist messages on the '/cmd_vel' topic.
        '''
        super().__init__('leader')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)  # Subscription to Twist messages.
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a TCP socket.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Set socket options.
        self.sock.bind(('localhost', 50000))  # Bind the socket to address and port.
        self.sock.listen(1)  # Listen for incoming connections.
        self.sock.setblocking(False)  # Set socket to non-blocking mode.
        self.conn = None  # Connection variable initialized.
        self.addr = None  # Address variable initialized.
        self.get_logger().info("Server is running, waiting for connections.")

    def try_accept_connection(self):
        '''
        This function attempts to accept a new connection from a client if available.
        '''
        try:
            ready, _, _ = select.select([self.sock], [], [], 0.1)  # Non-blocking select to check for readiness.
            if ready:
                self.conn, self.addr = self.sock.accept()  # Accept the new connection.
                self.conn.setblocking(True)  # Set the connection to blocking mode.
                self.get_logger().info("Connected to a client.")
        except Exception as e:
            self.get_logger().error(f"Error accepting connections: {e}")

    def listener_callback(self, msgs):
        '''
        This function is called whenever a Twist message is received. It attempts to send the message
        over an established TCP connection.
        '''
        if not self.conn:
            self.try_accept_connection()  # Try accepting a connection if not already connected.
        else:
            try:
                serialized_data = pickle.dumps(msgs)  # Serialize the Twist message.
                data_length = len(serialized_data).to_bytes(4, byteorder='big')  # Prepare length prefix.
                self.conn.sendall(data_length + serialized_data)  # Send length-prefixed data.
                self.get_logger().info(f"Sent data successfully, data: {serialized_data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send data: {e}")
                self.cleanup_connection()  # Clean up connection on failure.

    def cleanup_connection(self):
        '''
        This function cleans up the TCP connection, closing it and resetting the connection variable.
        '''
        if self.conn:
            self.conn.close()  # Close the connection.
            self.conn = None  # Reset the connection variable.
            self.get_logger().info("Connection closed and ready for a new client.")

def main(args=None):
    '''
    The main function initializes the ROS node and handles its lifecycle.
    It spins the node to handle callbacks and manages graceful shutdown.
    '''
    rclpy.init(args=args)  # Initialize the ROS client library.
    node = Leader()  # Create the Leader node.
    try:
        rclpy.spin(node)  # Spin the node to process callbacks.
    except KeyboardInterrupt:
        pass  # Handle KeyboardInterrupt (Ctrl+C).
    finally:
        node.destroy_node()  # Properly destroy the node.
        rclpy.shutdown()  # Shutdown the ROS client library.

if __name__ == '__main__':
    main()
