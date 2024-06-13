import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import socket
import pickle


class OdometryServer(Node):
    """
    A ROS node that acts as a server to send serialized odometry data to a connected client over TCP.
    """

    def __init__(self):
        """
        Initialize the odometry server node, set up TCP server and subscribe to odometry messages.
        """
        super().__init__('odometry_server')
        # Subscribe to the 'odometry/filtered' topic to receive odometry messages.
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        # Create a TCP/IP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the server address and listen for incoming connections.
        self.sock.bind(('localhost', 50000))
        self.sock.listen(1)
        # Accept a connection.
        self.conn, self.addr = self.sock.accept()
        self.get_logger().info("Server is running and connected to a client.")

    def odom_callback(self, msg):
        """
        Callback function that processes odometry messages and sends the data to a client.
        """
        # Extract position and orientation data from the odometry message.
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        odom_data = {
            "position": {"x": position.x, "y": position.y, "z": position.z},
            "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}
        }
        # Send the structured odometry data to the client.
        self.send_odom_data(odom_data)

    def send_odom_data(self, odom_data):
        """
        Serialize and send the odometry data to the connected client.
        """
        try:
            # Serialize the dictionary using pickle.
            serialized_data = pickle.dumps(odom_data)
            # Send the serialized data through the socket.
            self.conn.sendall(serialized_data)
            self.get_logger().info(f"Sent odometry data: {odom_data}")
        except Exception as e:
            # Log and handle exceptions.
            self.get_logger().error(f"Failed to send data: {e}")
            # Close the current connection and accept a new one in case of failure.
            self.conn.close()
            self.conn, self.addr = self.sock.accept()
            self.get_logger().info("Reconnected to the client.")

    def destroy_node(self):
        """
        Clean up the node resources by closing the socket connections before shutting down.
        """
        super().destroy_node()
        self.conn.close()
        self.sock.close()


def main(args=None):
    """
    Entry point for the ROS node.
    """
    rclpy.init(args=args)
    node = OdometryServer()
    try:
        rclpy.spin(node)
    finally:
        # Ensure proper resource cleanup.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
