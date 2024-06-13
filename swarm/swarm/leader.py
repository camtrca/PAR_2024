import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformException
import socket
import pickle

class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')
        # Publisher to send pose information
        self.publisher_ = self.create_publisher(PoseStamped, '/leader/pose_info', 10)
        self.time_step = 0.1 # frequency of publishing map pose
        self.timer = self.create_timer(self.time_step, self.publish_pose)

        # Initialize tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a TCP/IP socket.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bind the socket to the server address and listen for incoming connections.
        self.sock.bind(('localhost', 50000))
        self.sock.listen(1)
        # Accept a connection.
        self.conn, self.addr = self.sock.accept()
        self.get_logger().info("Server is running and connected to a client.")


    def transform_bot_pose(self, dest='map', src='base_link'):
        try:
            transform = self.tf_buffer.lookup_transform(dest, src, rclpy.time.Time())
            current_pose = Pose()
            current_pose.position.x = transform.transform.translation.x
            current_pose.position.y = transform.transform.translation.y
            current_pose.position.z = transform.transform.translation.z
            current_pose.orientation = transform.transform.rotation
            current_poseS = PoseStamped()
            current_poseS.header.frame_id = dest
            current_poseS.header.stamp = self.get_clock().now().to_msg()
            current_poseS.pose.position = current_pose.position
            current_poseS.pose.orientation = current_pose.orientation
            return current_poseS
        except TransformException as ex:
            self.get_logger().error(f'Failed to get or apply transform: {str(ex)}')
            return None

    def publish_pose(self):
        pose = self.transform_bot_pose()
        # if pose:
            # self.publisher_.publish(pose)
        if pose:
            self.send_pose_data(pose)

    def send_pose_data(self, pose):
        """
        Serialize and send the odometry data to the connected client.
        """
        try:
            # Serialize the dictionary using pickle.
            serialized_data = pickle.dumps(pose)
            # Send the serialized data through the socket.
            self.conn.sendall(serialized_data)
            self.get_logger().info(f"Sent pose data: {pose}")
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
    rclpy.init(args=args)
    leader_node = LeaderNode()
    rclpy.spin(leader_node)
    leader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
