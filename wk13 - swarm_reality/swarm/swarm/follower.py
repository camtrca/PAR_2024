import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import pickle
import socket
from visualization_msgs.msg import Marker

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.initial_position_set = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # self.pose_subscription = self.create_subscription(
        #     PoseStamped,
        #     '/leader/pose_info',
        #     self.pose_callback,
        #     10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.initial_distance = None
        self.transform_vector = None
        self.current_goal = None
        self.leader_positions = []
        self.check_interval = 5  # time interval for checking leader's position
        self.distance_threshold = 1.0  # distance away from the leader
        self.reviece_msg_timer = 1.0 # timer for tcp

        self.navigation_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # timer for check position
        self.timer = self.create_timer(self.check_interval, self.check_and_set_goal)
        # timer for receive position from leader
        self.timer = self.create_timer(self.reviece_msg_timer, self.receive_and_process)

        # Publisher for RViz visualization
        self.marker_publisher = self.create_publisher(Marker, '/leader/pose_marker', 10)

        # Create a TCP/IP socket for receiving messages
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server on the leader robot, assumed to be running on localhost port 50000
        self.sock.connect(('localhost', 50000))
        self.get_logger().info("Connected to the server at localhost:50000")

    def scan_callback(self, msg):
        if not self.initial_distance:
            self.initial_distance = 0.5
            self.get_logger().info(f'Initial distance: {self.initial_distance}')
            self.set_initial_position()

    def set_initial_position(self):
        try:
            initial_pose = Pose()
            initial_pose.position.x = self.initial_distance
            initial_pose.position.y = 0.0
            initial_pose.position.z = 0.0
            initial_pose.orientation.w = 1.0

            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            leader_post = tf2_geometry_msgs.do_transform_pose(initial_pose, transform)
            self.leader_init_pose = leader_post
            self.initial_position_set = True

            self.get_logger().info(f'Leader initial position set in map coordinates: {self.leader_init_pose}')
        except TransformException as ex:
            self.get_logger().error(f'Failed to get transform: {str(ex)}')

    def pose_callback(self, msg):
        if not self.initial_position_set:
            return
        if not self.transform_vector:
            lx = msg.pose.position.x
            ly = msg.pose.position.y
            dx = self.leader_init_pose.position.x - lx
            dy = self.leader_init_pose.position.y - ly
            self.transform_vector = (dx, dy)
        transformed_pose = self.apply_transform_vector(msg)
        self.leader_positions.append(transformed_pose)
        
        # Publish the transformed pose for RViz visualization
        self.publish_leader_pose(transformed_pose)

    def apply_transform_vector(self, leader_pose):
        x = leader_pose.pose.position.x + self.transform_vector[0] if self.transform_vector else leader_pose.pose.position.x
        y = leader_pose.pose.position.y + self.transform_vector[1] if self.transform_vector else leader_pose.pose.position.y
        self.get_logger().info(f'translate to ({x}, {y})')
        return (x, y)

    def publish_leader_pose(self, pose):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'leader'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_publisher.publish(marker)

    def check_and_set_goal(self):
        if not self.leader_positions:
            return
        
        if self.current_goal:
            return
        
        return # make sure it won't move
    
        current_pose = self.leader_positions[-1]
        for pose in reversed(self.leader_positions):
            distance = ((current_pose[0] - pose[0]) ** 2 + (current_pose[1] - pose[1]) ** 2) ** 0.5
            if distance > self.distance_threshold:
                self.set_goal(pose)
                self.current_goal = pose
                break

    def set_goal(self, goal_pose):
        if not self.navigation_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available')
            self.current_goal = None
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal_pose[0]
        goal_msg.pose.pose.position.y = goal_pose[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_goal_future = self.navigation_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal = None
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.leader_positions = self.leader_positions[self.leader_positions.index(self.current_goal):]
            self.current_goal = None
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            self.current_goal = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def receive_and_process(self):
        """
        Continuously receive and process data from the server.
        """
        try:
            while True:
                data = self.sock.recv(4096)  # Receive data from the server
                if data:
                    # Deserialize the received data using pickle
                    pose = pickle.loads(data)
                    self.pose_callback(pose) # act like subscriber
                    # Log the received distance data
                    # self.get_logger().info(f"Received data:")
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
    rclpy.init(args=args)
    follower_node = FollowerNode()
    rclpy.spin(follower_node)
    follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
