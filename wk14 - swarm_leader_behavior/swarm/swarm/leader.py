import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math
import time
import socket
import pickle

class Leader(Node):
    def __init__(self, square_size, shape_type):
        '''
        Initializes the Leader node with specific square size and shape type for movement patterns.
        Sets up publishers, subscriptions, and action clients for navigation.
        '''
        super().__init__('leader')
        self.square_size = square_size  # Size of the movement square.
        self.shape_type = shape_type  # Type of shape to move in (square, circle, map_square).
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # Publisher for velocity commands.
        self.get_logger().info(f"Square size set to: {self.square_size}")
        self.get_logger().info(f"Shape type set to: {self.shape_type}")
        self.duration_multi = 1
        self.current_goal = False
        self.reach_goal = False
        
        self.current_position = (0.0, 0.0)  # Initializes current position.
        
        # Subscribing to the map topic.
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            10)
        
        # Subscribing to odometry messages.
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.yaw = 0.0  # Initial yaw (orientation).
        self.initial_yaw = None  # To store initial yaw when starting to rotate.
        self.target_yaw = None  # Target yaw to achieve during rotation.
        self.is_rotating = False  # Flag to check if rotation is ongoing.
        self.tolerance = math.radians(2)  # Tolerance for rotation in radians (about 2 degrees).
        
        # Action client to navigate to a pose.
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def odom_callback(self, msg):
        '''
        Callback for odometry messages to update robot orientation.
        '''
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)  # Convert quaternion to yaw.

        # Set initial and target yaws for rotation.
        if self.initial_yaw is None:
            self.initial_yaw = self.yaw
            self.target_yaw = self.normalize_angle(self.initial_yaw - math.pi / 2)
        
        # Execute rotation if flagged.
        if self.is_rotating:
            self.rotate_bot()
 
    def normalize_angle(self, angle):
        '''
        Normalizes an angle to the range [-pi, pi].
        '''
        return math.atan2(math.sin(angle), math.cos(angle))
 
    def rotate_bot(self):
        '''
        Performs the rotation by publishing angular velocities based on the yaw error.
        '''
        error = self.normalize_angle(self.target_yaw - self.yaw)
        if abs(error) > self.tolerance:
            twist = Twist()
            twist.angular.z = 0.5 if error > 0 else -0.5
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            self.publisher_.publish(twist)
            self.is_rotating = False
            self.initial_yaw = None
            self.get_logger().info('Rotation complete')
 
    def start_rotation(self):
        '''
        Starts the rotation process by setting the is_rotating flag.
        '''
        self.is_rotating = True
        self.get_logger().info('Starting rotation')

    def send_odom_data(self, odom_data):
        '''
        Serializes and sends odometry data over a network connection.
        '''
        try:
            serialized_data = pickle.dumps(odom_data)
            self.conn.sendall(serialized_data)
            self.get_logger().info(f"Sent odometry data: {odom_data}")
        except Exception as e:
            self.get_logger().error(f"Failed to send data: {e}")
            self.conn.close()
            self.conn, self.addr = self.sock.accept()
            self.get_logger().info("Reconnected to the client.")

    def map_callback(self, msg):
        '''
        Callback for receiving map data.
        '''
        self.get_logger().info("Map received")

    def run(self):
        '''
        Executes movement based on the configured shape type.
        '''
        if self.shape_type == 'square':
            self.move_in_square()
        elif self.shape_type == 'circle':
            self.move_in_circle()
        elif self.shape_type == 'map_square':
            self.move_in_map_square()
        else:
            self.get_logger().error(f"Unknown shape type: {self.shape_type}")

    def move_in_square(self):
        '''
        Moves the robot in a square pattern.
        '''
        self.get_logger().info("Moving in a square.")
        for _ in range(4):
            self.move_forward(self.square_size)
            self.turn(90)

    def move_in_circle(self):
        '''
        Moves the robot in a circular pattern.
        '''
        self.get_logger().info("Moving in a circle.")
        radius = self.square_size / 2
        circumference = 2 * math.pi * radius
        steps = 36  # Number of steps to approximate a circle
        step_length = circumference / steps
        step_angle = 360 / steps
        for _ in range(steps):
            self.move_forward(step_length)
            self.turn(step_angle)

    def move_in_map_square(self):
        '''
        Moves the robot in a square pattern using map coordinates.
        '''
        if self.current_position is None:
            self.get_logger().error("Current position not available. Ensure odometry data is being received.")
            return

        self.get_logger().info("Moving in a square based on map points.")
        x, y = self.current_position
        points = [
            (x, y),
            (x, y + self.square_size),
            (x + self.square_size, y + self.square_size),
            (x + self.square_size, y),
            (x, y)
        ]

        for point in points:
            self.reach_goal = False
            while not self.reach_goal:
                self.send_goal(point[0], point[1])
                self.wait_for_result()

    def send_goal(self, x, y):
        '''
        Sends a navigation goal to the action server and waits for a response.
        '''
        self.get_logger().info(f'Sending goal to ({x}, {y})')
        self.current_goal = True
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = 0.0
        goal_pose.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Handles the response from the navigation goal.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.current_goal = False
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Processes the final result of the navigation goal.
        '''
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            self.current_goal = False
            self.reach_goal = True
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            self.current_goal=False

    def feedback_callback(self, feedback_msg):
        '''
        Receives feedback during navigation.
        '''
        feedback = feedback_msg.feedback

    def wait_for_result(self):
        '''
        Waits for the result of the navigation goal, checking periodically.
        '''
        while self.current_goal:
            rclpy.spin_once(self, timeout_sec=1.0)

    def move_forward(self, distance):
        '''
        Moves the robot forward a specified distance.
        '''
        self.get_logger().info(f"Moving forward {distance} units.")
        twist = Twist()
        twist.linear.x = 0.5
        duration = distance / twist.linear.x
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration*self.duration_multi)
        while self.get_clock().now() < end_time:
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

    def turn(self, angle, method='odom'):
        '''
        Turns the robot by a specified angle using either odometry or an angular velocity command.
        '''
        if method == 'agl':
            self.get_logger().info(f"Turning {angle} degrees.")
            twist = Twist()
            twist.angular.z = math.radians(90)
            duration = math.radians(angle) / twist.angular.z
            end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration*self.duration_multi)
            while self.get_clock().now() < end_time:
                self.publisher_.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.1)

            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(1)
        elif method == 'odom':
            self.start_rotation()
            while self.is_rotating:
                rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    '''
    Main function to initialize and run the ROS node.
    '''
    rclpy.init(args=args)  # Initialize ROS2 client library.

    size = 0
    shape = ''

    while size > 4 or size < 1:
        try:
            size = int(input("Please select a size between 1 and 4: "))
        except ValueError:
            print("Invalid input. Please enter an integer between 1 and 4.")

    while shape not in ['circle', 'square', 'map_square']:
        shape = input("Please select a shape ('square', 'map_square'): ").strip().lower()
        if shape not in ['circle', 'square', 'map_square']:
            print("Invalid input. Please enter 'circle', 'square' or 'map_square'.")

    leader = Leader(size, shape)  # Create a Leader node instance with selected size and shape.

    try:
        leader.run()  # Run the movement patterns based on the selected shape.
    except KeyboardInterrupt:
        pass  # Handle keyboard interrupt gracefully.

    leader.destroy_node()  # Properly destroy the ROS node.
    rclpy.shutdown()  # Shutdown the ROS2 client library.

if __name__ == '__main__':
    main()
