import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.initial_position_set = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscription to leader's pose information
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/leader/pose_info',
            self.pose_callback,
            10)

        # Subscription to follower's laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/follower/scan',
            self.scan_callback,
            10)

        self.initial_distance = None
        self.transform_vector = None

    def scan_callback(self, msg):
        if not self.initial_position_set:
            self.initial_distance = msg.ranges[0]
            self.get_logger().info(f'Initial distance: {self.initial_distance}')
            self.set_initial_position()

    def set_initial_position(self):
        try:
            # Create a Pose for the initial distance in front of the base_link
            initial_pose = Pose()
            initial_pose.position.x = self.initial_distance
            initial_pose.position.y = 0.0
            initial_pose.position.z = 0.0
            initial_pose.orientation.w = 1.0

            # Look up the transform from base_link to map
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Transform the initial_pose to map frame
            leader_post = tf2_geometry_msgs.do_transform_pose(initial_pose, transform)
            
            # Store the transformed position as the transform vector
            self.leader_init_pose = leader_post

            self.get_logger().info(f'l;eader Initial position set in map coordinates: {self.leader_init_pose}')
        except TransformException as ex:
            self.get_logger().error(f'Failed to get transform: {str(ex)}')

    def pose_callback(self, msg):
        if self.transform_vector:
            transformed_pose = self.apply_transform_vector(msg)
            self.get_logger().info(f'Transformed Pose: {transformed_pose}')
        else:
            if self.leader_init_pose:
                dx_lf = self.leader_init_pose.position.x - msg.pose.position.x
                dy_lf = self.leader_init_pose.position.y - msg.pose.position.y
                self.transform_vector = (dx_lf, dy_lf)
            else:
                self.get_logger().info('no initial position and no transform vector')

    def apply_transform_vector(self, leader_pose):
        x = leader_pose.pose.position.x + self.transform_vector[0]
        y = leader_pose.pose.position.y + self.transform_vector[1]
        return (x, y)
    
def main(args=None):
    rclpy.init(args=args)
    follower_node = FollowerNode()
    rclpy.spin(follower_node)
    follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
