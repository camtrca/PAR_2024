import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, TransformException


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
        if pose:
            self.publisher_.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    leader_node = LeaderNode()
    rclpy.spin(leader_node)
    leader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
