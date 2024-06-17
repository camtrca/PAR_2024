import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')
        # Publisher to send velocity information
        self.publisher_ = self.create_publisher(Twist, '/leader/vel_info', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/leader/rosbot_base_controller/odom',
            self.odom_callback,
            10)
        self.time_step = 0.1 # time step setting should be same as follower
        self.current_velocity = Twist()
        self.timer = self.create_timer(self.time_step, self.publish_velocity)

    def odom_callback(self, msg):
        self.current_velocity.linear = msg.twist.twist.linear
        self.current_velocity.angular = msg.twist.twist.angular

    def publish_velocity(self):
        self.publisher_.publish(self.current_velocity)
        # self.get_logger().info(f'Publishing: {self.current_velocity}')

def main(args=None):
    rclpy.init(args=args)
    leader_node = LeaderNode()
    rclpy.spin(leader_node)
    leader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
