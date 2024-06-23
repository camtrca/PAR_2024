import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class rotate_robot(Node):

    def __init__(self):
        super().__init__('rotate_bot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.yaw = 0.0
        self.initial_yaw = None
        self.target_yaw = None
        self.is_rotating = False
        self.tolerance = math.radians(2)  # 2 degrees tolerance

    def odom_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.initial_yaw is None:
            self.initial_yaw = self.yaw
            self.target_yaw = self.normalize_angle(self.initial_yaw - math.pi / 2)  # 90 degrees in radians

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def rotate_bot(self):
        error = self.normalize_angle(self.target_yaw - self.yaw)
        self.is_rotating = True
        if abs(error) > self.tolerance:
            twist = Twist()
            twist.angular.z = 0.5 if error > 0 else -0.5
            self.publisher_.publish(twist)
        else:
            twist = Twist()
            self.publisher_.publish(twist)
            self.is_rotating = False
            self.get_logger().info('Rotation complete')

    # def run_square(self):

    

def main(args=None):
    rclpy.init(args=args)
    rb = rotate_robot()
    rb.start_rotation()

    rclpy.spin(rb)

    rb.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
