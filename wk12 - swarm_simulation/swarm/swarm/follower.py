import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        # Subscription to leader's velocity information
        self.subscription = self.create_subscription(
            Twist,
            '/leader/vel_info',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/follower/cmd_vel', 10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/follower/scan',
            self.scan_callback,
            10)
        self.time_step = 0.1
        self.initial_position_set = False
        self.initial_distance = 0.0
        self.vel_list = []
        self.current_velocity = Twist()
        self.get_logger().info('Follower Node has been started.')

    def scan_callback(self, msg):
        # self.get_logger().info(f'laser callback : {msg.ranges[0]}')
        if not self.initial_position_set:
            self.initial_distance = msg.ranges[0]
            self.get_logger().info(f'Initial distance : {self.initial_distance}')
            self.initial_position_set = True
            self.calculate_initial_movements()

    def calculate_initial_movements(self):
        # Calculate initial movements to reach the leader
        vel = Twist()
        speed = 0.08 # Set the desired speed
        vel.linear.x = speed*6
        move_distance_per_step = speed * self.time_step
        steps = int(self.initial_distance / move_distance_per_step)
        for _ in range(steps):
            self.vel_list.append(vel)      

        remaining_distance = self.initial_distance - steps * move_distance_per_step
        if remaining_distance > 0:
            final_step = Twist()
            final_step.linear.x = remaining_distance / self.time_step
            self.vel_list.append(final_step)
        
        self.get_logger().info(f'Initial movements calculated: {len(self.vel_list)} steps')

    def stop(self):
        # Function to stop the follower
        stop_vel = Twist()
        self.publisher_.publish(stop_vel)

    def is_stationary(self, msg, threshold=1e-4):
        # Determine if the robot is stationary based on the given message.
        if abs(msg.linear.x) < threshold and abs(msg.linear.y) < threshold and abs(msg.linear.z) < threshold and \
           abs(msg.angular.x) < threshold and abs(msg.angular.y) < threshold and abs(msg.angular.z) < threshold:
            return True
        return False

    def listener_callback(self, msg):
        # Callback function for leader's velocity information
        if not self.initial_position_set:
            return

        if self.is_stationary(msg):
            # if leader stop then follower stop
            self.stop()
        else:
            if self.vel_list:
                next_move = self.vel_list.pop(0)
                self.publisher_.publish(next_move)
                self.get_logger().info(f'Publishing: {next_move}')
            self.vel_list.append(msg)

def main(args=None):
    rclpy.init(args=args)
    follower_node = FollowerNode()
    rclpy.spin(follower_node)
    follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
