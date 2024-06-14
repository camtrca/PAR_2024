import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class Leader(Node):
    def __init__(self, square_size, shape_type):
        super().__init__('leader')
        self.square_size = square_size
        self.shape_type = shape_type
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info(f"Square size set to: {self.square_size}")
        self.get_logger().info(f"Shape type set to: {self.shape_type}")
        self.duration_multi = 2.231

    def run(self):
        if self.shape_type == 'square':
            self.move_in_square()
        elif self.shape_type == 'circle':
            self.move_in_circle()
        else:
            self.get_logger().error(f"Unknown shape type: {self.shape_type}")

    def move_in_square(self):
        self.get_logger().info("Moving in a square.")
        for _ in range(4):
            self.move_forward(self.square_size)
            self.turn(90)

    def move_in_circle(self):
        self.get_logger().info("Moving in a circle.")
        radius = self.square_size / 2
        circumference = 2 * math.pi * radius
        steps = 36  # Number of steps to approximate a circle
        step_length = circumference / steps
        step_angle = 360 / steps
        for _ in range(steps):
            self.move_forward(step_length)
            self.turn(step_angle)

    def move_forward(self, distance):
        self.get_logger().info(f"Moving forward {distance} units.")
        twist = Twist()
        twist.linear.x = 0.5  # Adjust speed as necessary
        duration = distance / twist.linear.x
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration*self.duration_multi)
        while self.get_clock().now() < end_time:
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)  # Give some time to stop

    def turn(self, angle):
        self.get_logger().info(f"Turning {angle} degrees.")
        twist = Twist()
        twist.angular.z = math.radians(90)  # Adjust turn speed as necessary
        duration = math.radians(angle) / twist.angular.z
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration*self.duration_multi)
        while self.get_clock().now() < end_time:
            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)  # Give some time to stop

def main(args=None):
    rclpy.init(args=args)

    size = 0
    shape = ''

    while size not in [1, 2, 3, 4]:
        try:
            size = int(input("Please select a size between 1 and 4: "))
        except ValueError:
            print("Invalid input. Please enter an integer between 1 and 4.")

    while shape not in ['circle', 'square']:
        shape = input("Please select a shape ('circle' or 'square'): ").strip().lower()
        if shape not in ['circle', 'square']:
            print("Invalid input. Please enter 'circle' or 'square'.")

    leader = Leader(size, shape)

    try:
        leader.run()
    except KeyboardInterrupt:
        pass

    leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
