import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')

        # Declare parameters with default values
        self.declare_parameter('leader', 'leader')
        self.declare_parameter('followers', 'follower')

        # Get ledaer name
        leader = self.get_parameter(
            'leader').get_parameter_value().string_value

        # Get list of follower name
        followers_str = self.get_parameter(
            'followers').get_parameter_value().string_value

        # Split the comma-separated followers string into a list
        followers = followers_str.split(',')

        # Create subscription to leader cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            f'/{leader}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers for each follower
        self._publishers = []
        for follower in followers:
            self._publishers.append(self.create_publisher(
                Twist, f'/{follower.strip()}/cmd_vel', 10))

    def cmd_vel_callback(self, msg):
        # Publish the cmd_vel to all follower robots
        for publisher in self._publishers:
            publisher.publish(msg)


def main(args=None):
    # Intitialise the ROS2 program
    rclpy.init(args=args)

    # Initialise the node
    node = CmdVelRepublisher()

    # Start the node in a loop
    rclpy.spin(node)

    # stop the node
    node.destroy_node()

    # stop Ros2 program
    rclpy.shutdown()


if __name__ == '__main__':
    main()
