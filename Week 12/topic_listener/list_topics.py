#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ListTopics(Node):
    def __init__(self):
        super().__init__('list_topics')
        self.get_logger().info('ListTopics node has been started')
        self.timer = self.create_timer(1.0, self.list_topics)

    def list_topics(self):
        topics = self.get_topic_names_and_types()
        for topic_name, topic_types in topics:
            self.get_logger().info(f"Topic: {topic_name}, Types: {topic_types}")

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 communication system
    node = ListTopics()
    try:
        rclpy.spin(node)  # Keep your node alive and responsive
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()