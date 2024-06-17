#!/usr/bin/env python  


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, PoseStamped
import argparse
# import geometry_msgs.msg

class MoveTo(Node):
    def __init__(self):
        super().__init__('move_to')
        
        # Config
        self.map_frame = "map"
        
        # Goal position publisher
        self.topic = "/goal_pose"
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('w', 1.0)
        
        # Set the target pose
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.w = self.get_parameter('w').get_parameter_value().double_value
        
        # Setup timer callback
        self.timer = self.create_timer(1.0, self.goToPose)
    
    def goToPose(self):
        time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.5)
        
        self.get_logger().info("--------------------------------")
        self.get_logger().info("Time:" + str(time))
        
        # Construct the destination pose
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = self.w
        self.get_logger().info(f" - Destination Pose: {pose}")
        
        # Construct Message
        poseS = PoseStamped()
        poseS.header.frame_id = self.map_frame
        poseS.header.stamp = time.to_msg()
        poseS.pose = pose
        
        # Publish
        self.pub.publish(poseS)
        self.get_logger().info(f" - Published")


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveTo()
    
    try:
        # Execute once
        rclpy.spin_once(node)
        
        # Execute on repeat
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)


