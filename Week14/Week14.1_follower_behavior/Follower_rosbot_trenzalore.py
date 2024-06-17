import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pickle
import socket
import time
import queue

class DistanceReceiver(Node):

    def __init__(self):
        super().__init__('distance_receiver')
        self.sock = None
        self.setup_connection()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_queue = queue.Queue()
        self.is_processing = False

    def setup_connection(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('localhost', 50000))
            self.get_logger().info("Connected to the server at localhost:50000")
        except socket.timeout:
            self.get_logger().error("Connection to the server timed out, trying again in 5 seconds...")
            rclpy.get_default_context().call_later(5, self.setup_connection)

    def receive_and_process(self):
        try:
            while rclpy.ok():
                raw_length = self.sock.recv(4)
                if raw_length:
                    data_length = int.from_bytes(raw_length, byteorder='big')
                    data = b''
                    while len(data) < data_length:
                        more = self.sock.recv(data_length - len(data))
                        if not more:
                            raise Exception("Socket connection broken")
                        data += more

                    try:
                        movement_message = pickle.loads(data)
                        if isinstance(movement_message, str):
                            self.get_logger().error(f"Received data is a string, expected a dict: {movement_message}")
                            continue

                        self.command_queue.put(movement_message)
                        if not self.is_processing:
                            self.process_next_command()
                        self.get_logger().info(f"Received movement description: {movement_message}")
                    except pickle.PickleError as e:
                        self.get_logger().error(f"Failed to deserialize movement data: {e}")
                else:
                    self.get_logger().info("No more data received, stopping robot...")
                    self.stop_robot()
                    break
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.cleanup_socket()

    def process_next_command(self):
        if not self.command_queue.empty():
            self.is_processing = True
            movement_message = self.command_queue.get()
            self.actuate_robot_based_on_message(movement_message)
        else:
            self.is_processing = False

    def actuate_robot_based_on_message(self, movement_message):
        twist_data = movement_message.get('twist', {})
        twist = Twist()
        twist.linear.x = twist_data.get('linear', {}).get('x', 0.0)
        twist.linear.y = twist_data.get('linear', {}).get('y', 0.0)
        twist.linear.z = twist_data.get('linear', {}).get('z', 0.0)
        twist.angular.x = twist_data.get('angular', {}).get('x', 0.0)
        twist.angular.y = twist_data.get('angular', {}).get('y', 0.0)
        twist.angular.z = twist_data.get('angular', {}).get('z', 0.0)

        distance = movement_message.get('distance', 0.0)
        rotation = movement_message.get('rotation', 0.0)
        
        self.publish_robot_movement_command(twist, distance, rotation)
        self.get_logger().info(f"Actuating robot with: Linear X: {twist.linear.x}, Y: {twist.linear.y}, Z: {twist.linear.z}, Angular X: {twist.angular.x}, Y: {twist.angular.y}, Z: {twist.angular.z}")

    def publish_robot_movement_command(self, twist, distance, rotation):
        if distance == 0.0 and rotation == 0.0:
            self.stop_robot()
            self.process_next_command()
            return
        else:
            time_to_travel = abs(distance / twist.linear.x) if twist.linear.x != 0.0 else 0.0
            time_to_rotate = abs(rotation / twist.angular.z) if twist.angular.z != 0.0 else 0.0
            if twist.linear.x == 0.0 and twist.angular.z != 0.0:
                time_ = rclpy.duration.Duration(seconds=time_to_rotate)
                start_time = rclpy.clock.Clock().now()
            else:
                time_to_travel = max(time_to_travel, time_to_rotate)
                time_ = rclpy.duration.Duration(seconds=time_to_travel)

            self.get_logger().info(f"Changed time to travel: {time_} seconds")
            start_time = rclpy.clock.Clock().now()

        while True:
            current_time = rclpy.clock.Clock().now()
            if current_time - start_time < time_:
                self.publisher.publish(twist)
            else:
                self.stop_robot()
                break
            time.sleep(0.01)

        self.get_logger().info(f"Published move command: Linear X: {twist.linear.x}, Angular Z: {twist.angular.z}")
        self.process_next_command()

    def stop_robot(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        self.get_logger().info("Published stop command")

    def cleanup_socket(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            self.get_logger().info("Socket cleaned up and ready for a new connection.")

    def destroy_node(self):
        self.cleanup_socket()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DistanceReceiver()
    try:
        node.receive_and_process()
    except KeyboardInterrupt:
        node.get_logger().info('Node shutdown by KeyboardInterrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
