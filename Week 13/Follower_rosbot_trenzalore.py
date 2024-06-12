import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pickle
import socket
import math

class DistanceReceiver(Node):
    def __init__(self):
        super().__init__('distance_receiver')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10.0)
        try:
            self.sock.connect(('localhost', 50000))
            self.get_logger().info("Connected to the server at localhost:50000")
        except socket.timeout:
            self.get_logger().error("Connection to the server timed out")
            return
        
        self.last_position = None
        self.total_distance = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def receive_and_process(self):
        try:
            while rclpy.ok():
                # Read the length of the data first
                raw_length = self.sock.recv(4)
                if raw_length:
                    data_length = int.from_bytes(raw_length, byteorder='big')
                    data = b''
                    while len(data) < data_length:
                        more = self.sock.recv(data_length - len(data))
                        if not more:
                            raise Exception("Socket connection broken")
                        data += more
                    odom_data = pickle.loads(data)
                    self.calculate_and_move(odom_data['position'])
                else:
                    self.get_logger().info("No more data received, closing connection...")
                    break
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
        finally:
            self.sock.close()

    def calculate_and_move(self, current_position):
        if self.last_position is not None:
            distance = math.sqrt(
                (current_position['x'] - self.last_position['x']) ** 2 +
                (current_position['y'] - self.last_position['y']) ** 2 +
                (current_position['z'] - self.last_position['z']) ** 2)
            distance = round(distance, 2)
            if distance > 0.01:
                self.total_distance += distance
                self.get_logger().info(f"New distance segment: {distance}, Total distance: {self.total_distance}")
                self.move_robot(distance)
        self.last_position = current_position

    def move_robot(self, distance):
        velocity = max(0.1, distance / 2)  # Minimum speed threshold
        move_cmd = Twist()
        move_cmd.linear.x = velocity
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        # Calculate sleep time based on distance and velocity
        sleep_time = distance / velocity
        # Use a ROS 2 timer to handle the waiting without blocking
        timer = self.create_timer(sleep_time, self.stop_robot)
        # Keep the node alive until it's time to stop the robot
        rclpy.spin_once(self, timeout_sec=sleep_time)
        timer.cancel()  # Cancel the timer if it's still running

    def stop_robot(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0  # Stop the robot
        self.publisher.publish(move_cmd)
        self.get_logger().info("Robot movement command completed.")

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
