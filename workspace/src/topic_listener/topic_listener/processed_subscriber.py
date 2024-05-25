import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pickle
import socket

class ProcessedSubscriber(Node):
    def __init__(self):
        super().__init__('processed_subscriber')
        self.publisher = self.create_publisher(Twist, '/temeria/cmd_vel', 10) # Adjust the topic name
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP socket, for receiving messages
        self.sock.bind(('localhost', 50000)) # Bind the socket to the address
        self.sock.listen(1) # Listen for incoming connections
        self.conn, self.addr = self.sock.accept() # Accept the connection

    def receive_and_publish(self):
        buffer = b"" # Create a buffer to store the received data
        while True:
            try:
                data = self.conn.recv(4096) # Receive data from the client
                if not data:
                    print("nomore data received, closing connection...")
                    break
                buffer += data
                try: 
                    data = pickle.loads(buffer) # Deserialize the received data
                    print(f"Received: {data}")
                    buffer = b""
                    self.publisher.publish(data)
                except pickle.UnpicklingError:
                    continue
            except BrokenPipeError: # Handle the case where the connection is broken: close the connection, create a new one, and continue receiving data
                print("Connection broken, attempting to reconnect...")
                self.conn.close()
                self.conn, self.addr = self.sock.accept()
            except KeyboardInterrupt: 
                print("Shutting down...")
                self.conn.close()
                break
            except Exception as e:
                print(f"An error occurred: {e}")
                self.conn.close()
                break

def main(args=None):
    rclpy.init(args=args)
    node = ProcessedSubscriber()
    try:
        node.receive_and_publish()
    finally:
        node.destroy_node()
        node.conn.close()
        node.sock.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
