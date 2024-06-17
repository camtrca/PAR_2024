# Leader Robot Node

## Overview

The `Leader` class implements a ROS2 node designed to control a robot that can navigate in different patterns based on user input. The robot can move in a square pattern using odometry and velocity commands, or it can navigate a square pattern using map points with the help of the `nav2` stack.

## Features

- Move in a square pattern using odometry and velocity commands (`/cmd_vel`).
- Move in a square pattern based on map points using the `nav2` stack.
- Send odometry data to a connected client.
- Rotate the robot to specific angles using odometry.

## Node Parameters

- `square_size` (int): The size of the square for the movement pattern.
- `shape_type` (str): The type of movement pattern (`square`,  `map_square`).

## Subscriptions

- `/odometry/filtered` (nav_msgs/msg/Odometry): Subscribes to the robot's odometry data.
- `/map` (nav_msgs/msg/OccupancyGrid): Subscribes to the map data.

## Publishers

- `cmd_vel` (geometry_msgs/msg/Twist): Publishes velocity commands to control the robot.

## Action Client

- `NavigateToPose` (nav2_msgs/action/NavigateToPose): Action client for navigating to specific poses on the map.

## Methods

- `odom_callback`: Callback for odometry data, updates the robot's yaw.
- `rotate_bot`: Rotates the robot to a target yaw.
- `start_rotation`: Initiates the rotation process.
- `send_odom_data`: Sends odometry data to a connected client.
- `map_callback`: Callback for map data.
- `move_in_square`: Moves the robot in a square pattern.
- `move_in_circle`: Moves the robot in a circular pattern.
- `move_in_map_square`: Moves the robot in a square pattern based on map points.
- `send_goal`: Sends a navigation goal to a specified point.
- `goal_response_callback`: Handles the response for the navigation goal.
- `get_result_callback`: Handles the result of the navigation goal.
- `feedback_callback`: Handles feedback during navigation.
- `wait_for_result`: Waits for the navigation result.
- `move_forward`: Moves the robot forward by a specified distance.
- `turn`: Turns the robot by a specified angle.

## Running the Node

To run the node, execute the `main` function:

```python
def main(args=None):
    rclpy.init(args=args)

    size = 0
    shape = ''

    while size > 4 or size < 1:
        try:
            size = int(input("Please select a size between 1 and 4: "))
        except ValueError:
            print("Invalid input. Please enter an integer between 1 and 4.")

    while shape not in ['square', 'map_square']:
        shape = input("Please select a shape ('square', 'map_square'): ").strip().lower()
        if shape not in ['square', 'map_square']:
            print("Invalid input. Please enter 'circle', 'square' or 'map_square'.")

    leader = Leader(size, shape)

    try:
        leader.run()
    except KeyboardInterrupt:
        pass

    leader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()