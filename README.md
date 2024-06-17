# ROS2 Swarm Project

This project consists of two ROS2 nodes: a `LeaderNode` and a `FollowerNode`. The `LeaderNode` publishes its pose and sends it to the `FollowerNode` using TCP. The `FollowerNode` receives this pose information and follows the leader.

## Node Descriptions

### Leader Node (`leader.py`)

The `LeaderNode` performs the following functions:
1. Initializes a TCP server to send pose information to the `FollowerNode`.
2. Uses a `TransformListener` to get the current pose of the leader in the map frame.
3. Publishes the pose at regular intervals (every 0.1 seconds) using a timer.
4. Sends the pose data to the `FollowerNode` over a TCP connection.

#### Key Methods:
- `transform_bot_pose(dest='map', src='base_link')`: Retrieves the current pose of the leader.
- `publish_pose()`: Publishes the current pose and sends it to the follower.
- `send_pose_data(pose)`: Serializes and sends pose data to the follower using TCP.
- `destroy_node()`: Closes the TCP connection and cleans up resources.

### Follower Node (`follower.py`)

The `FollowerNode` performs the following functions:
1. Connects to the `LeaderNode` over TCP to receive pose information.
2. Uses a `TransformListener` to get and apply transforms between different frames.
3. Receives and processes laser scan data to set an initial position.
4. Uses a timer to periodically check the leader's position and set navigation goals.
5. Publishes visualization markers for RViz.

#### Key Methods:
- `scan_callback(msg)`: Processes laser scan data to set the initial distance.
- `set_initial_position()`: Sets the initial position of the follower relative to the leader.
- `pose_callback(msg)`: Receives and processes the leader's pose information.
- `apply_transform_vector(leader_pose)`: Applies the transformation vector to the leader's pose.
- `publish_leader_pose(pose)`: Publishes the leader's pose as a marker in RViz.
- `check_and_set_goal()`: Periodically checks the leader's position and sets navigation goals.
- `set_goal(goal_pose)`: Sends a navigation goal to the `NavigateToPose` action server.
- `goal_response_callback(future)`: Handles the response from the navigation action server.
- `get_result_callback(future)`: Processes the result of the navigation goal.
- `feedback_callback(feedback_msg)`: Processes feedback during goal execution.
- `receive_and_process()`: Receives and processes data from the `LeaderNode` over TCP.

## Issues and Considerations

- **Transform Issues**: Both nodes may encounter issues with missing transforms such as `/map` or `/base_link`. Ensure that these frames are correctly set up and available in your TF tree.


