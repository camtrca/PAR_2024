# Swarm Simulation

## Overview

This project demonstrates a simple swarm simulation where a leader robot's movements are followed by a follower robot. The leader obtains its current actions via odometry and publishes them to the `/leader/vel_info` topic. The follower subscribes to this topic to understand the leader's actions and mimics them accordingly. The interaction in this simulation is unidirectional, meaning only the leader publishes information while the follower subscribes and reacts.

## Simulation Details

### Leader Node
- Retrieves the current behavior using odometry data.
- Publishes the velocity information to the `/leader/vel_info` topic.

### Follower Node
- Subscribes to the `/leader/vel_info` topic to receive the leader's actions.
- Determines the leader's current status based on the received information.
- If the leader is moving, the follower gradually follows the leader's path.
- If the leader stops, the follower also stops.

## Demonstration

A short demonstration video of the simulation is available in the `video_demo` folder.

## Running the Simulation

To run the simulation, follow these steps:

1. Launch the simulation environment:
   ```bash
   ros2 launch rosbot_gazebo simulation.launch.py world:=/home/user/aiil_workspace/humble_workspace/src/aiil_gazebo/worlds/maze.sdf robots:="leader={x: 0.0, y: 2.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0};follower={x: -1.0, y: 2.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
2. Launch the leader and follower nodes:
   ```bash
   ros2 launch swarm swarm_launch_file.launch.py
3. Control the leader using the keyboard:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/leader/cmd_vel
