
# How to use
## Fork the repo
1. Login to The Construct
2. Folk this repo https://app.theconstruct.ai/l/6236c2db/
3. Run the Project and follow instruction in Jupyter Notebook

## Launch robot in stimulation
In the aiil_gazebo package in AIIL repository, it contains the Husarion rosbot_gazebo package for launcing the robot in simulation.
It support to assign the coordinate position values for multiple robots in the launch command.
After the "robots:=" it will all paramater values for the robot and launch it in simulation in preferred position.
Why do we launch the robot in simulation this way? Because the launch file is able to launch the robot with the namespace based on the given name in the command line.
Moreover, all of the robot topics are laid under the same Ros Domain. Hence, the robots can interact to each other by touching the topics.
Although communication method via namespace works perfectly in the stimulation, we are not able to apply to the real robot.
It is due to that the real robot has various of packages replied on the hardcoded topic name. 
However those package is simplified in the simulation.

- To launch 2 robots name as leader and follower, use the following command:

ros2 launch rosbot_gazebo simulation.launch.py world:=empty.sdf robots:="leader={x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw:0.0};follower={x: 0.0, y: 1.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
 
 
- To launch 3 robots name as leader, follower and follower1, use the following command:
 
ros2 launch rosbot_gazebo simulation.launch.py world:=empty.sdf robots:="leader={x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw:0.0};follower={x: 0.0, y: 1.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0};follower1={x: 0.0, y: 2.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0};follower2={x: 0.3, y: 1.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}"
 
## Run Swarm package
The swarm package contains a node ``cmd_vel_republisher`` to replicate the moving action from a robot to other robots.
For example by using teleop_twist_keyboard (builtin node) to controll a robot by keyboard: 

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/leader/cmd_vel
 
The ``teleop_twist_keyboard`` is controlling action on the leader robot. 
Now we run the ``cmd_vel_republisher`` package to replicate the movement on follower and follower1 robots by:

ros2 launch swarm_pkg cmd_vel_republisher.launch.py leader:=leader followers:="follower,follower1"

