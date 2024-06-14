to run:
ros2 launch Trenzalore Follower_rosbot_trenzalore.launch.py leader_x:=0.5 leader_y:=0.0 leader_z:=0.0

step how run:
1. start driver in both robots:
2. start ros2 run teleop_twist_keyboard teleop_twist_keyboard 
3. start ros2 run Temeria Leader_rosbot_temeria 
4. ros2 run Trenzalore Follower_rosbot_trenzalore.py leader_x:=0.5 leader_y:=0.0 leader_z:=0.0