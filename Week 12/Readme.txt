Hi, This readme file is the instruction used to estabulish a basic communication in between Rosbot running ROS2 using TCP servers.

In the version 1.0 zip, you should see 
	- sample code
	- Temeria.zip
	- Trenzalore.zip
Sample code will be used later when we communicating using other topics, within the sample, topic /scan is being used as an example.

Temeria.zip should only be runnning on the rosbot of <Temeria>
Trenzalore.zip should only be running on the rosbot of <Trenzalore>


Pre-requirement and acknowlegdment before running:
1. Within the ssh of the robot, type in <rosenv> for checking ROS_DOMAIN_ID
	Temeria > 98
	Trenzalore > 99
	
2. After adapting and changing the ROS_DOMAIN_ID, colcon build each pkg and source after de-compressing them from the zip file.

Running command:
	- colcon build --packages-select Temeria Trenzalore
alternativly you can do :
	ros2 pkg create Temeria --build-type ament_python --dependencies rclpy nav_msgs geometry_msgs
	ros2 pkg create Trenzalore --build-type ament_python --dependencies rclpy nav_msgs geometry_msgs
Remember to copy the setup file.

	- source install/setup.bash
	
	On Temeria: -ros2 run Temeria Leader_rosbot_temeria 
	On Trenzalore: -ros2 run Trenzalore Follower_rosbot_trenzalore
	On Third terminal, export to temeria via :
		- export ROS_DOMAIN_ID=99
		- ros2 run teleop_twist_keyboard telep_twist_keyboard


Last edited May 26th 2024
J.J
	
