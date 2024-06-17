from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm',
            executable='leader_node',
            output='screen'
        ),
        Node(
            package='swarm',
            executable='follower_node',
            output='screen'
        ),
    ])
