from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('leader', default_value='leader'),
        DeclareLaunchArgument('followers', default_value='follower'),  
        Node(
            package='swarm_pkg',
            executable='cmd_vel_republisher',
            name='cmd_vel_republisher',
            output='screen',
            parameters=[
                {'leader': LaunchConfiguration('leader')},
                {'followers': LaunchConfiguration('followers')}
            ]
        ),
    ])
