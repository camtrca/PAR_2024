from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('leader', default_value='leader'),
        DeclareLaunchArgument('followers', default_value='follower'),
        Node(
            package='Trenzalore',
            executable='Follower_rosbot_trenzalore',
            name='Follower_rosbot_trenzalore',
            output='screen',
            parameters=[
                {'leader': LaunchConfiguration('leader')}
            ]
        ),
    ])
