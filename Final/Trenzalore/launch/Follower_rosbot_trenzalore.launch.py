from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_x', default_value='0.0', description='Initial X position of the leader'
        ),
        DeclareLaunchArgument(
            'leader_y', default_value='0.0', description='Initial Y position of the leader'
        ),
        DeclareLaunchArgument(
            'leader_z', default_value='0.0', description='Initial Z position of the leader'
        ),
        Node(
            package='Trenzalore',
            executable='Follower_rosbot_trenzalore',
            name='Follower_rosbot_trenzalore',
            output='screen',
            parameters=[{
                'leader_x': LaunchConfiguration('leader_x'),
                'leader_y': LaunchConfiguration('leader_y'),
                'leader_z': LaunchConfiguration('leader_z')
            }]
        )
    ])