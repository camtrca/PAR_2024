from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='X coordinate'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Y coordinate'
        ),
        DeclareLaunchArgument(
            'w',
            default_value='1.0',
            description='W orientation'
        ),
        Node(
            package='move_to',
            executable='move_to',
            name='move_to',
            output='screen',
            parameters=[
                {'x': LaunchConfiguration('x')},
                {'y': LaunchConfiguration('y')},
                {'w': LaunchConfiguration('w')}
            ]
        )
    ])