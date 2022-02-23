from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deep_sogm',
            executable='collider',
            name='sogm_collider',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'nav_without_sogm': 'false'}
            ]
        )
    ])