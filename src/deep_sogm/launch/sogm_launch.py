from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name='nav_without_sogm', default_value='false', description='Specify if we use SOGM for navigation'),
                              DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Specify if we are in simulation or not'),
                              DeclareLaunchArgument(name='model_path', default_value='', description='Specify the name of the trained model session.'),
                              LogInfo(msg=LaunchConfiguration('nav_without_sogm')),
                              LogInfo(msg=LaunchConfiguration('use_sim_time')),
                              LogInfo(msg=LaunchConfiguration('model_path')),
                              Node(package='deep_sogm',
                                   executable='collider',
                                   name='sogm_collider',
                                   output='screen',
                                   emulate_tty=True,
                                   parameters=[{'nav_without_sogm': LaunchConfiguration('nav_without_sogm'),
                                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                                                'model_path': LaunchConfiguration('model_path')}])
                              ])
