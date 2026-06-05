from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_level = LaunchConfiguration('log_level', default='warn')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for odometry node',
        ),
        Node(
            package='robot_odometry',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )
    ])
