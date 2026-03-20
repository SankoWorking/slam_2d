from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_serial',
            executable='serial_reader_node',
            name='serial_node',
            output='screen'
        )
    ])