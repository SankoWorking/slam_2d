from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/chassis')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/chassis',
        description='Serial port for chassis MCU (default: /dev/chassis udev symlink)'
    )

    serial_node = Node(
        package='robot_serial',
        executable='serial_reader_node',
        name='serial_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': serial_port,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        serial_node
    ])
