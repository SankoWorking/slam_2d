from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    get_package_share_directory('sllidar_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    log_level = LaunchConfiguration('log_level', default='warn')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='ROS log level for lidar nodes',
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='RPLidar serial baudrate',
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='RPLidar frame id',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '0.18', '0', '0.24', '3.1415926', '0', '0',
            'base_link', frame_id,
            '--ros-args', '--log-level', log_level,
        ]
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/scan', '/scan_raw'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        log_level_arg,
        serial_baudrate_arg,
        frame_id_arg,
        static_tf_node,
        lidar_node,
    ])
