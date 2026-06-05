import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_serial  = get_package_share_directory('robot_serial')
    pkg_imu     = get_package_share_directory('robot_imu')
    pkg_lidar   = get_package_share_directory('robot_lidar')
    pkg_odom    = get_package_share_directory('robot_odometry')
    pkg_bringup = get_package_share_directory('robot_bringup')
    chassis_port = LaunchConfiguration('chassis_port', default='/dev/ttyACM0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    log_level = LaunchConfiguration('log_level', default='warn')

    return LaunchDescription([
        DeclareLaunchArgument(
            'chassis_port',
            default_value='/dev/ttyACM0',
            description='Serial port for chassis MCU',
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLidar',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for hardware bringup nodes',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_serial, 'launch', 'serial.launch.py')),
            launch_arguments={
                'serial_port': chassis_port,
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_imu, 'launch', 'imu.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_lidar, 'launch', 'lidar.launch.py')),
            launch_arguments={
                'serial_port': lidar_port,
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_odom, 'launch', 'odometry.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'laserfilter.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
        ),
    ])
