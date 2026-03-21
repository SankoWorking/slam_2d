import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_serial = get_package_share_directory('robot_serial')
    pkg_imu    = get_package_share_directory('robot_imu')
    pkg_lidar  = get_package_share_directory('robot_lidar')
    pkg_odom   = get_package_share_directory('robot_odometry')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_serial, 'launch', 'serial.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_imu, 'launch', 'imu.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_lidar, 'launch', 'lidar.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_odom, 'launch', 'odometry.launch.py'))),
    ])