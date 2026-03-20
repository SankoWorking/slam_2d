import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.18', '0', '0.25', '0', '0', '0', 'base_link', 'laser_frame']
    )

    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/rplidar', 
            'frame_id': 'laser_frame'
        }.items()
    )

    return LaunchDescription([
        static_tf_node,
        lidar_driver_launch
    ])