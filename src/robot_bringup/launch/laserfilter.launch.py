from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    params_file = os.path.join(pkg_dir, 'config', 'laser_filter_params.yaml')
    log_level = LaunchConfiguration('log_level', default='warn')

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        arguments=['--ros-args', '--log-level', log_level],
        parameters = [params_file],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for laser filter node',
        ),
        laser_filter_node
    ])
