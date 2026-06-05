from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    log_level = LaunchConfiguration('log_level', default='warn')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for Cartographer nodes',
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_dir, 'config'),
                '-configuration_basename', 'slam.lua',
                '--ros-args', '--log-level', log_level,
            ],
            remappings=[
                ('/imu', '/imu/data'),
                ('/odom', '/odom')
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'resolution': 0.05}]
        ),
    ])
