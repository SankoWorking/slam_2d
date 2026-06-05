import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    slam_params_file = os.path.join(pkg_bringup, 'config', 'slam_params.yaml')
    log_level = LaunchConfiguration('log_level', default='warn')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for SLAM Toolbox',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                slam_params_file,
                {'use_sim_time': False}
            ]
        )
    ])
