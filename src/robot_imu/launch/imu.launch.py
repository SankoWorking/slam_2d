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
            description='ROS log level for IMU nodes',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf_pub',
            arguments=[
                '-0.07', '0.02', '0.145', '0', '0', '0',
                'base_link', 'imu_link',
                '--ros-args', '--log-level', log_level,
            ]
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'reverse_tf': False
            }],
        )
    ])
