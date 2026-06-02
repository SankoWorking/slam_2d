import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    port = context.launch_configurations.get('port', '9090')

    return [
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_web_nav', 'web_nav_server'],
            output='screen',
            additional_env={'WEB_NAV_PORT': port},
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        OpaqueFunction(function=launch_setup),
    ])
