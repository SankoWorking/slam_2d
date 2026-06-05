from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    port = context.launch_configurations.get('port', '9090')
    log_level = context.launch_configurations.get('log_level', 'warn')

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'robot_web_nav', 'web_nav_server',
                '--ros-args', '--log-level', log_level,
            ],
            output='screen',
            additional_env={'WEB_NAV_PORT': port},
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        OpaqueFunction(function=launch_setup),
    ])
