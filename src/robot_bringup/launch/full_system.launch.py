import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('robot_bringup')

    hardware = context.launch_configurations.get('hardware', 'true')
    navigation = context.launch_configurations.get('navigation', 'true')
    web_nav = context.launch_configurations.get('web_nav', 'true')
    rviz = context.launch_configurations.get('rviz', 'false')
    map_name = context.launch_configurations.get('map', 'map')
    web_port = context.launch_configurations.get('web_port', '9090')

    actions = []

    if hardware == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('robot_bringup'),
                                 'launch', 'bringup.launch.py')
                )
            )
        )

    if navigation == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
                ),
                launch_arguments={'map': map_name}.items(),
            )
        )

    if web_nav == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('robot_web_nav'),
                                 'launch', 'web_nav.launch.py')
                ),
                launch_arguments={'port': web_port}.items(),
            )
        )

    if rviz == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_bringup, 'launch', 'rviz.launch.py')
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('web_nav', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('map', default_value='map'),
        DeclareLaunchArgument('web_port', default_value='9090'),
        OpaqueFunction(function=launch_setup),
    ])
