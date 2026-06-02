from launch import LaunchDescription
import os
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from os.path import join
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('robot_bringup')

    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_name = context.launch_configurations.get('map', 'map')
    map_yaml_file = os.path.join(pkg_dir, 'maps', f'{map_name}.yaml')

    print(f"Loading map from: {map_yaml_file}")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(nav2_launch_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': 'false',
            }.items()
        ),
    ]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='map',
        description='Map name (without .yaml extension) in robot_bringup/maps/',
    )

    return LaunchDescription([
        map_arg,
        OpaqueFunction(function=launch_setup),
    ])
