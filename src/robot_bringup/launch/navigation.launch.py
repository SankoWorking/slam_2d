from launch import LaunchDescription
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('robot_bringup')

    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(nav2_launch_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': 'false',
            }.items()
        ),
    ])