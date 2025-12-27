#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    autostart = LaunchConfiguration('autostart')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_use_namespace = DeclareLaunchArgument('use_namespace', default_value="False")
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="true")
    declare_params_file = DeclareLaunchArgument('params_file', description='Nav2 params YAML')
    declare_map = DeclareLaunchArgument('map', description='Map YAML')
    declare_slam = DeclareLaunchArgument('slam', default_value="False")
    declare_autostart = DeclareLaunchArgument('autostart', default_value="true")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam': slam,
            'autostart': autostart,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_map)
    ld.add_action(declare_slam)
    ld.add_action(declare_autostart)
    ld.add_action(bringup)
    return ld
