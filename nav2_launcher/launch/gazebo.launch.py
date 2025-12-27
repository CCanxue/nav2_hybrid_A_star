#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    declare_world = DeclareLaunchArgument('world', description='Gazebo world file (SDF)')
    declare_gui = DeclareLaunchArgument('gui', default_value='true', description='Start gzclient (GUI)')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_gui)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    return ld
