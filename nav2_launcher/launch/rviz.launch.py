#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="true")
    declare_rviz_config = DeclareLaunchArgument('rviz_config', description='RViz2 config file')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config)
    ld.add_action(rviz)
    return ld
