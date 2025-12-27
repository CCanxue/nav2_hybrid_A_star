#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = LaunchConfiguration('urdf')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='ROS namespace (optional)')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="true")
    declare_urdf = DeclareLaunchArgument('urdf', description='URDF/Xacro path')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_urdf)
    ld.add_action(rsp)
    return ld
