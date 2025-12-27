#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    entity = LaunchConfiguration('entity')
    # urdf = LaunchConfiguration('urdf')
    sdf = LaunchConfiguration('sdf')  

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='ROS namespace for robot')
    declare_entity = DeclareLaunchArgument('entity', default_value='robot', description='Gazebo entity name')
    # declare_urdf = DeclareLaunchArgument('urdf', description='URDF/Xacro path')
    declare_sdf = DeclareLaunchArgument('sdf', description='Gazebo SDF path')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.01')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Expand xacro at launch-time and pass as a parameter to spawn_entity
    # robot_description = ParameterValue(Command(['xacro', ' ', urdf]), value_type=str)

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        # parameters=[{'robot_description': robot_description}],
        arguments=[
            '-entity', entity,
            '-file', sdf,
            '-robot_namespace', namespace,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
            '-timeout', '180'
        ],
    )
    publish_spawn_done = ExecuteProcess(
    cmd=['ros2', 'topic', 'pub', '-1',
         '/nav2_launcher/spawn_done', 'std_msgs/msg/Empty', '{}'],
    output='screen'
    )

    spawn_done_handler = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn,   # <- 这里换成你实际的 spawn Node 变量名（spawn 或 spawn_entity）
        on_exit=[publish_spawn_done]
    )
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_entity)
    ld.add_action(declare_sdf)
    # ld.add_action(declare_urdf)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)
    ld.add_action(declare_yaw)
    ld.add_action(spawn_done_handler)
    ld.add_action(spawn)
    return ld
