#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    pkg_share = FindPackageShare('nav2_launcher')

    # --- High level switches ---
    robot = LaunchConfiguration('robot')
    namespace = LaunchConfiguration('namespace')

    declare_robot = DeclareLaunchArgument(
        'robot',
        default_value='turtlebot3_waffle',
        description='Robot profile folder under nav2_launcher/robots/'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this robot/Nav2 stack (empty = no namespace)'
    )
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value="False",
        description='Whether to apply namespace inside Nav2 bringup'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value="true",
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_sim = DeclareLaunchArgument('use_sim', default_value="true", description='Start Gazebo server/client')
    declare_gui = DeclareLaunchArgument('gui', default_value="true", description='Start Gazebo GUI (gzclient)')
    declare_use_rsp = DeclareLaunchArgument('use_rsp', default_value="true", description='Start robot_state_publisher')
    declare_use_spawn = DeclareLaunchArgument('use_spawn', default_value="true", description='Spawn robot into Gazebo')
    declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value="true", description='Start Nav2 stack (bringup)')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value="true", description='Start RViz2')

    # --- Default paths from robot profile ---
    default_urdf = PathJoinSubstitution([pkg_share, 'robots', robot, 'urdf', 'robot.urdf.xacro'])
    default_params = PathJoinSubstitution([pkg_share, 'robots', robot, 'config', 'nav2_params.yaml'])
    default_map = PathJoinSubstitution([pkg_share, 'robots', robot, 'maps', 'house.yaml'])
    default_rviz = PathJoinSubstitution([pkg_share, 'robots', robot, 'rviz', 'nav2.rviz'])
    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'house.world'])
    default_sdf = PathJoinSubstitution([pkg_share,'robots', robot, 'models', robot, 'model.sdf'])

    declare_urdf = DeclareLaunchArgument('urdf', default_value=default_urdf, description='URDF/Xacro path')
    declare_sdf = DeclareLaunchArgument('sdf', default_value=default_sdf, description='Gazebo SDF path')
    declare_params_file = DeclareLaunchArgument('params_file', default_value=default_params, description='Nav2 params YAML')
    declare_map = DeclareLaunchArgument('map', default_value=default_map, description='Map YAML for map_server')
    declare_rviz_config = DeclareLaunchArgument('rviz_config', default_value=default_rviz, description='RViz2 config')
    declare_world = DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world file')

    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='-1.99')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='-0.48')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value="False",
        description='Use SLAM (slam_toolbox) instead of AMCL+map_server'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value="true",
        description='Autostart Nav2 lifecycle nodes'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'gazebo.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_sim')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'robot_state_publisher.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_rsp')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf': LaunchConfiguration('urdf'),
        }.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'spawn_entity.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_spawn')),
        launch_arguments={
            'namespace': namespace,
            'entity': robot,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # 'urdf': LaunchConfiguration('urdf'),
            'sdf': LaunchConfiguration('sdf'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'nav2_bringup.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': LaunchConfiguration('use_namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
            'slam': LaunchConfiguration('slam'),
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    # 等待 spawn_entity.launch.py 发出完成信号，再启动 Nav2
    wait_spawn_done = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_spawn')),
        cmd=['ros2', 'topic', 'echo', '--once', '/nav2_launcher/spawn_done'],
        output='screen'
    )

    start_nav2_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_spawn_done,
            on_exit=[nav2]   # nav2 你原本的 IncludeLaunchDescription（带 use_nav2 条件也没关系）
        )
    )

    # 不 spawn 的情况下，Nav2 仍然照常立即启动（避免卡住）
    nav2_no_spawn = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_nav2')),
        actions=[
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration('use_spawn')),
                actions=[nav2],
            )
        ]
    )   
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'rviz.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    ld = LaunchDescription()
    for a in [
        declare_robot, declare_namespace, declare_use_namespace, declare_use_sim_time,
        declare_use_sim, declare_gui, declare_use_rsp, declare_use_spawn, declare_use_nav2, declare_use_rviz,
        declare_urdf,  declare_sdf, declare_params_file, declare_map, declare_rviz_config, declare_world,
        declare_x_pose, declare_y_pose, declare_z_pose, declare_yaw,
        declare_slam, declare_autostart,
    ]:
        ld.add_action(a)

    ld.add_action(gazebo)
    ld.add_action(rsp)
    ld.add_action(spawn)
    ld.add_action(nav2_no_spawn)
    ld.add_action(wait_spawn_done)
    ld.add_action(start_nav2_after_spawn)
    #ld.add_action(nav2)
    ld.add_action(rviz)

    return ld
