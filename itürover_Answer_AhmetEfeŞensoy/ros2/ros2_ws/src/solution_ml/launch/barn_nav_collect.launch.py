#!/usr/bin/env python3
"""
barn_nav_collect.launch.py — BARN dünyasında Nav2 ile veri toplama (ROS 2).

ROS 1 barn_nav_collect.launch dosyasının ROS 2 karşılığı. En büyük fark:
move_base + TEB + ayrı costmap dosyaları yerine Nav2 bringup (controller_server /
planner_server / bt_navigator / behavior_server) tek nav2_params.yaml ile
başlatılır (bkz. config/nav2_params.yaml).

Diğer değişiklikler:
  * <node pkg="tf" type="static_transform_publisher"> → tf2_ros static_transform_publisher
    (ROS 2'de argümanlar x y z yaw pitch roll parent child sırasındadır)
  * move_base action arayüzü → Nav2 navigate_to_pose (barn_goal_publisher.py içinde)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_share = get_package_share_directory('reactive_robot_sim')
    ml_share = get_package_share_directory('solution_ml')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    world_file   = LaunchConfiguration('world_file')
    world_idx    = LaunchConfiguration('world_idx')
    world_tag    = LaunchConfiguration('world_tag')
    out_path     = LaunchConfiguration('out_path')
    run_duration = LaunchConfiguration('run_duration')
    max_vel      = LaunchConfiguration('max_vel')
    max_angular  = LaunchConfiguration('max_angular')
    spawn_x      = LaunchConfiguration('spawn_x')
    spawn_y      = LaunchConfiguration('spawn_y')

    declared_args = [
        DeclareLaunchArgument('world_file', description='Gazebo .world dosya yolu'),
        DeclareLaunchArgument('world_idx', default_value='0'),
        DeclareLaunchArgument('world_tag', default_value='barn_unknown'),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('run_duration', default_value='120.0'),
        DeclareLaunchArgument('max_vel', default_value='0.90'),
        DeclareLaunchArgument('max_angular', default_value='2.84'),
        DeclareLaunchArgument('spawn_x', default_value='-4.42'),
        DeclareLaunchArgument('spawn_y', default_value='3.98'),
    ]

    # ── Gazebo headless (gui=false) ───────────────────────────────────────────
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'pause': 'false', 'verbose': 'false'}.items(),
    )

    # ── TurtleBot3 Burger spawn ──────────────────────────────────────────────
    xacro_path = os.path.join(sim_share, 'urdf')
    robot_description_content = Command([
        'xacro ', xacro_path, '/turtlebot3_burger_rgbd.urdf.xacro ground_truth:=false'
    ])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-entity', 'turtlebot3_burger_rgbd',
            '-x', spawn_x, '-y', spawn_y, '-z', '0.1', '-Y', '0.0',
        ],
    )

    # ── map→odom sabit TF: offset = robot spawn pozu ──────────────────────────
    # ROS 1 tf static_transform_publisher argüman sırası (x y z yaw pitch roll
    # parent child period) ile ROS 2 tf2_ros'un sırası (x y z yaw pitch roll
    # frame-id child-frame-id) aynıdır; period argümanı ROS 2'de yoktur.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        output='screen',
        arguments=[spawn_x, spawn_y, '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── odom→base_footprint TF (URDF publish_odom_tf=false) ─────────────────
    odom_to_tf = Node(
        package='solution_ml',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
    )

    # ── Nav2 bringup (ROS 1 move_base + TEB + costmap'lerin karşılığı) ───────
    nav2_params_file = os.path.join(ml_share, 'config', 'nav2_params.yaml')
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    # ── Goal publisher ────────────────────────────────────────────────────────
    barn_goal_publisher = Node(
        package='solution_ml',
        executable='barn_goal_publisher',
        name='barn_goal_publisher',
        output='screen',
        parameters=[{'world_idx': ParameterValue(world_idx, value_type=int)}],
    )

    # ── Shutdown timer ────────────────────────────────────────────────────────
    shutdown_timer = Node(
        package='solution_ml',
        executable='shutdown_timer',
        name='shutdown_timer',
        output='screen',
        parameters=[{'duration': ParameterValue(run_duration, value_type=float)}],
    )

    # ── Data Collector ────────────────────────────────────────────────────────
    collect_data = Node(
        package='solution_ml',
        executable='collect_data',
        name='collect_data',
        output='screen',
        parameters=[{
            'n_sectors': 36,
            'max_vel': ParameterValue(max_vel, value_type=float),
            'max_angular': ParameterValue(max_angular, value_type=float),
            'world_tag': world_tag,
            'out_path': out_path,
        }],
    )

    return LaunchDescription(declared_args + [
        gzserver,
        robot_state_publisher,
        spawn_entity,
        map_odom_tf,
        odom_to_tf,
        navigation_launch,
        barn_goal_publisher,
        shutdown_timer,
        collect_data,
    ])
