#!/usr/bin/env python3
"""
barn_collect.launch.py — BARN dünyasında (headless) FGM-DWA expert ile veri
toplama (ROS 2).

Kullanım:
  ros2 launch solution_ml barn_collect.launch.py \
    world_file:=/root/BARN_dataset/BARN_dataset/world_files/world_0.world \
    world_tag:=barn_0

ROS 1 barn_collect.launch dosyasının karşılığı. BARN dünyaları x:[-4.5,0]
y:[0,4.5] aralığını kaplar; robot merkeze spawn edilir.
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
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file   = LaunchConfiguration('world_file')
    world_tag    = LaunchConfiguration('world_tag')
    out_path     = LaunchConfiguration('out_path')
    run_duration = LaunchConfiguration('run_duration')
    max_vel      = LaunchConfiguration('max_vel')
    max_angular  = LaunchConfiguration('max_angular')

    declared_args = [
        DeclareLaunchArgument('world_file', description='Gazebo .world dosya yolu'),
        DeclareLaunchArgument('world_tag', default_value='barn_unknown'),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('run_duration', default_value='90.0'),
        DeclareLaunchArgument('max_vel', default_value='0.65'),
        DeclareLaunchArgument('max_angular', default_value='2.84'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'pause': 'false', 'verbose': 'false'}.items(),
    )

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
            '-x', '-2.0', '-y', '2.0', '-z', '0.1', '-Y', '0.0',
        ],
    )

    # ── FGM-DWA Expert (solution_fgm_dwa paketinin node'u, özel parametrelerle) ──
    fgm_dwa_robot = Node(
        package='solution_fgm_dwa',
        executable='reactive_robot',
        name='fgm_dwa_robot',
        output='screen',
        parameters=[{
            'max_vel':           ParameterValue(max_vel, value_type=float),
            'max_angular':       ParameterValue(max_angular, value_type=float),
            'v_min_forward':     0.28,
            'gap_threshold':     0.30,
            'min_gap_width':     0.40,
            'predict_time':      1.2,
            'sim_dt':            0.1,
            'v_samples':         7,
            'w_samples':         15,
            'obstacle_weight':   1.30,
            'potential_d0':      1.20,
            'heading_weight':    2.00,
            'speed_weight':      0.80,
            'continuity_weight': 0.25,
            'corridor_weight':   0.00,
            'max_linear_accel':  2.00,
            'max_angular_accel': 8.00,
            'robot_radius':      0.12,
            'run_duration':      ParameterValue(run_duration, value_type=float),
            'wedge_front_dist':  0.18,
            'wedge_consecutive': 5,
            'recover_seconds':   0.70,
            'recover_linear':    -0.13,
            'use_sim_time':      True,
        }],
    )

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
        gzserver, robot_state_publisher, spawn_entity,
        fgm_dwa_robot, collect_data,
    ])
