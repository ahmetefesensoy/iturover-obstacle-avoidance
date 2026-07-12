#!/usr/bin/env python3
"""
ml_robot.launch.py — Behavioural-Cloning politikasıyla reaktif sürüş (ROS 2).

ROS 1 ml_robot.launch dosyasının ROS 2 karşılığı. Sim ortamı ile node aynı
launch dosyasından birlikte başlatılır (ödev gereği).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PARAMS = [
    ("weights_path",      ""),
    ("n_sectors",         36),
    ("max_vel",           0.26),
    ("max_angular",       1.82),
    ("run_duration",      60.0),
    # Wedge-recovery safety constants (fgm_dwa_robot.launch varsayılanlarını yansıtır)
    ("wedge_front_dist",  0.20),
    ("wedge_consecutive", 3),
    ("recover_seconds",   0.7),
    ("recover_linear",    -0.13),
]


def generate_launch_description():
    sim_share = get_package_share_directory('reactive_robot_sim')

    declared_args = [
        DeclareLaunchArgument(name, default_value=str(default))
        for name, default in _PARAMS
    ]

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'reactive_robot_sim.launch.py')
        ),
        launch_arguments={
            'gui':   'true',
            'rviz':  'true',
            'model': 'burger_rgbd',
        }.items(),
    )

    node_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
        if not isinstance(default, str)
        else LaunchConfiguration(name)
        for name, default in _PARAMS
    }

    ml_robot = Node(
        package='solution_ml',
        executable='ml_robot',
        name='ml_robot',
        output='screen',
        parameters=[node_params, {'use_sim_time': True}],
    )

    return LaunchDescription(declared_args + [sim_launch, ml_robot])
