#!/usr/bin/env python3
"""
dqn_finetune.launch.py — BC-başlatılmış MLP politikasının DQN ile ince ayarı (ROS 2).

ROS 1 dqn_finetune.launch dosyasının ROS 2 karşılığı. Sim ortamı ile node
aynı launch dosyasından birlikte başlatılır.
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
    ("init_weights",       ""),
    ("out_weights",        ""),
    ("n_sectors",          36),
    ("max_vel",            0.26),
    ("max_angular",        1.82),
    # Spawn pose (obs_corridor_world varsayılanları)
    ("spawn_x",            0.0),
    ("spawn_y",            9.5),
    ("spawn_z",            0.0),
    ("spawn_yaw",          -1.57),
    ("episode_max_steps",  600),
    ("replay_capacity",    10000),
    ("batch_size",         64),
    ("gamma",              0.99),
    ("lr",                 0.0005),
    ("epsilon_start",      0.5),
    ("epsilon_end",        0.05),
    ("epsilon_decay_eps",  200),
    ("update_freq",        4),
    ("target_sync_steps",  200),
    ("save_every_eps",     50),
    # Wedge backstop
    ("wedge_front_dist",   0.20),
    ("wedge_consecutive",  3),
    ("recover_seconds",    0.7),
    ("recover_linear",     -0.13),
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
            'rviz':  'false',
            'model': 'burger_rgbd',
        }.items(),
    )

    node_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
        if not isinstance(default, str)
        else LaunchConfiguration(name)
        for name, default in _PARAMS
    }

    dqn_finetune = Node(
        package='solution_ml',
        executable='dqn_finetune',
        name='dqn_finetune',
        output='screen',
        parameters=[node_params, {'use_sim_time': True}],
    )

    return LaunchDescription(declared_args + [sim_launch, dqn_finetune])
