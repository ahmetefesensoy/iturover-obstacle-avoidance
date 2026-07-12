#!/usr/bin/env python3
"""
collect_data.launch.py — Expert (FGM+DWA) demonstrasyonlarından BC verisi
toplama (ROS 2).

ROS 1'de reactive_robot paketindeki fgm_dwa_robot.launch include edilirdi;
ROS 2 portunda expert solution ayrı bir pakette (solution_fgm_dwa) olduğu
için o paketin launch dosyası include edilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fgm_dwa_share = get_package_share_directory('solution_fgm_dwa')

    n_sectors   = LaunchConfiguration('n_sectors')
    max_vel     = LaunchConfiguration('max_vel')
    max_angular = LaunchConfiguration('max_angular')
    world_tag   = LaunchConfiguration('world_tag')
    out_path    = LaunchConfiguration('out_path')

    declared_args = [
        DeclareLaunchArgument('n_sectors', default_value='36'),
        DeclareLaunchArgument('max_vel', default_value='0.26'),
        DeclareLaunchArgument('max_angular', default_value='1.82'),
        DeclareLaunchArgument('world_tag', default_value='obs_corridor_world'),
        DeclareLaunchArgument('out_path', default_value=''),
    ]

    # Expert (FGM+DWA) + sim ortamı — demonstrasyon verisini üreten sürücü.
    expert_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fgm_dwa_share, 'launch', 'reactive_robot.launch.py')
        ),
    )

    collect_data = Node(
        package='solution_ml',
        executable='collect_data',
        name='collect_data',
        output='screen',
        parameters=[{
            'n_sectors': ParameterValue(n_sectors, value_type=int),
            'max_vel': ParameterValue(max_vel, value_type=float),
            'max_angular': ParameterValue(max_angular, value_type=float),
            'world_tag': world_tag,
            'out_path': out_path,
        }],
    )

    return LaunchDescription(declared_args + [expert_launch, collect_data])
