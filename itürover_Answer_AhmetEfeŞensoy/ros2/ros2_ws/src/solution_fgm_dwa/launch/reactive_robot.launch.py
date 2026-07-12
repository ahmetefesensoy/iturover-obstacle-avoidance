#!/usr/bin/env python3
"""
reactive_robot.launch.py — FGM + DWA (ROS 2).

ROS 1 reactive_robot.launch dosyasının ROS 2 karşılığı.
Checkpoint config: ~14.20m net yer değiştirme, 0 wedge (canlı koşuyla doğrulandı).

Ödev gereği simülasyon ortamı ile node aynı launch dosyasından birlikte
başlatılır; tüm ayar değerleri buradan (launch argümanı) çekilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# (isim, varsayılan) — ROS 1 launch değerleriyle birebir. Python tipi (float/int)
# parametrenin tipini belirler ve ParameterValue ile doğru tipe çevrilir.
_PARAMS = [
    # ── Core ─────────────────────────────────────────────────────────────
    ("max_vel",                     0.65),
    ("max_angular",                 2.84),
    ("v_min_forward",               0.28),
    ("gap_threshold",               0.30),
    ("min_gap_width",               0.40),
    ("predict_time",                1.2),
    ("sim_dt",                      0.1),
    ("v_samples",                   7),
    ("w_samples",                   15),
    # ── DWA Weights ──────────────────────────────────────────────────────
    ("obstacle_weight",             1.30),
    ("potential_d0",                1.80),
    ("heading_weight",              1.50),
    ("speed_weight",                0.80),
    ("continuity_weight",           1.00),
    ("corridor_weight",             0.40),
    ("lateral_weight",              0.35),
    ("bearing_smoothing_alpha",     0.20),
    # ── Door-Commit ──────────────────────────────────────────────────────
    ("door_trigger_displacement",   13.5),
    ("door_commit_front_dist",      1.10),
    ("door_exit_front_dist",        1.45),
    ("door_commit_speed",           0.48),
    ("door_center_gain",            0.8),
    ("door_center_max_w",           0.9),
    ("door_max_lock_bearing_deg",   15.0),
    ("door_commit_max_yaw_deg",     45.0),
    ("door_max_target_yaw_deg",     35.0),
    ("door_commit_max_wz",          0.6),
    ("door_commit_max_measured_wz", 1.2),
    ("gap_forward_bias",            0.02),
    ("door_commit_timeout",         10.0),
    ("door_stall_check_sec",        2.0),
    ("door_stall_min_progress",     0.15),
    ("door_commit_confirm_ticks",   3),
    # ── Obstacle Memory ──────────────────────────────────────────────────
    ("mem_min_range",               0.13),
    ("mem_max_range",               2.0),
    ("mem_ttl",                     0.8),
    ("mem_center_trigger",          0.9),
    ("mem_center_gain",             1.2),
    ("mem_center_max",              0.5),
    # ── Accel & Robot ────────────────────────────────────────────────────
    ("max_linear_accel",            2.00),
    ("max_angular_accel",           8.00),
    ("robot_radius",                0.12),
    ("run_duration",                60.0),
    ("wedge_front_dist",            0.18),
    ("wedge_consecutive",           5),
    ("recover_seconds",             0.70),
    ("recover_linear",              -0.13),
]


def generate_launch_description():
    sim_share = get_package_share_directory('reactive_robot_sim')

    declared_args = [
        DeclareLaunchArgument(name, default_value=str(default))
        for name, default in _PARAMS
    ]

    # ── Simülasyon ortamı (ödev: node ile aynı anda başlatılır) ──────────────
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

    # ── Reaktif kontrolcü node'u ─────────────────────────────────────────────
    node_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
        for name, default in _PARAMS
    }

    reactive_node = Node(
        package='solution_fgm_dwa',
        executable='reactive_robot',
        name='reactive_robot',
        output='screen',
        parameters=[node_params, {'use_sim_time': True}],
    )

    return LaunchDescription(declared_args + [sim_launch, reactive_node])
