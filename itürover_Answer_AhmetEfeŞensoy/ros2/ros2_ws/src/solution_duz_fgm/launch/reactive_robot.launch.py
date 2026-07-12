#!/usr/bin/env python3
"""
reactive_robot.launch.py — Follow the Gap Method (FGM) + Disparity Extender (ROS 2).

ROS 1 reactive_robot.launch dosyasının ROS 2 karşılığı. Tek davranışlı temiz
reaktif kontrolcü (DWA / door-mode / memory yok).

Ödev gereği simülasyon ortamı ile node aynı launch dosyasından birlikte
başlatılır. Tüm ayar parametreleri buradan (launch argümanı olarak) çekilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Node parametreleri: (isim, varsayılan) — ROS 1 launch <arg>/<param> ile birebir.
# Varsayılanın Python tipi (float/int) parametrenin tipini belirler; böylece
# LaunchConfiguration'dan gelen string doğru tipe (ParameterValue ile) çözülür.
_PARAMS = [
    # ── Core ─────────────────────────────────────────────────────────────
    ("max_vel",                 0.65),
    ("max_angular",             2.84),
    ("v_min_forward",           0.20),
    ("gap_threshold",           0.30),
    ("min_gap_width",           0.40),
    ("robot_radius",            0.12),
    # ── Steering & speed shaping ─────────────────────────────────────────
    ("steer_gain",              1.4),
    ("bearing_smoothing_alpha", 0.50),
    ("gap_forward_bias",        0.02),
    ("slow_down_dist",          1.3),
    ("turn_slow_gain",          0.55),
    # ── Motion smoothing (slew-rate + EMA) ───────────────────────────────
    ("v_slew",                  0.55),
    ("w_slew",                  1.4),
    ("v_alpha",                 0.22),
    ("w_alpha",                 0.18),
    ("door_v_slew",             1.0),
    ("door_w_slew",             3.0),
    ("door_v_alpha",            0.35),
    ("door_w_alpha",            0.30),
    # ── Simple door-commit ───────────────────────────────────────────────
    ("door_gap_half_deg",       40.0),
    ("door_min_depth",          1.0),
    ("door_near_dist",          1.8),
    ("door_max_bearing_deg",    32.0),
    ("door_steer_gain",         1.0),
    ("door_commit_speed",       0.30),
    ("door_reach",              1.6),
    ("door_reached_dist",       0.5),
    ("door_exit_front",         1.0),
    ("door_min_progress",       1.3),
    ("door_stall_sec",          1.5),
    ("door_stall_min_move",     0.15),
    ("door_cooldown_sec",       0.8),
    # ── Wedge recovery ───────────────────────────────────────────────────
    ("wedge_front_dist",        0.20),
    ("wedge_consecutive",       4),
    ("recover_seconds",         0.60),
    ("recover_linear",          -0.14),
    ("recover_angular",         0.0),
    ("run_duration",            60.0),
]


def generate_launch_description():
    sim_share = get_package_share_directory('reactive_robot_sim')

    # Her parametre için bir launch argümanı bildir (komut satırından override edilebilir).
    declared_args = [
        DeclareLaunchArgument(name, default_value=default)
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
    # ROS 2'de parametreler tip-duyarlıdır: LaunchConfiguration string döndürür,
    # bu yüzden her değeri varsayılanın tipine (float/int) ParameterValue ile
    # açıkça çeviriyoruz. Aksi hâlde double bekleyen parametreye string atanır
    # ve node başlangıçta tip hatası verir.
    node_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
        for name, default in _PARAMS
    }

    reactive_node = Node(
        package='solution_duz_fgm',
        executable='reactive_robot',
        name='reactive_robot',
        output='screen',
        parameters=[node_params, {'use_sim_time': True}],
    )

    return LaunchDescription(declared_args + [sim_launch, reactive_node])
