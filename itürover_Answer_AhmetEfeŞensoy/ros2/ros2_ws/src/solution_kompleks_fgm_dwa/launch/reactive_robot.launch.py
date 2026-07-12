#!/usr/bin/env python3
"""
reactive_robot.launch.py — FGM + DWA hibriti (kompleks) — ROS 2.

ROS 1 reactive_robot.launch dosyasının ROS 2 karşılığı. Kapı-geçiş durum
makinesi (ALIGN/COMMIT), IFGM utility gap skorlama, hibrit FGM-I/DWA,
ağırlıklı arbiter, grid-BFS kapı ve SWEEP davranışları içeren en kapsamlı çözüm.

Simülasyon ground_truth:=true ile başlatılır (/ground_truth_odom telemetri;
kontrolde kullanılmaz). Tüm ayar değerleri buradan (launch argümanı) çekilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# (isim, varsayılan) — ROS 1 launch değerleriyle birebir. Python tipi (bool/int/
# float) parametrenin tipini belirler ve ParameterValue ile doğru tipe çevrilir.
_PARAMS = [
    # ── Core ─────────────────────────────────────────────────────────────
    ("max_vel",                      0.40),
    ("max_angular",                  2.84),
    ("v_min_forward",                0.24),
    ("gap_threshold",                0.30),
    ("min_gap_width",                0.40),
    ("predict_time",                 1.2),
    ("sim_dt",                       0.1),
    ("v_samples",                    10),
    ("w_samples",                    21),
    # ── DWA Weights ──────────────────────────────────────────────────────
    ("obstacle_weight",              1.30),
    ("potential_d0",                 1.80),
    ("heading_weight",               2.40),
    ("speed_weight",                 0.80),
    ("continuity_weight",            0.25),
    ("corridor_weight",              0.40),
    ("lateral_weight",               0.15),
    ("bearing_smoothing_alpha",      0.20),
    # ── Door-Commit (narrow-opening traversal state machine) ─────────────
    ("door_trigger_gap_width",       0.90),
    ("door_release_gap_width",       1.30),
    ("door_through_depth",           2.2),
    ("door_depth_margin",            1.3),
    ("door_min_light_width",         0.30),
    ("door_only_exit_frac",          0.15),
    ("door_align_speed",             0.60),
    ("door_align_gain",              1.8),
    ("door_align_max_w",             1.2),
    ("door_align_timeout",           5.0),
    ("door_retrigger_cooldown",      0.5),
    ("uturn_hard_deg",               90.0),
    ("door_trigger_streak",          6),
    ("door_release_streak",          5),
    ("door_commit_front_dist",       0.85),
    ("door_exit_front_dist",         1.05),
    ("door_commit_speed",            0.55),
    ("door_center_gain",             1.1),
    ("door_center_max_w",            1.2),
    ("door_max_lock_bearing_deg",    40.0),
    ("door_commit_max_bearing_deg",  25.0),
    ("door_commit_max_wz",           0.6),
    ("door_commit_timeout",          6.5),
    ("door_commit_max_world_off_deg", 65.0),
    ("door_commit_abort_off_deg",    78.0),
    ("door_stall_nudge_delay",       0.8),
    ("door_stall_front_dist",        0.25),
    ("door_stall_nudge_w",           0.9),
    # ── IFGM utility gap scoring ─────────────────────────────────────────
    ("gap_align_weight",             0.45),
    # ── Planner arm / hybrid ─────────────────────────────────────────────
    ("use_dwa",                      True),
    ("fgm_p_gain",                   2.1),
    ("hybrid_mode",                  True),
    ("hybrid_open_frac",             1.15),
    ("hybrid_close_frac",            0.90),
    ("use_arbiter",                  True),
    ("use_grid_door",                True),
    # ── SWEEP ────────────────────────────────────────────────────────────
    ("sweep_mode",                   True),
    ("sweep_speed",                  0.40),
    ("sweep_heading_deg",            75.0),
    ("sweep_timeout",                20.0),
    ("sweep_max_flips",              5),
    ("sweep_period",                 5.0),
    ("sweep_main_frac",              0.8),
    ("sweep_min_doors",              3),
    # ── Laser corridor centering ─────────────────────────────────────────
    ("center_bias_gain",             12.0),
    ("center_bias_max_deg",          16.0),
    # ── Gap-selection hysteresis ─────────────────────────────────────────
    ("gap_keep_bonus",               0.35),
    ("gap_keep_match_deg",           20.0),
    # ── Dynamic DWA weighting ────────────────────────────────────────────
    ("weight_decay_dist",            1.1),
    # ── Velocity governor ────────────────────────────────────────────────
    ("governor_full_dist",           1.6),
    ("governor_floor_dist",          0.5),
    # ── Conditional sprint ───────────────────────────────────────────────
    ("sprint_vel",                   0.62),
    ("sprint_front_dist",            1.4),
    ("sprint_bearing_deg",           14.0),
    ("sprint_max_w",                 0.5),
    # ── Accel & Robot ────────────────────────────────────────────────────
    ("max_linear_accel",             2.00),
    ("max_angular_accel",            8.00),
    ("robot_radius",                 0.12),
    ("run_duration",                 60.0),
    ("wedge_front_dist",             0.24),
    ("wedge_consecutive",            5),
    ("recover_seconds",              1.00),
    ("recover_linear",               -0.20),
]


def _default_str(value):
    """Python değerini launch-argümanı string'ine çevir.
    bool için ROS 2'nin beklediği küçük harfli 'true'/'false' üretilir."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def generate_launch_description():
    sim_share = get_package_share_directory('reactive_robot_sim')

    declared_args = [
        DeclareLaunchArgument(name, default_value=_default_str(default))
        for name, default in _PARAMS
    ]

    # ── Simülasyon ortamı (ödev: node ile aynı anda başlatılır) ──────────────
    # ground_truth:=true → /ground_truth_odom telemetri (kontrolde kullanılmaz).
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'reactive_robot_sim.launch.py')
        ),
        launch_arguments={
            'gui':          'true',
            'rviz':         'false',
            'model':        'burger_rgbd',
            'ground_truth': 'true',
        }.items(),
    )

    # ── Reaktif kontrolcü node'u ─────────────────────────────────────────────
    node_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=type(default))
        for name, default in _PARAMS
    }

    reactive_node = Node(
        package='solution_kompleks_fgm_dwa',
        executable='reactive_robot',
        name='reactive_robot',
        output='screen',
        parameters=[node_params, {'use_sim_time': True}],
    )

    return LaunchDescription(declared_args + [sim_launch, reactive_node])
