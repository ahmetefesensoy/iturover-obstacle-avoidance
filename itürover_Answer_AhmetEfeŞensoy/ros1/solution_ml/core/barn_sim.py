"""
barn_sim.py — BARN grid geometrisi + LiDAR tarama simülatörü.

test_model.py, prepare_barn_data.py ve expert_policy.py arasında paylaşılan
grid-fiziği burada tek yerde tutulur.  Yalnızca numpy kullanır — rospy
gerektirmez.
"""

import math
import numpy as np

GRID_SIZE = 30
CELL_SIZE = 0.15
ORIGIN_X  = -4.5
ORIGIN_Y  = 0.0
MAX_RANGE = 6.0
N_RAYS    = 360

_ray_angles = -math.pi + np.arange(N_RAYS) * (2.0 * math.pi / N_RAYS)
_ray_cos = np.cos(_ray_angles).astype(np.float64)
_ray_sin = np.sin(_ray_angles).astype(np.float64)


def grid_to_world(row, col):
    """Grid hücre indeksini (row, col) dünya koordinatına (x, y) çevir."""
    return ORIGIN_X + col * CELL_SIZE + CELL_SIZE / 2.0, ORIGIN_Y + row * CELL_SIZE + CELL_SIZE / 2.0


def world_to_grid_batch(wx, wy):
    """Dünya koordinat dizilerini (wx, wy) grid hücre indekslerine (row, col) çevir."""
    col = np.floor((wx - ORIGIN_X) / CELL_SIZE).astype(np.int64)
    row = np.floor((wy - ORIGIN_Y) / CELL_SIZE).astype(np.int64)
    row = np.clip(row, 0, GRID_SIZE - 1)
    col = np.clip(col, 0, GRID_SIZE - 1)
    return row, col


def simulate_scan(grid, robot_x, robot_y, robot_yaw):
    """
    Robot pozunda 360 ışınlı ray-cast LiDAR taraması simüle et.
    Döndürür: (ranges, angle_min, angle_increment, range_min, range_max)
    — LaserScan mesajıyla aynı imza, feature_utils.scan_to_sectors'a doğrudan verilebilir.
    """
    range_min = 0.02
    range_max = MAX_RANGE
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / N_RAYS

    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    dx_all = _ray_cos * cos_yaw - _ray_sin * sin_yaw
    dy_all = _ray_cos * sin_yaw + _ray_sin * cos_yaw

    step_size = CELL_SIZE * 0.5
    n_steps = int(math.ceil(range_max / step_size))
    steps = (np.arange(1, n_steps + 1, dtype=np.float64) * step_size).reshape(-1, 1)

    wx = robot_x + dx_all * steps
    wy = robot_y + dy_all * steps
    row, col = world_to_grid_batch(wx, wy)
    occupied = grid[row, col] == 1

    cum = np.cumsum(occupied.astype(np.int64), axis=0)
    first_hit = cum == 1
    hit_dist = np.where(first_hit, steps, range_max)
    ranges = hit_dist.min(axis=0).astype(np.float32)

    return ranges, angle_min, angle_increment, range_min, range_max


def is_occupied(grid, wx, wy):
    """Tek bir dünya koordinatının (wx, wy) işgal edilmiş bir hücrede olup olmadığını kontrol et."""
    row, col = world_to_grid_batch(np.array([wx]), np.array([wy]))
    return bool(grid[row[0], col[0]] == 1)
