#!/usr/bin/env python3
"""
test_model.py — Eğitilmiş BC modelini BARN haritalarında simüle eder.
ROS gerektirmez, pure numpy ile çalışır.

Kullanım:
  python3 test_model.py                          # world_0'da test
  python3 test_model.py --world 42               # world_42'de test
  python3 test_model.py --custom                 # özel zor haritada test
"""

import argparse
import math
import os
import sys
import zipfile
import numpy as np

from reactive_robot_common import feature_utils as fu
from reactive_robot_common import barn_sim as bs
from reactive_robot_common.nn import MLP

# ── BARN geometrisi (core/barn_sim.py'ye devrediliyor, prepare_barn_data.py ile paylaşılıyor) ──
GRID_SIZE = bs.GRID_SIZE
CELL_SIZE = bs.CELL_SIZE
ORIGIN_X  = bs.ORIGIN_X
ORIGIN_Y  = bs.ORIGIN_Y
MAX_RANGE = bs.MAX_RANGE
N_RAYS    = bs.N_RAYS

grid_to_world       = bs.grid_to_world
world_to_grid_batch = bs.world_to_grid_batch
simulate_scan       = bs.simulate_scan


def simulate(model, grid, path, max_vel=0.26, max_angular=1.82, max_steps=2000):
    """Modeli haritada gezdir, katedilen mesafe ve çarpışma kontrolü yap."""
    start_row, start_col = path[0]
    robot_x, robot_y = grid_to_world(start_row, start_col)
    robot_yaw = 0.0

    prev_action_idx = fu.cmd_to_action_index(max_vel, 0.0, max_vel, max_angular)
    waypoint_idx = 1
    path_len = len(path)

    traj = [(robot_x, robot_y)]
    collisions = 0
    goal_reached = False
    total_dist = 0.0

    # Hedef (path'in son waypoint'i)
    goal_row, goal_col = path[-1]
    goal_x, goal_y = grid_to_world(goal_row, goal_col)
    goal_dist_thresh = CELL_SIZE * 2.0

    for step in range(max_steps):
        # Waypoint kontrolü
        if waypoint_idx < path_len:
            tr, tc = path[waypoint_idx]
            tx, ty = grid_to_world(tr, tc)
            if math.hypot(tx - robot_x, ty - robot_y) < CELL_SIZE * 0.8:
                waypoint_idx += 1

        # Hedefe ulaştık mı?
        if math.hypot(goal_x - robot_x, goal_y - robot_y) < goal_dist_thresh:
            goal_reached = True
            break

        # LIDAR + model inference
        ranges, a_min, a_inc, r_min, r_max = simulate_scan(grid, robot_x, robot_y, robot_yaw)
        sectors = fu.scan_to_sectors(ranges, a_min, a_inc, r_min, r_max, fu.N_SECTORS)
        feat = fu.build_feature_vector(sectors, prev_action_idx, n_actions=len(fu.ACTIONS))
        action_idx = model.predict_one(feat)
        name, v, w = fu.action_table(max_vel, max_angular, fu.ACTIONS)[action_idx]

        # Çarpışma kontrolü
        front_sectors = sectors[16:20]  # ön ~40°
        if np.min(front_sectors) < 0.03:
            collisions += 1
            if collisions > 5:
                break
        else:
            collisions = 0

        # Robotu hareket ettir
        dt = 0.2
        robot_yaw += w * dt
        robot_x += v * math.cos(robot_yaw) * dt
        robot_y += v * math.sin(robot_yaw) * dt
        total_dist += v * dt
        traj.append((robot_x, robot_y))
        prev_action_idx = action_idx

    return traj, total_dist, goal_reached, step


def print_stats(world_idx, traj, total_dist, goal_reached, steps):
    """İstatistikleri yazdır."""
    status = "[OK] HEDEFE ULASTI" if goal_reached else "[FAIL] HEDEFE ULASAMADI"
    print(f"  World {world_idx:3d}: {status} | mesafe={total_dist:.1f}m | adim={steps} | iz-uzunlugu={len(traj)}")


# ── ÖZEL ZOR HARİTA ──────────────────────────────────────────────────────────
def create_hard_maze():
    """
    30×30 grid'de zor bir labirent oluştur.
    Duvarlar ve dar koridorlar içerir.
    0 = boş, 1 = duvar
    """
    grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int64)

    # Dış duvarlar
    grid[0, :] = 1
    grid[-1, :] = 1
    grid[:, 0] = 1
    grid[:, -1] = 1

    # İç labirent duvarları
    # S-tipi dar koridor
    for i in range(2, 28, 4):
        grid[i, 5:25] = 1
        grid[i+1, 5:25] = 1
        # Geçit
        gate = 14 + (i % 8)
        grid[i, gate] = 0
        grid[i+1, gate] = 0

    # Dikey duvarlar
    for j in range(8, 24, 6):
        grid[4:26, j] = 1
        grid[4:26, j+1] = 1
        # Geçit
        gate_r = 8 + (j % 12)
        grid[gate_r, j] = 0
        grid[gate_r, j+1] = 0

    # Engel sütunları (rastgele)
    rng = np.random.RandomState(42)
    for _ in range(15):
        r, c = rng.randint(3, 27, 2)
        grid[r:r+2, c:c+2] = 1

    # Path (başlangıçtan bitişe)
    path = []
    # Başlangıç: sol alt
    r, c = 28, 2
    # Bitiş: sağ üst
    goal_r, goal_c = 2, 28

    # Basit bir yol: başlangıçtan hedefe doğru (labirenti takip ederek)
    path.append((r, c))
    # Önce yukarı
    while r > goal_r:
        r -= 1
        if grid[r, c] == 0:
            path.append((r, c))
    # Sonra sağa
    while c < goal_c:
        c += 1
        if grid[r, c] == 0:
            path.append((r, c))

    # Başlangıç ve bitiş hücrelerini temizle
    grid[28, 2] = 0
    grid[28, 3] = 0
    grid[27, 2] = 0
    grid[2, 28] = 0
    grid[2, 27] = 0
    grid[3, 28] = 0

    return grid, np.array(path)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="BC modelini BARN haritalarında test et")
    parser.add_argument("--weights", default="weights/bc_weights.npz",
                        help="Model ağırlıkları")
    parser.add_argument("--barn-zip", default="../../BARN_dataset.zip",
                        help="BARN_dataset.zip yolu")
    parser.add_argument("--world", type=int, default=None,
                        help="Tek bir world'de test et (default: 0-249 arası)")
    parser.add_argument("--world-step", type=int, default=1,
                        help="Hızlı alt küme için world indekslerini bu adımla örnekle "
                             "(örn. 5 → 0,5,10,...,245)")
    parser.add_argument("--custom", action="store_true",
                        help="Özel zor labirent haritasında test et")
    args = parser.parse_args()

    # Modeli yükle
    model = MLP.load(args.weights)
    print(f"Model yüklendi: {model.sizes}")
    print()

    if args.custom:
        # ── Özel zor harita ──
        print("=== ÖZEL ZOR LABİRENT HARİTASI ===")
        grid, path = create_hard_maze()
        print(f"Grid: {grid.shape}, occupied: {grid.sum()}/{GRID_SIZE*GRID_SIZE}")
        print(f"Path: {len(path)} waypoint")
        print(f"  Başlangıç: {path[0]} → Bitiş: {path[-1]}")
        print()

        # 5 kez dene (farklı seed'ler yok, aynı model)
        for trial in range(3):
            traj, dist, goal, steps = simulate(model, grid, path, max_steps=3000)
            print(f"  Deneme {trial+1}: ", end="")
            print_stats(-1, traj, dist, goal, steps)

        # Grid'i görselleştir
        print_grid(grid, path, traj if goal else None)
        print()

    else:
        # ── BARN haritaları ──
        print("=== BARN HARİTALARINDA TEST ===")
        z = zipfile.ZipFile(args.barn_zip)

        worlds = [args.world] if args.world is not None else range(0, 250, args.world_step)
        results = []

        for world_idx in worlds:
            try:
                grid = np.load(z.open(f"BARN_dataset/grid_files/grid_{world_idx}.npy")).astype(np.int64)
                path = np.load(z.open(f"BARN_dataset/path_files/path_{world_idx}.npy")).astype(np.int64)
            except KeyError:
                continue

            traj, dist, goal, steps = simulate(model, grid, path)
            results.append((world_idx, goal, dist, steps))
            print_stats(world_idx, traj, dist, goal, steps)

        z.close()

        # Özet
        successes = sum(1 for _, g, _, _ in results if g)
        print(f"\nÖzet: {successes}/{len(results)} başarılı "
              f"({100*successes/len(results):.0f}%)")


def print_grid(grid, path, traj=None):
    """Grid'i karakter bazlı görselleştir (konsolda)."""
    visual = np.full(grid.shape, '.', dtype=str)
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 1:
                visual[r, c] = '#'

    # Path
    for r, c in path:
        visual[r, c] = '·'

    # Trajectory
    if traj:
        for x, y in traj:
            # world → grid dönüşümü
            col = int((x - ORIGIN_X) / CELL_SIZE)
            row = int((y - ORIGIN_Y) / CELL_SIZE)
            if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
                if visual[row, col] not in ('#', 'S', 'G'):
                    visual[row, col] = '*'

    # Start/Goal
    sr, sc = path[0]
    gr, gc = path[-1]
    visual[sr, sc] = 'S'
    visual[gr, gc] = 'G'

    print(f"Harita ({GRID_SIZE}x{GRID_SIZE}):")
    print(f"  #=duvar  ·=path  *=robot izi  S=başlangıç  G=hedef")
    for r in range(GRID_SIZE):
        print('  ' + ''.join(visual[r, :]))


if __name__ == "__main__":
    main()