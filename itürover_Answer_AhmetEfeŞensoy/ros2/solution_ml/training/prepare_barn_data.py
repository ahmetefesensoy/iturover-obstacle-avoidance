#!/usr/bin/env python3
"""
prepare_barn_data.py — BARN dataset'inden BC eğitim verisi üretir.

Etiketler artık expert_policy.expert_action() (birleşik potansiyel-alan
controller) tarafından üretiliyor — eski 3-modlu (WAYPOINT/AVOID_SOFT/
AVOID_HARD) ayrık geçişli mantık kaldırıldı, çünkü o mantık hedefi
gözden kaybedip dar geçitlerde çarpışıyordu (bkz.
docs/superpowers/specs/2026-07-10-ml-expert-dagger-design.md).

V2 iyileştirmeleri (korunuyor):
  - Daha sık sample (dt=0.1sn, 2x daha fazla veri)
  - Waypoint'e yaklaşırken yavaşlama + keskin dönüş
  - Her adımda rastgele gürültü eklenmiş senaryo çeşitlemesi
"""

import argparse
import math
import os
import sys
import zipfile
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CORE_DIR = os.path.join(os.path.dirname(_SCRIPT_DIR), "core")
for d in [_SCRIPT_DIR, _CORE_DIR]:
    if d not in sys.path:
        sys.path.insert(0, d)

import feature_utils as fu
import barn_sim as bs
import expert_policy as ep

GRID_SIZE = bs.GRID_SIZE
CELL_SIZE = bs.CELL_SIZE
ORIGIN_X  = bs.ORIGIN_X
ORIGIN_Y  = bs.ORIGIN_Y
MAX_RANGE = bs.MAX_RANGE
N_RAYS    = bs.N_RAYS

grid_to_world      = bs.grid_to_world
world_to_grid_batch = bs.world_to_grid_batch
simulate_scan       = bs.simulate_scan

def process_world(grid, path, max_vel=0.26, max_angular=1.82, rng=None):
    if rng is None:
        rng = np.random.RandomState()

    features = []
    labels = []
    start_row, start_col = path[0]
    robot_x, robot_y = grid_to_world(start_row, start_col)
    robot_yaw = rng.uniform(-0.5, 0.5)  # Rastgele başlangıç yönü

    prev_action_idx = fu.cmd_to_action_index(max_vel, 0.0, max_vel, max_angular)
    waypoint_idx = 1
    path_len = len(path)
    dt = 0.1  # Daha sık sample
    
    # Hedef
    goal_row, goal_col = path[-1]
    goal_x, goal_y = grid_to_world(goal_row, goal_col)

    for step in range(path_len * 30):
        if waypoint_idx >= path_len:
            # Hedefe ulaştı mı kontrol et
            if math.hypot(goal_x - robot_x, goal_y - robot_y) < CELL_SIZE * 2:
                break
            # Son waypoint'te bekle
            target_row, target_col = path[-1]
        else:
            target_row, target_col = path[waypoint_idx]
        
        target_x, target_y = grid_to_world(target_row, target_col)
        dist_to_target = math.hypot(target_x - robot_x, target_y - robot_y)

        # Waypoint'e çok yaklaştık mı?
        if dist_to_target < CELL_SIZE * 0.6 and waypoint_idx < path_len:
            waypoint_idx += 1
            continue

        # Simule et + birleşik potansiyel-alan expert karar ver
        ranges, a_min, a_inc, r_min, r_max = simulate_scan(grid, robot_x, robot_y, robot_yaw)
        action_idx, sectors = ep.expert_action(
            robot_x, robot_y, robot_yaw, target_x, target_y,
            ranges, a_min, a_inc, r_min, r_max, max_vel, max_angular
        )

        feat = fu.build_feature_vector(sectors, prev_action_idx, len(fu.ACTIONS))
        features.append(feat)
        labels.append(action_idx)
        prev_action_idx = action_idx

        # Robotu hareket ettir
        _, v, w = fu.action_table(max_vel, max_angular, fu.ACTIONS)[action_idx]
        
        # Dönüş yapıyorsak hızı düşür
        if action_idx in [0, 1, 3, 4]:  # HARD_LEFT, LEFT, RIGHT, HARD_RIGHT
            v = v * 0.5
        
        robot_yaw += w * dt + rng.uniform(-0.05, 0.05)  # Fizik gürültüsü
        robot_x += v * math.cos(robot_yaw) * dt + rng.uniform(-0.01, 0.01)
        robot_y += v * math.sin(robot_yaw) * dt + rng.uniform(-0.01, 0.01)

        # Harita dışına çıktıysan dur
        row, col = world_to_grid_batch(np.array([robot_x]), np.array([robot_y]))
        if grid[row[0], col[0]] == 1:
            break

    if not features or len(features) < 20:
        return None, None

    return np.array(features, dtype=np.float32), np.array(labels, dtype=np.int64)


def create_hard_worlds(n_worlds=20):
    """n_worlds farklı zor senaryo üret."""
    worlds = []
    rng = np.random.RandomState(123)
    
    for seed in range(n_worlds):
        grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int64)
        grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 1
        
        # Rastgele engel sütunları (zorluk seviyesine göre)
        rr = np.random.RandomState(seed + 100)
        n_obstacles = 25 + (seed % 5) * 5  # 25-45 arası engel
        for _ in range(n_obstacles):
            r, c = rr.randint(3, 27, 2)
            h, w = rr.randint(1, 4, 2)
            grid[r:min(r+h, 29), c:min(c+w, 29)] = 1
        
        # Başlangıç ve bitiş
        grid[28, 2] = grid[28, 3] = grid[27, 2] = grid[2, 28] = grid[2, 27] = grid[3, 28] = 0
        
        # Path oluştur (basit)
        path = []
        r, c = 28, 2
        max_steps = 200
        for _ in range(max_steps):
            if r <= 5 and c >= 27:
                break
            # Rastgele ama akıllı hareket
            options = []
            if r > 2 and grid[r-1, c] == 0:
                options.append((-1, 0))
            if c < 28 and grid[r, c+1] == 0:
                options.append((0, 1))
            if r > 2 and c > 2 and grid[r-1, c-1] == 0:
                options.append((-1, -1))
            if r > 2 and c < 28 and grid[r-1, c+1] == 0:
                options.append((-1, 1))
            
            if not options:
                break
            dr, dc = options[rr.randint(len(options))]
            r += dr
            c += dc
            path.append((r, c))
        
        if len(path) > 15:
            worlds.append((grid, np.array(path)))
    
    return worlds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--barn-zip", default="BARN_dataset.zip")
    parser.add_argument("--out-dir", default=os.path.expanduser("~/rover_ml_data"))
    parser.add_argument("--worlds", type=int, nargs=2, default=[0, 299])
    parser.add_argument("--max-vel", type=float, default=0.26)
    parser.add_argument("--max-angular", type=float, default=1.82)
    parser.add_argument("--augment-runs", type=int, default=2,
                        help="Her world'ü kaç farklı seed ile çalıştır")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    rng = np.random.RandomState(42)
    total_samples = 0
    n_saved = 0

    print(f"BARN worlds {args.worlds[0]}–{args.worlds[1]}...")
    print(f"Her world: {args.augment_runs} run  |  Output: {args.out_dir}/")
    t0 = __import__('time').time()

    # 1. BARN world'leri
    z = zipfile.ZipFile(args.barn_zip)
    for world_idx in range(args.worlds[0], args.worlds[1] + 1):
        try:
            grid = np.load(z.open(f"BARN_dataset/grid_files/grid_{world_idx}.npy")).astype(np.int64)
            path = np.load(z.open(f"BARN_dataset/path_files/path_{world_idx}.npy")).astype(np.int64)
        except KeyError:
            continue

        for run in range(args.augment_runs):
            X, y = process_world(grid, path, args.max_vel, args.max_angular,
                                 rng=np.random.RandomState(world_idx * 10 + run))
            if X is None:
                continue

            tag = f"barn_{world_idx}" if run == 0 else f"barn_{world_idx}_r{run}"
            out_path = os.path.join(args.out_dir, f"{tag}.npz")
            np.savez(out_path, X=X, y=y, world_tag=tag,
                     n_sectors=np.int64(fu.N_SECTORS),
                     max_vel=np.float32(args.max_vel),
                     max_angular=np.float32(args.max_angular))
            n_saved += 1
            total_samples += len(y)

    z.close()

    # 2. Özel zor senaryolar
    print("\nÖzel zor senaryolar üretiliyor...")
    hard_worlds = create_hard_worlds(n_worlds=20)
    for i, (grid, path) in enumerate(hard_worlds):
        for run in range(5):
            X, y = process_world(grid, path, args.max_vel, args.max_angular,
                                 rng=np.random.RandomState(500 + i * 10 + run))
            if X is None:
                continue
            out_path = os.path.join(args.out_dir, f"hard_{i:02d}_r{run}.npz")
            np.savez(out_path, X=X, y=y, world_tag=f"hard_{i:02d}",
                     n_sectors=np.int64(fu.N_SECTORS),
                     max_vel=np.float32(args.max_vel),
                     max_angular=np.float32(args.max_angular))
            n_saved += 1
            total_samples += len(y)
            print(f"  hard_{i:02d}_r{run}: {len(y)} samples")

    elapsed = __import__('time').time() - t0
    print(f"\nDone! {n_saved} files, {total_samples} total samples in {elapsed:.0f}s")
    print(f"Output: {args.out_dir}/")


if __name__ == "__main__":
    main()