#!/usr/bin/env python3
"""
dagger_relabel.py — DAgger veri toplama: mevcut modeli BARN'da çalıştır,
model her adımda karar verirken aynı state'te expert'in ne yapacağını da
sorup etiketle. Üretilen (state, expert_action) çiftleri yeni bir .npz
dosyasına yazılır — mevcut BC eğitim setine eklenip yeniden eğitim için
kullanılır.

ROS gerektirmez, pure numpy. testing/test_model.py'nin simulate() mantığını
takip eder ama aksiyonu modelden alıp etiketi expert'ten üretir (klasik
DAgger: pi = model, etiketleyici = expert).

Kullanım:
  python3 dagger_relabel.py --weights weights/bc_weights_v9.npz \
      --barn-zip datasets/BARN_dataset.zip --world-step 5 \
      --out datasets_dagger/round1.npz
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
from nn import MLP


def rollout_and_relabel(model, grid, path, max_vel=0.26, max_angular=1.82, max_steps=2000):
    """
    Modeli haritada gezdir (aksiyonu MODEL seçer, trajectory model'e göre
    ilerler — DAgger'ın state-distribution mantığı budur). Her adımda,
    modelin gördüğü aynı state için expert'in ne yapacağını da hesaplayıp
    (feat, expert_action_idx) çiftini kaydet.

    Döndürür: features (list), expert_labels (list), goal_reached (bool)
    """
    start_row, start_col = path[0]
    robot_x, robot_y = bs.grid_to_world(start_row, start_col)
    robot_yaw = 0.0

    prev_action_idx = fu.cmd_to_action_index(max_vel, 0.0, max_vel, max_angular)
    waypoint_idx = 1
    path_len = len(path)

    goal_row, goal_col = path[-1]
    goal_x, goal_y = bs.grid_to_world(goal_row, goal_col)
    goal_dist_thresh = bs.CELL_SIZE * 2.0

    features = []
    expert_labels = []
    collisions = 0
    goal_reached = False

    for step in range(max_steps):
        if waypoint_idx < path_len:
            tr, tc = path[waypoint_idx]
            tx, ty = bs.grid_to_world(tr, tc)
            if math.hypot(tx - robot_x, ty - robot_y) < bs.CELL_SIZE * 0.8:
                waypoint_idx += 1

        if math.hypot(goal_x - robot_x, goal_y - robot_y) < goal_dist_thresh:
            goal_reached = True
            break

        target_row, target_col = path[min(waypoint_idx, path_len - 1)]
        target_x, target_y = bs.grid_to_world(target_row, target_col)

        ranges, a_min, a_inc, r_min, r_max = bs.simulate_scan(grid, robot_x, robot_y, robot_yaw)
        sectors = fu.scan_to_sectors(ranges, a_min, a_inc, r_min, r_max, fu.N_SECTORS)
        feat = fu.build_feature_vector(sectors, prev_action_idx, n_actions=len(fu.ACTIONS))

        # Expert etiketi: modelin gördüğü AYNI state için expert kararı
        expert_action_idx, _ = ep.expert_action(
            robot_x, robot_y, robot_yaw, target_x, target_y,
            ranges, a_min, a_inc, r_min, r_max, max_vel, max_angular
        )
        features.append(feat)
        expert_labels.append(expert_action_idx)

        # Trajectory modelin kendi kararına göre ilerler (DAgger state dist.)
        action_idx = model.predict_one(feat)
        _, v, w = fu.action_table(max_vel, max_angular, fu.ACTIONS)[action_idx]

        front_sectors = sectors[16:20]
        if np.min(front_sectors) < 0.03:
            collisions += 1
            if collisions > 5:
                break
        else:
            collisions = 0

        dt = 0.2
        robot_yaw += w * dt
        robot_x += v * math.cos(robot_yaw) * dt
        robot_y += v * math.sin(robot_yaw) * dt
        prev_action_idx = action_idx

    return features, expert_labels, goal_reached


def main():
    parser = argparse.ArgumentParser(description="DAgger relabeling: model rollout + expert labels")
    parser.add_argument("--weights", required=True, help="Mevcut BC modeli (.npz)")
    parser.add_argument("--barn-zip", default="datasets/BARN_dataset.zip")
    parser.add_argument("--world-step", type=int, default=5,
                        help="World indekslerini bu adımla örnekle (5 -> 0,5,10,...,245)")
    parser.add_argument("--out", required=True, help="Çıktı .npz yolu")
    parser.add_argument("--max-vel", type=float, default=0.26)
    parser.add_argument("--max-angular", type=float, default=1.82)
    args = parser.parse_args()

    model = MLP.load(args.weights)
    print(f"Model yuklendi: {model.sizes}")

    z = zipfile.ZipFile(args.barn_zip)
    all_features = []
    all_labels = []
    n_success = 0
    n_total = 0

    for world_idx in range(0, 250, args.world_step):
        try:
            grid = np.load(z.open(f"BARN_dataset/grid_files/grid_{world_idx}.npy")).astype(np.int64)
            path = np.load(z.open(f"BARN_dataset/path_files/path_{world_idx}.npy")).astype(np.int64)
        except KeyError:
            continue
        n_total += 1

        feats, labels, goal = rollout_and_relabel(
            model, grid, path, args.max_vel, args.max_angular
        )
        if goal:
            n_success += 1
        status = "OK" if goal else "FAIL"
        print(f"  World {world_idx:3d}: [{status}] {len(labels)} adim relabel edildi")

        all_features.extend(feats)
        all_labels.extend(labels)

    z.close()

    X = np.array(all_features, dtype=np.float32)
    y = np.array(all_labels, dtype=np.int64)

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    np.savez(args.out, X=X, y=y, world_tag="dagger_round",
              n_sectors=np.int64(fu.N_SECTORS),
              max_vel=np.float32(args.max_vel),
              max_angular=np.float32(args.max_angular))

    print(f"\nBaseline model basari: {n_success}/{n_total} ({100*n_success/n_total:.0f}%)")
    print(f"DAgger etiketleri kaydedildi -> {args.out}  ({len(y)} ornek)")


if __name__ == "__main__":
    main()
