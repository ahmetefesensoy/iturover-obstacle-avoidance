#!/usr/bin/env python3
"""
collect_data.py — Behavioural Cloning için expert veri toplama node'u.

fgm_dwa_robot.py (expert) ile birlikte çalıştırılır. /scan ve /cmd_vel'e abone
olur; her scan tick'inde, scan'den hesaplanan özellik vektörünü + expert'in
son yayınladığı cmd_vel'den türetilen en yakın ayrık aksiyon etiketini kaydeder.
Node kapanışında bir .npz veri seti kaydeder.

Kullanım: collect_data.launch üzerinden başlatılır (bu da ayrıca
fgm_dwa_robot.launch'ı başlatır). Expert'in kendi _shutdown() metodu
rospy.signal_shutdown çağırır, bu da roslaunch process grubu üzerinden yayılır
ve bu node'un on_shutdown hook'unu tetikler.

Ortaya çıkan .npz alanları:
  X          float32 (N, n_sectors + n_actions) — özellik vektörleri
  y          int64   (N,)                        — aksiyon etiketleri
  world_tag  str     — serbest metin dünya tanımlayıcısı
  n_sectors  int
  max_vel    float
  max_angular float
"""

import os
import sys
import rospy
import numpy as np
from datetime import datetime
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import feature_utils as fu


class DataCollector:

    def __init__(self):
        rospy.init_node("collect_data", anonymous=False)

        # ── Parametreler ──────────────────────────────────────────────────────
        self.n_sectors   = int(rospy.get_param("~n_sectors",   fu.N_SECTORS))
        self.max_vel     = float(rospy.get_param("~max_vel",   0.26))
        self.max_angular = float(rospy.get_param("~max_angular", 1.82))
        self.world_tag   = rospy.get_param("~world_tag", "obs_corridor_world")

        default_out = os.path.join(
            os.path.expanduser("~"),
            "rover_ml_data",
            f"{self.world_tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        )
        self.out_path = rospy.get_param("~out_path", default_out)
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)

        # ── Durum ─────────────────────────────────────────────────────────────
        self.last_cmd = (0.0, 0.0)          # /cmd_vel'den gelen son (v, w)
        # Önceki aksiyonu FORWARD olarak başlat (ACTIONS tablosunda indeks 2)
        self.prev_action_idx = fu.cmd_to_action_index(
            self.max_vel, 0.0, self.max_vel, self.max_angular)

        self.features = []                  # float32 (input_dim,) dizilerinin listesi
        self.labels   = []                  # int aksiyon indekslerinin listesi

        # ── ROS ───────────────────────────────────────────────────────────────
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_callback, queue_size=1)
        rospy.Subscriber("/scan",    LaserScan, self._scan_callback, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            f"[collect_data] Ready — world_tag='{self.world_tag}' "
            f"n_sectors={self.n_sectors} out_path={self.out_path}"
        )

    # ── Callback'ler ──────────────────────────────────────────────────────────

    def _cmd_callback(self, msg):
        self.last_cmd = (msg.linear.x, msg.angular.z)

    def _scan_callback(self, msg):
        # Ham scan'den sektör özellik vektörünü hesapla
        sectors = fu.scan_to_sectors(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.n_sectors
        )

        # Etiket: expert'in son cmd_vel'ini en yakın ayrık aksiyona eşle
        v, w = self.last_cmd
        action_idx = fu.cmd_to_action_index(v, w, self.max_vel, self.max_angular)

        # Tam özellik vektörünü oluştur (sektörler + önceki aksiyon one-hot)
        feat = fu.build_feature_vector(
            sectors, self.prev_action_idx, n_actions=len(fu.ACTIONS)
        )

        self.features.append(feat)
        self.labels.append(action_idx)
        self.prev_action_idx = action_idx

    # ── Kapanış ───────────────────────────────────────────────────────────────

    def _on_shutdown(self):
        n = len(self.labels)
        if n == 0:
            rospy.logwarn("[collect_data] No data collected — nothing saved.")
            return

        X = np.array(self.features, dtype=np.float32)
        y = np.array(self.labels,   dtype=np.int64)

        np.savez(
            self.out_path,
            X=X, y=y,
            world_tag=self.world_tag,
            n_sectors=np.int64(self.n_sectors),
            max_vel=np.float32(self.max_vel),
            max_angular=np.float32(self.max_angular),
        )
        rospy.loginfo(
            f"[collect_data] Saved {n} samples → {self.out_path}  "
            f"(X shape {X.shape}, action dist: "
            + ", ".join(
                f"{fu.ACTIONS[i][0]}={int((y == i).sum())}"
                for i in range(len(fu.ACTIONS))
            ) + ")"
        )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    DataCollector().run()
