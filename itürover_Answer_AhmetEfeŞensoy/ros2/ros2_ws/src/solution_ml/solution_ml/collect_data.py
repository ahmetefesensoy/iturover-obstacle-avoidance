#!/usr/bin/env python3
"""
collect_data.py — Behavioural Cloning için expert veri toplama node'u (ROS 2).

fgm_dwa (expert) ile birlikte çalıştırılır. /scan ve /cmd_vel'e abone olur;
her scan tick'inde, scan'den hesaplanan özellik vektörünü + expert'in son
yayınladığı cmd_vel'den türetilen en yakın ayrık aksiyon etiketini kaydeder.
Node kapanışında bir .npz veri seti kaydeder.

ROS 2 notları:
  * rospy → rclpy.node.Node
  * get_param("~x") → declare_parameter + get_parameter
  * rospy.on_shutdown(fn) yerine main()'in finally bloğunda .npz kaydedilir
  * feature_utils artık reactive_robot_common paketinden import edilir

Ortaya çıkan .npz alanları:
  X          float32 (N, n_sectors + n_actions) — özellik vektörleri
  y          int64   (N,)                        — aksiyon etiketleri
  world_tag  str     — serbest metin dünya tanımlayıcısı
  n_sectors  int
  max_vel    float
  max_angular float
"""

import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from reactive_robot_common import feature_utils as fu


class DataCollector(Node):

    def __init__(self):
        super().__init__("collect_data")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.n_sectors   = int(self._param("n_sectors",   fu.N_SECTORS))
        self.max_vel     = float(self._param("max_vel",   0.26))
        self.max_angular = float(self._param("max_angular", 1.82))
        self.world_tag   = self._param("world_tag", "obs_corridor_world")

        default_out = os.path.join(
            os.path.expanduser("~"),
            "rover_ml_data",
            f"{self.world_tag}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        )
        # ROS 1'de <param if="out_path != ''"> ile boşsa hiç set edilmiyordu.
        # ROS 2'de parametre her zaman geçirilir; boş string'i "varsayılanı kullan"
        # olarak yorumluyoruz.
        out_path = self._param("out_path", default_out)
        self.out_path = out_path if out_path else default_out
        os.makedirs(os.path.dirname(self.out_path), exist_ok=True)

        # ── Durum ─────────────────────────────────────────────────────────────
        self.last_cmd = (0.0, 0.0)          # /cmd_vel'den gelen son (v, w)
        # Önceki aksiyonu FORWARD olarak başlat (ACTIONS tablosunda indeks 2)
        self.prev_action_idx = fu.cmd_to_action_index(
            self.max_vel, 0.0, self.max_vel, self.max_angular)

        self.features = []                  # float32 (input_dim,) dizilerinin listesi
        self.labels   = []                  # int aksiyon indekslerinin listesi

        # ── ROS ───────────────────────────────────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 1)
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"[collect_data] Ready — world_tag='{self.world_tag}' "
            f"n_sectors={self.n_sectors} out_path={self.out_path}"
        )

    def _param(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

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

    # ── Kaydetme (ROS 1 on_shutdown karşılığı; main finally'den çağrılır) ──────

    def save(self):
        n = len(self.labels)
        if n == 0:
            self.get_logger().warn("[collect_data] No data collected — nothing saved.")
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
        self.get_logger().info(
            f"[collect_data] Saved {n} samples → {self.out_path}  "
            f"(X shape {X.shape}, action dist: "
            + ", ".join(
                f"{fu.ACTIONS[i][0]}={int((y == i).sum())}"
                for i in range(len(fu.ACTIONS))
            ) + ")"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 1 on_shutdown hook'unun karşılığı: kapanışta veri setini kaydet.
        try:
            node.save()
        except Exception as exc:
            node.get_logger().error(f"[collect_data] save failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
