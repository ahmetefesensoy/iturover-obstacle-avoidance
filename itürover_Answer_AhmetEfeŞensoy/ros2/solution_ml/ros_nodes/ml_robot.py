#!/usr/bin/env python3


import math
import os
import sys
import rospy
import numpy as np
from datetime import datetime
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteLight

# Kardeş modül import'ları (feature_utils.py, nn.py aynı scripts/ dizininde)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import feature_utils as fu
from nn import MLP


class MlRobot:

    def __init__(self):
        rospy.init_node("ml_robot", anonymous=False)

        # ── Parametreler ──────────────────────────────────────────────────────
        self.weights_path = rospy.get_param("~weights_path", "")
        self.n_sectors    = int(rospy.get_param("~n_sectors",    fu.N_SECTORS))
        self.max_vel      = float(rospy.get_param("~max_vel",    0.26))
        self.max_angular  = float(rospy.get_param("~max_angular", 1.82))
        self.run_duration = float(rospy.get_param("~run_duration", 60.0))
        self.log_dir      = rospy.get_param(
            "~log_dir",
            os.path.expanduser("~/rover_run_logs/solution_ml")
        )

        # Wedge-recovery sabitleri (fgm_dwa_robot.py varsayılanlarıyla aynı)
        self.wedge_front_dist  = float(rospy.get_param("~wedge_front_dist",  0.20))
        self.wedge_consecutive = int(rospy.get_param("~wedge_consecutive",   3))
        self.recover_seconds   = float(rospy.get_param("~recover_seconds",   0.7))
        self.recover_linear    = float(rospy.get_param("~recover_linear",   -0.13))

        # ── Eğitilmiş ağırlıkları yükle ───────────────────────────────────────
        if not self.weights_path or not os.path.isfile(self.weights_path):
            rospy.logfatal(
                f"[MlRobot] weights_path not set or file not found: "
                f"'{self.weights_path}'.  Pass ~weights_path to the launch file."
            )
            rospy.signal_shutdown("missing weights")
            return

        self.net = MLP.load(self.weights_path)
        rospy.loginfo(
            f"[MlRobot] Loaded weights from {self.weights_path}  "
            f"sizes={self.net.sizes}"
        )

        # Her ayrık aksiyon indeksi için gerçek (v, w) değerini önceden hesapla
        self._action_table = fu.action_table(self.max_vel, self.max_angular)

        # ── Çalışma durumu ────────────────────────────────────────────────────
        self.start_time    = None
        self.scan_received = False

        # Wedge-recovery durumu
        self.wedge_count   = 0
        self.recover_until = None
        self.wedge_trigger_count = 0

        # Önceki aksiyon indeksi (başlangıç = FORWARD, indeks 2)
        self.prev_action_idx = fu.cmd_to_action_index(
            self.max_vel, 0.0, self.max_vel, self.max_angular)

        self.last_label = "INIT"
        self.min_front_min_seen = float("inf")

        # ── Loglama ──────────────────────────────────────────────────────────
        self._open_log()

        # ── ROS ───────────────────────────────────────────────────────────────
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback, queue_size=1
        )
        rospy.loginfo("[MlRobot] Ready — waiting for /scan …")

    # ── Loglama ──────────────────────────────────────────────────────────────

    def _open_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"run_{stamp}.log")
        self._logf = open(log_path, "w", buffering=1)
        self._log(
            f"RUN START ML-ROBOT | weights={self.weights_path} "
            f"n_sectors={self.n_sectors} max_vel={self.max_vel} "
            f"max_angular={self.max_angular} wedge_front_dist={self.wedge_front_dist} "
            f"wedge_consecutive={self.wedge_consecutive} "
            f"recover_seconds={self.recover_seconds} recover_linear={self.recover_linear}"
        )
        rospy.loginfo(f"[MlRobot] Log → {log_path}")

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logf.write(f"[{ts}] {msg}\n")

    # ── Wedge recovery yardımcı fonksiyonu ───────────────────────────────────

    def _publish_recover(self, front_min):
        """Düz geri git — ağ devam etmeden önce duvar temasını kırar."""
        cmd = Twist()
        cmd.linear.x = self.recover_linear
        self.cmd_pub.publish(cmd)

        if self.last_label != "RECOVER":
            rospy.loginfo(f"[MlRobot] {self.last_label} → RECOVER  front_min={front_min:.2f}")
            self._log(f"TRANSITION {self.last_label} → RECOVER | front_min={front_min:.2f}")
            self.last_label = "RECOVER"

    # ── Ana scan callback ─────────────────────────────────────────────────

    def scan_callback(self, msg):
        now = rospy.Time.now().to_sec()

        # ── Zamanlayıcı ────────────────────────────────────────────────────
        if not self.scan_received:
            self.scan_received = True
            self.start_time    = now
            rospy.loginfo("[MlRobot] First scan received. 60-second run started.")
            self._log("FIRST SCAN — timer started")

        elapsed = now - self.start_time
        if elapsed >= self.run_duration:
            self._shutdown()
            return

        # ── Ön mesafe (0° etrafında ±20 indeks penceresi) ───────────────────
        n = len(msg.ranges)
        cleaned = fu.clean_ranges(msg.ranges, msg.range_min, msg.range_max)
        front_idxs = [int(o * n / 360) % n for o in range(-20, 21)]
        front_min  = float(min(cleaned[i] for i in front_idxs))
        if front_min < self.min_front_min_seen:
            self.min_front_min_seen = front_min

        # ── Wedge recovery (sert güvenlik önlemi) ───────────────────────────
        if self.recover_until is not None:
            if now < self.recover_until:
                self._publish_recover(front_min)
                return
            self.recover_until = None
            self.wedge_count   = 0

        if front_min < self.wedge_front_dist:
            self.wedge_count += 1
        else:
            self.wedge_count = 0

        if self.wedge_count >= self.wedge_consecutive:
            self.recover_until = now + self.recover_seconds
            self.wedge_count   = 0
            self.wedge_trigger_count += 1
            rospy.loginfo(f"[MlRobot] Wedged — front_min={front_min:.2f}. Backing up.")
            self._log(f"RECOVER triggered | front_min={front_min:.2f}")
            self._publish_recover(front_min)
            return

        # ── Ağ çıkarımı (inference) ──────────────────────────────────────────
        sectors = fu.scan_to_sectors(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.n_sectors
        )
        feat = fu.build_feature_vector(
            sectors, self.prev_action_idx, n_actions=len(fu.ACTIONS)
        )
        action_idx = self.net.predict_one(feat)
        name, v, w = self._action_table[action_idx]

        # cmd_vel yayınla
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        self.prev_action_idx = action_idx

        # ── Loglama (fgm_dwa_robot.py ile aynı etiket-değişim deseni) ───────
        if name != self.last_label:
            rospy.loginfo(
                f"[MlRobot] {self.last_label} → {name}  "
                f"front_min={front_min:.2f} v={v:.2f} w={w:.2f}"
            )
            self._log(
                f"TRANSITION {self.last_label} → {name} | "
                f"front_min={front_min:.2f} v={v:.2f} w={w:.2f}"
            )
            self.last_label = name

        # Her 5 saniyede bir periyodik durum kaydı
        if int(elapsed) % 5 == 0 and round(elapsed * 10) % 10 == 0:
            self._log(
                f"t={elapsed:5.1f}s  {name:<12} v={v:.2f} w={w:.2f} "
                f"front_min={front_min:.2f}"
            )

    # ── Kapanış ───────────────────────────────────────────────────────────────

    def _shutdown(self):
        rospy.loginfo("[MlRobot] Time limit reached. Stopping...")
        self._log("TIME LIMIT — stopping")
        self._log(
            f"SUMMARY min_front_min_seen={self.min_front_min_seen:.2f}m "
            f"wedge_triggers={self.wedge_trigger_count}"
        )

        self.scan_sub.unregister()
        self.cmd_pub.publish(Twist())   # hızı sıfırla

        try:
            rospy.wait_for_service("/gazebo/delete_light", timeout=3.0)
            rospy.ServiceProxy("/gazebo/delete_light", DeleteLight)("sun")
            rospy.loginfo("[MlRobot] Deleted 'sun' light from Gazebo.")
            self._log("Deleted 'sun' light")
        except Exception as exc:
            rospy.logwarn(f"[MlRobot] Could not delete light: {exc}")
            self._log(f"WARNING delete_light: {exc}")

        self._log("RUN END")
        self._logf.close()
        rospy.signal_shutdown("60 s complete.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    MlRobot().run()
