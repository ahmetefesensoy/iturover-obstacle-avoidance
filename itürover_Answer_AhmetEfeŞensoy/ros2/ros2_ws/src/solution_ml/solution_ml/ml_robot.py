#!/usr/bin/env python3
"""ml_robot.py — Behavioural-Cloning ile eğitilmiş MLP politikasıyla reaktif
sürüş (ROS 2).

Her scan tick'inde LiDAR'dan sektör özellik vektörü çıkarılır, eğitilmiş MLP
en yakın ayrık aksiyonu seçer ve karşılık gelen (v, w) /cmd_vel'e yayınlanır.
Sert güvenlik için wedge-recovery (kısa geri gitme) korunur.

ROS 2 notları:
  * rospy → rclpy.node.Node
  * get_param("~x") → declare_parameter + get_parameter (self._param)
  * rospy.Time.now().to_sec() → self._now()
  * feature_utils / nn artık reactive_robot_common paketinden import edilir
  * DeleteLight: create_client + call_async; süre dolunca rclpy.shutdown()
"""

import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteLight

from reactive_robot_common import feature_utils as fu
from reactive_robot_common.nn import MLP


class MlRobot(Node):

    def __init__(self):
        super().__init__("ml_robot")
        self._is_shutdown = False
        self._ready = False

        # ── Parametreler ──────────────────────────────────────────────────────
        self.weights_path = self._param("weights_path", "")
        self.n_sectors    = int(self._param("n_sectors",    fu.N_SECTORS))
        self.max_vel      = float(self._param("max_vel",    0.26))
        self.max_angular  = float(self._param("max_angular", 1.82))
        self.run_duration = float(self._param("run_duration", 60.0))
        self.log_dir      = self._param(
            "log_dir",
            os.path.expanduser("~/rover_run_logs/solution_ml")
        )

        # Wedge-recovery sabitleri (fgm_dwa varsayılanlarıyla aynı)
        self.wedge_front_dist  = float(self._param("wedge_front_dist",  0.20))
        self.wedge_consecutive = int(self._param("wedge_consecutive",   3))
        self.recover_seconds   = float(self._param("recover_seconds",   0.7))
        self.recover_linear    = float(self._param("recover_linear",   -0.13))

        # ── Eğitilmiş ağırlıkları yükle ───────────────────────────────────────
        if not self.weights_path or not os.path.isfile(self.weights_path):
            self.get_logger().fatal(
                f"[MlRobot] weights_path not set or file not found: "
                f"'{self.weights_path}'.  Pass weights_path to the launch file."
            )
            # Yükleme yapılamadı: node oluşturulur ama abone olmaz; main
            # rclpy.ok() sonrası kapanır.
            rclpy.shutdown()
            return

        self.net = MLP.load(self.weights_path)
        self.get_logger().info(
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
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self._ready = True
        self.get_logger().info("[MlRobot] Ready — waiting for /scan …")

    # ── ROS 2 yardımcıları ─────────────────────────────────────────────────────

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

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
        self.get_logger().info(f"[MlRobot] Log → {log_path}")

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
            self.get_logger().info(f"[MlRobot] {self.last_label} → RECOVER  front_min={front_min:.2f}")
            self._log(f"TRANSITION {self.last_label} → RECOVER | front_min={front_min:.2f}")
            self.last_label = "RECOVER"

    # ── Ana scan callback ─────────────────────────────────────────────────

    def scan_callback(self, msg):
        if self._is_shutdown:
            return
        now = self._now()

        # ── Zamanlayıcı ────────────────────────────────────────────────────
        if not self.scan_received:
            self.scan_received = True
            self.start_time    = now
            self.get_logger().info("[MlRobot] First scan received. 60-second run started.")
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
            self.get_logger().info(f"[MlRobot] Wedged — front_min={front_min:.2f}. Backing up.")
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

        # ── Loglama (fgm_dwa ile aynı etiket-değişim deseni) ───────
        if name != self.last_label:
            self.get_logger().info(
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
        if self._is_shutdown:
            return
        self._is_shutdown = True
        self.get_logger().info("[MlRobot] Time limit reached. Stopping...")
        self._log("TIME LIMIT — stopping")
        self._log(
            f"SUMMARY min_front_min_seen={self.min_front_min_seen:.2f}m "
            f"wedge_triggers={self.wedge_trigger_count}"
        )

        self.destroy_subscription(self.scan_sub)
        self.cmd_pub.publish(Twist())   # hızı sıfırla

        try:
            self._delete_sun_light()
        except Exception as exc:
            self.get_logger().warn(f"[MlRobot] Could not delete light: {exc}")
            self._log(f"WARNING delete_light: {exc}")

        self._log("RUN END")
        self._logf.close()
        rclpy.shutdown()

    def _delete_sun_light(self):
        """/gazebo/delete_light servisini 'sun' argümanıyla çağır (senkron)."""
        client = self.create_client(DeleteLight, "/gazebo/delete_light")
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/gazebo/delete_light service unavailable")
        req = DeleteLight.Request()
        req.light_name = "sun"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info("[MlRobot] Deleted 'sun' light from Gazebo.")
            self._log("Deleted 'sun' light")
        else:
            raise RuntimeError("delete_light call returned no result")


def main(args=None):
    rclpy.init(args=args)
    node = MlRobot()
    # Ağırlık yükleme başarısızsa __init__ rclpy.shutdown() çağırmış olabilir.
    if rclpy.ok() and getattr(node, "_ready", False):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    try:
        node.cmd_pub.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
