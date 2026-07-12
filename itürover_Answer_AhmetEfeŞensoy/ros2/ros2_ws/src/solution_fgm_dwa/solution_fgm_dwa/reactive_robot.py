#!/usr/bin/env python3
"""
Çözüm 2 — FGM + DWA (Follow the Gap Method + Dynamic Window Approach)  (ROS 2 portu)
İTÜ Rover Engelden Kaçınma Ödevi

ROS 2 notları:
  * rospy → rclpy.node.Node (FgmDwaRobot artık Node alt-sınıfı)
  * get_param("~x", d) → declare_parameter + get_parameter (self._param yardımcısı)
  * rospy.Time.now().to_sec() → self._now() = get_clock().now().nanoseconds*1e-9
  * loginfo/logwarn → self.get_logger().info/warn
  * DeleteLight: rospy.ServiceProxy → self.create_client(DeleteLight, ...) + call_async
  * sub.unregister() → self.destroy_subscription(sub)
  * Süre dolunca rclpy.shutdown() ile çıkılır (roslaunch required=true yerine).
"""

import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteLight

# İleri arama penceresi: gap'ler yalnızca tam ileri yönden bu kadar derece
# sapmaya kadar aranır.
GAP_SEARCH_HALF_DEG = 100

# Merkezi, robotun mevcut yönünden bu değerden fazla derece sapan run'lar
# yanal/arka olarak kabul edilir ve sadece son çare olarak kullanılır (bkz.
# _best_gap_bearing_deg). Merkezden bu kadar uzak bir run neredeyse hiçbir
# zaman gerçek ileri yol değildir -- genellikle robot bir kapıya
# yaklaştığında kapı pervazının ya da yan duvarın en geniş açıklık gibi
# yanlış okunmasıdır ve bunun peşine düşmek gerçek gap'ten geçmek yerine
# robotu yerinde döndürür.
FORWARD_BEARING_LIMIT_DEG = 70

# COMMIT'e girişte dünya çerçevesinde dondurulan hedef nokta için sabit
# ileri bakış mesafesi (metre). Bilinçli olarak kısa ve sabit tutuldu --
# HERHANGİ bir LiDAR okumasından TÜRETİLMİYOR (gap_depth bir yöndeki en
# uzak dönüş değeridir, çoğu zaman kapının ötesindeki açık odadır ve küçük
# bir açısal sınırlamayı bu mesafeyle ölçeklendirmek bir keresinde hedefi
# çok yana fırlatmıştı). Kısa ve sabit bir erişim mesafesi, hedefi yakın
# tutar; böylece robot azıcık ilerlese bile hedefe olan bearing anlamlı
# şekilde değişir ve bu da hiçbir zaman ileri harekete birikmeyen anlık
# tarama açısının peşinden koşmak yerine gerçek kapalı-çevrim
# pure-pursuit düzeltmesi sağlar.
DOOR_COMMIT_REACH = 1.6


def _angle_diff(a: float, b: float) -> float:
    """a-b için en kısa işaretli fark, [-pi, pi] aralığına sarılmış."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def _linspace(lo: float, hi: float, n: int) -> list:
    if n <= 1 or hi <= lo:
        return [max(lo, min(hi, lo))]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


class FgmDwaRobot(Node):

    def __init__(self):
        super().__init__("reactive_robot")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.max_vel              = self._param("max_vel",              0.65)
        self.max_angular          = self._param("max_angular",          2.84)
        self.v_min_forward        = self._param("v_min_forward",        0.28)
        self.gap_threshold        = self._param("gap_threshold",        0.30)
        self.min_gap_width        = self._param("min_gap_width",        0.40)
        self.predict_time         = self._param("predict_time",         1.2)
        self.sim_dt               = self._param("sim_dt",               0.1)
        self.v_samples            = int(self._param("v_samples",        7))
        self.w_samples            = int(self._param("w_samples",        15))
        self.obstacle_weight      = self._param("obstacle_weight",      1.3)
        self.potential_d0         = self._param("potential_d0",         1.2)
        self.heading_weight       = self._param("heading_weight",       2.0)
        self.speed_weight         = self._param("speed_weight",         0.8)
        self.continuity_weight    = self._param("continuity_weight",    0.25)
        self.corridor_weight      = self._param("corridor_weight",      0.4)
        self.lateral_weight       = self._param("lateral_weight",       0.35)
        self.max_linear_accel     = self._param("max_linear_accel",     2.0)
        self.max_angular_accel    = self._param("max_angular_accel",    8.0)
        self.robot_radius         = self._param("robot_radius",         0.12)
        self.run_duration         = self._param("run_duration",         60.0)
        self.log_dir              = self._param("log_dir", os.path.expanduser("~/rover_run_logs"))
        self.bearing_smoothing_alpha = self._param("bearing_smoothing_alpha", 0.20)

        # Door-commit: run'ın geç bir aşamasında dar bir açıklıktan geçerken bearing'i dondur.
        self.door_trigger_displacement = self._param("door_trigger_displacement", 13.5)
        self.door_commit_front_dist    = self._param("door_commit_front_dist",    1.8)
        self.door_exit_front_dist      = self._param("door_exit_front_dist",      0.8)
        self.door_commit_speed         = self._param("door_commit_speed",         0.35)
        self.door_center_gain          = self._param("door_center_gain",          0.8)
        self.door_center_max_w         = self._param("door_center_max_w",         0.9)
        self.door_max_lock_bearing_deg = self._param("door_max_lock_bearing_deg", 15.0)
        self.door_commit_max_yaw_deg   = self._param("door_commit_max_yaw_deg",   45.0)
        self.door_max_target_yaw_deg   = self._param("door_max_target_yaw_deg",   35.0)
        self.door_commit_max_wz         = self._param("door_commit_max_wz",         0.6)
        self.door_commit_max_measured_wz = self._param("door_commit_max_measured_wz", 1.2)
        self.gap_forward_bias          = self._param("gap_forward_bias",          0.02)
        self.door_commit_timeout       = self._param("door_commit_timeout",       4.0)
        self.door_stall_check_sec      = self._param("door_stall_check_sec",      2.0)
        self.door_stall_min_progress   = self._param("door_stall_min_progress",   0.15)
        self.door_commit_confirm_ticks = int(self._param("door_commit_confirm_ticks", 3))

        # Obstacle memory: kapı pervazı konumlarını dünya çerçevesinde hatırla,
        # böylece robot LiDAR'ın 0.12m'lik kör noktasındayken (canlı taramanın
        # pervazı açık alan olarak okuduğu durum) bunlara göre ortalanabilsin.
        # Sadece door-commit direksiyon mantığı tarafından kullanılır.
        self.mem_min_range      = self._param("mem_min_range",      0.13)
        self.mem_max_range      = self._param("mem_max_range",      2.0)
        self.mem_ttl            = self._param("mem_ttl",            1.5)
        self.mem_center_trigger = self._param("mem_center_trigger", 0.9)
        self.mem_center_gain    = self._param("mem_center_gain",    1.2)
        self.mem_center_max     = self._param("mem_center_max",     0.5)
        self.obstacle_memory    = []  # (wx, wy, t) listesi

        # Wedge recovery: ön boşluk birkaç tick boyunca sabit kalırsa kısa süreliğine geri git.
        self.wedge_front_dist     = self._param("wedge_front_dist",     0.18)
        self.wedge_consecutive    = int(self._param("wedge_consecutive", 5))
        self.recover_seconds      = self._param("recover_seconds",      0.7)
        self.recover_linear       = self._param("recover_linear",       -0.13)
        self.wedge_count          = 0
        self.recover_until        = None

        # İstenen yaw = 0 (corridor yönü, robot odom çerçevesiyle hizalı olarak spawn olur)
        self.desired_yaw          = 0.0

        # ── Run durumu ────────────────────────────────────────────────────────
        self.start_time           = None
        self.scan_received        = False
        self.last_scan_time       = None
        self.cmd_linear_x         = 0.0
        self.cmd_angular_z        = 0.0
        self.last_label           = "INIT"
        self.smoothed_bearing_deg = 0.0

        # Door-commit durum makinesi: None | "ALIGN" | "COMMIT"
        self.door_mode               = None
        self.door_commit_start_time  = None
        self.door_target_x           = 0.0  # COMMIT girişinde dondurulan dünya çerçevesi noktası
        self.door_target_y           = 0.0
        self.door_commit_heading     = 0.0  # COMMIT girişinde kilitlenen heading, hedefe ulaşılınca yeniden projekte etmek için kullanılır
        self.door_commit_stall_xy    = None  # commit içi stall tespiti için (t, dist_to_target, path_length) anlık görüntüsü
        self.door_commit_recover_until = None
        self.door_commit_below_count = 0  # tüm commit koşullarının geçtiği ardışık tick sayısı
        self.last_commit_debug_log_t = None  # COMMIT-tick teşhis loglaması için throttle

        # Teşhis amaçlı (kontrolde kullanılmaz)
        self.have_odom            = False
        self.x = self.y = self.yaw = 0.0
        self.start_xy             = None
        self.prev_xy              = None
        self.prev_yaw             = 0.0
        self.prev_yaw_t           = None
        self.measured_yaw_rate    = 0.0
        self.path_length          = 0.0
        self.min_front_min_seen   = float("inf")
        self.wedge_trigger_count  = 0
        self.last_xy_log_t        = None

        # Kapanış guard'ı (rclpy.shutdown yalnızca bir kez çağrılsın).
        self._is_shutdown = False

        # ── Loglama ───────────────────────────────────────────────────────────
        self._open_log()

        # ── ROS ───────────────────────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data)

        self.get_logger().info("Node started.")
        self.get_logger().info("[FgmDwaRobot] ready — waiting for /scan …")

    # ── ROS 2 yardımcıları ─────────────────────────────────────────────────────

    def _param(self, name: str, default):
        """ROS 1 rospy.get_param("~name", default) kısayolunun ROS 2 karşılığı."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        """Geçerli zaman (saniye, float) — rospy.Time.now().to_sec() karşılığı."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Loglama ──────────────────────────────────────────────────────────────

    def _open_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"run_{stamp}.log")
        self._logf = open(log_path, "w", buffering=1)
        self._log(
            f"RUN START FGM-DWA | max_vel={self.max_vel} max_angular={self.max_angular} "
            f"v_min_forward={self.v_min_forward} gap_threshold={self.gap_threshold} "
            f"min_gap_width={self.min_gap_width} predict_time={self.predict_time} "
            f"sim_dt={self.sim_dt} v_samples={self.v_samples} w_samples={self.w_samples} "
            f"obstacle_w={self.obstacle_weight} heading_w={self.heading_weight} "
            f"speed_w={self.speed_weight} continuity_w={self.continuity_weight} "
            f"corridor_w={self.corridor_weight} "
            f"max_lin_accel={self.max_linear_accel} max_ang_accel={self.max_angular_accel} "
            f"robot_radius={self.robot_radius} wedge_front_dist={self.wedge_front_dist} "
            f"wedge_consecutive={self.wedge_consecutive} recover_seconds={self.recover_seconds} "
            f"recover_linear={self.recover_linear} "
            f"bearing_smoothing_alpha={self.bearing_smoothing_alpha} "
            f"door_trigger_disp={self.door_trigger_displacement} door_commit_front={self.door_commit_front_dist} "
            f"door_exit_front={self.door_exit_front_dist} door_commit_speed={self.door_commit_speed} "
            f"door_center_gain={self.door_center_gain} door_center_max_w={self.door_center_max_w}"
        )
        self.get_logger().info(f"[FgmDwaRobot] Log → {log_path}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logf.write(f"[{ts}] {msg}\n")

    # ── Odometry (yalnızca teşhis amaçlı) ────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw  = math.atan2(siny_cosp, cosy_cosp)
        now = self._now()

        if not self.have_odom:
            self.have_odom = True
            self.start_xy  = (self.x, self.y)
            self.prev_xy   = (self.x, self.y)
            self.prev_yaw  = self.yaw
            self.prev_yaw_t = now
            self.measured_yaw_rate = 0.0
            return

        self.path_length += math.hypot(self.x - self.prev_xy[0], self.y - self.prev_xy[1])
        self.prev_xy = (self.x, self.y)

        # Ölçülen (komut edilen değil) yaw rate. Commit koşulunun bilmesi
        # gereken şey robotun şu an GERÇEKTEN dönüp dönmediği, son komut
        # edilen w'nin küçük olup olmadığı değil -- bir kolon kenarına
        # takılıp dönen bir robotun komut edilen w'si taramadan taramaya
        # salınabilirken gerçek yaw 100+ derece/sn hızında sallanmaya devam
        # edebilir; bunu yalnızca komut edilen-w koşulu göremez.
        dt = now - self.prev_yaw_t
        if dt > 1e-3:
            self.measured_yaw_rate = _angle_diff(self.yaw, self.prev_yaw) / dt
        self.prev_yaw   = self.yaw
        self.prev_yaw_t = now

    # ── Tarama ön işleme ──────────────────────────────────────────────────────

    @staticmethod
    def _clean_ranges(ranges, range_min=0.12, range_max=10.0) -> list:
        out = []
        for r in ranges:
            if r != r or r == float("inf") or r < range_min or r > range_max:
                out.append(range_max)
            else:
                out.append(r)
        return out

    def _disparity_extend(self, ranges: list, radians_per_sample: float) -> list:
        """
        Disparity extender: yakın okumayı, robotun yarı genişliğini kapsamak
        için gereken açısal yay boyunca yayar; böylece dar gap'ler planlayıcı
        onları görmeden önce doğru şekilde geçilemez olarak işaretlenir.
        """
        n = len(ranges)
        extended = list(ranges)
        half_width = self.robot_radius * 1.1

        for i in range(n - 1):
            r1, r2 = ranges[i], ranges[i + 1]
            if abs(r1 - r2) <= self.gap_threshold:
                continue

            near = min(r1, r2)
            if near < 1e-3:
                continue

            arc_samples = int(math.ceil(half_width / (near * radians_per_sample)))
            arc_samples = min(arc_samples, n // 2)

            if r1 < r2:
                for k in range(1, arc_samples + 1):
                    idx = (i + k) % n
                    if extended[idx] > near:
                        extended[idx] = near
            else:
                for k in range(0, arc_samples):
                    idx = (i - k) % n
                    if extended[idx] > near:
                        extended[idx] = near

        return extended

    # ── FGM gap seçimi ────────────────────────────────────────────────────────

    def _best_gap_bearing_deg(self, extended: list, n: int) -> tuple:
        """
        İleri penceredeki (fiziksel yay uzunluğuna göre) en geniş geçilebilir
        run'ı bul ve içindeki en uzak noktayı hedefle.
        Hiçbir run min_gap_width'i geçemezse tek başına en uzak noktaya geri döner.
        """
        offsets = list(range(-GAP_SEARCH_HALF_DEG, GAP_SEARCH_HALF_DEG + 1))
        idxs = [int(o * n / 360) % n for o in offsets]
        navigable = [extended[i] > self.min_gap_width for i in idxs]

        runs = []
        run_start = None
        for pos, ok in enumerate(navigable):
            if ok and run_start is None:
                run_start = pos
            elif not ok and run_start is not None:
                runs.append((run_start, pos - 1))
                run_start = None
        if run_start is not None:
            runs.append((run_start, len(navigable) - 1))

        if not runs:
            best_pos = max(range(len(idxs)), key=lambda p: extended[idxs[p]])
            return float(offsets[best_pos]), extended[idxs[best_pos]], 0.0

        def physical_width(run):
            s, e = run
            angular_width_rad = math.radians(e - s)
            avg_range = sum(extended[idxs[p]] for p in range(s, e + 1)) / (e - s + 1)
            return angular_width_rad * avg_range

        def run_score(run):
            # En geniş run'ı tercih et, ama merkezi tam ileriden çok uzağa
            # sapan run'ları cezalandır. Bu bias olmadan FGM, yönden bağımsız
            # olarak fiziksel olarak en geniş açıklığı seçer; bu da hedefin
            # engeller geçtikçe yandan yana savrulmasına ve DWA'nın her
            # savrulmayı takip etmesine yol açar (corridor zigzag'inin kaynağı).
            s, e = run
            center_offset_deg = abs(offsets[(s + e) // 2])
            return physical_width(run) - self.gap_forward_bias * center_offset_deg

        # Merkezi robotun mevcut heading'ine yakın/arkasında olan run'ları
        # kesin olarak reddet (FORWARD_BEARING_LIMIT_DEG'in ötesinde). Yukarıdaki
        # yumuşak gap_forward_bias cezası skoru sadece hafifçe iter, dolayısıyla
        # geniş-ama-yanal bir run (örn. robot yaklaştığında en geniş açıklık
        # gibi yanlış okunan bir kapı pervazı) yine de doğrudan kazanabilir ve
        # DWA robotu bunun peşinden döndürür -- doğrudan gözlemlendi: yaw'ın
        # bir saniyeden kısa sürede ~130 derece sıçraması ve robotun daha
        # önce geçtiği bir koridorda geriye doğru sürülmesi. İleri bir run
        # varsa her zaman tercih edilir; reddedilen/yanal bir run'a yalnızca
        # başka hiçbir şey bulunamadığında geri dönülür, böylece robot
        # gerçekten yanlardan sıkıştığında kilitlenip kalmaz.
        forward_runs = [
            r for r in runs
            if abs(offsets[(r[0] + r[1]) // 2]) <= FORWARD_BEARING_LIMIT_DEG
        ]
        candidate_runs = forward_runs if forward_runs else runs

        best_run = max(candidate_runs, key=run_score)
        s, e = best_run
        best_pos = max(range(s, e + 1), key=lambda p: extended[idxs[p]])
        run_half_width_deg = (offsets[e] - offsets[s]) / 2.0
        return float(offsets[best_pos]), extended[idxs[best_pos]], run_half_width_deg

    # ── DWA yörünge maliyeti ──────────────────────────────────────────────────

    def _trajectory_cost(self, v: float, w: float, target_bearing_rad: float,
                          obstacles_cartesian: list, prev_w: float,
                          current_yaw: float = None, current_y: float = None) -> float:
        x = y = theta = 0.0
        steps = max(1, int(self.predict_time / self.sim_dt))
        min_clearance = float("inf")

        for step_idx in range(1, steps + 1):
            x += v * math.cos(theta) * self.sim_dt
            y += v * math.sin(theta) * self.sim_dt
            theta += w * self.sim_dt

            if step_idx % 2 == 0 or step_idx == steps:
                for ox, oy in obstacles_cartesian:
                    clearance = math.hypot(ox - x, oy - y) - (self.robot_radius + 0.03)
                    if clearance < min_clearance:
                        min_clearance = clearance

                if min_clearance <= 0.0:
                    break

        if min_clearance <= 0.0:
            obstacle_cost = 1e6 + abs(min_clearance) * 1e5
        elif min_clearance < self.potential_d0:
            obstacle_cost = 0.5 * self.obstacle_weight * (1.0 / min_clearance - 1.0 / self.potential_d0) ** 2
        else:
            obstacle_cost = 0.0

        heading_cost    = self.heading_weight * abs(_angle_diff(target_bearing_rad, theta))
        speed_cost      = self.speed_weight * (self.max_vel - v)
        continuity_cost = self.continuity_weight * abs(w - prev_w)

        if current_yaw is None:
            corridor_cost = 0.0
        else:
            # Corridor ekseninden sapma için her zaman aktif ceza (ölü bölge
            # yok). Önceki versiyon yalnızca 90 dereceyi aşan yaw'ı
            # cezalandırıyordu, dolayısıyla kapılar arasındaki açık alanda --
            # obstacle_cost genelde 0 iken -- DWA'yı 90+ dereceyi aşmış
            # gürültülü bir FGM bearing'ini takip etmekten hiçbir şey
            # alıkoymuyordu. Defalarca gözlemlendi: yaw'ın 100 -> 175 -> -145
            # -> -176 derece arasında dönmesi ve robotun toparlanmak yerine
            # ileri geri salınması, çünkü corridor_cost tüm süre boyunca
            # sıfıra yakın kalıyordu. Küçük, her zaman aktif bir kuadratik
            # terim, FGM'nin engellerden kaçınırken zaten yaptığı normal
            # +/-30-60 derecelik salınımlara neredeyse hiç maliyet
            # getirmez, ama yaw 90 dereceyi geçtiğinde düzleşmek yerine
            # büyümeye devam eder; böylece DWA'da her zaman onu corridor
            # eksenine geri çeken bir gradyan bulunur.
            yaw_diff = abs(_angle_diff(self.desired_yaw, current_yaw + theta))
            corridor_cost = self.corridor_weight * (yaw_diff / (math.pi / 2.0)) ** 2

        if current_y is None:
            lateral_cost = 0.0
        else:
            # Öngörülen yörüngenin dünya çerçevesindeki Y konumuna (spawn'a
            # göre) doğrudan ceza; yalnızca yaw'ı takip eden corridor_cost'tan
            # bağımsız olarak. Bir robot örneğin 50 derecelik kararlı,
            # salınımsız bir yaw'ı koruyabilir ve corridor_cost küçük kalır
            # (yaw_diff 90 derecenin çok altında) ama bu sırada dümdüz bir
            # çizgide corridor ekseninden birkaç metre uzaklaşabilir --
            # doğrudan gözlemlendi: y değeri ~8 saniyede 3.1 -> 5.6m'ye
            # tırmanırken yaw tüm süre boyunca 45-70 derece civarına
            # sabitlenmişti, ardından açıklığın kendisi y=0 civarındayken
            # bir kapı commit hedefi y=6.02'de kilitlenmişti. Öngörülen
            # yerel çerçeve uç noktasını (x,y) dünya çerçevesine döndür ve
            # robotun mevcut dünya çerçevesi Y'sine ekle.
            end_world_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
            lateral_cost = self.lateral_weight * end_world_y ** 2

        return obstacle_cost + heading_cost + speed_cost + continuity_cost + corridor_cost + lateral_cost

    @staticmethod
    def _state_label(front_min: float, robot_radius: float, v: float, max_vel: float) -> str:
        if front_min < robot_radius * 2.5:
            return "TIGHT"
        if v < max_vel * 0.6:
            return "AVOID"
        return "CRUISE"

    def _publish_recover(self, front_min: float):
        """Düz geri geri gitme, dönüş yok — DWA devam etmeden önce duvar temasını keser."""
        self.cmd_linear_x  = self.recover_linear
        self.cmd_angular_z = 0.0
        cmd = Twist()
        cmd.linear.x = self.recover_linear
        self.cmd_pub.publish(cmd)

        if self.last_label != "RECOVER":
            self.get_logger().info(f"[FgmDwaRobot] {self.last_label} → RECOVER  front_min={front_min:.2f}")
            self._log(f"TRANSITION {self.last_label} → RECOVER | front_min={front_min:.2f}")
            self.last_label = "RECOVER"

    def _memory_centering_bias(self):
        """Canlı tarama yerine HATIRLANAN pervaz konumlarını (self.obstacle_memory,
        son mem_ttl saniyede yakalanan dünya çerçevesi noktaları) kullanarak
        kapının ortasına doğru yönlendir. Obstacle memory'nin tüm amacı bu:
        robot açıklığa burnunu soktuğunda pervazlar LiDAR'ın 0.12m'lik kör
        noktasına düşer ve açık alan olarak okunur, dolayısıyla canlı-tarama
        ortalama terimi hiçbir şey üretmez ve robot bir pervaza sürüklenip
        döner (doğrudan gözlemlendi). Hatırlanan noktalar kör nokta boyunca
        dolu kalır, böylece burada yine de gerçek bir düzeltme sağlanır.

        Açısal hız bias'ı döndürür (rad/s), pozitif = sola dön.
        Loglama için (bias, left_off, right_off) döndürür; buradaki offsetler
        her iki taraftaki en yakın pervaza olan YANAL (dik) mesafelerdir --
        Öklid mesafesi DEĞİL. İlk versiyon Öklid mesafesi kullanıyordu ve
        robot bir pervazla hizaya geldiği anda bu değer ~0.02m'ye çöküyordu
        (pervaz robotun tam yanında olduğundan, robot mükemmel ortalanmış
        olsa bile düz çizgi mesafesi çok küçüktür); bu da her tick'te
        pin-guard'ı tetikleyip bias=0 üretiyordu. Ortalamada gerçekten
        önemli olan yanal offset'tir ve robot pervazlar arasından geçerken
        anlamlı kalır."""
        if not self.obstacle_memory:
            return 0.0, float("inf"), float("inf")

        # Her hatırlanan noktayı robotun heading çerçevesine projekte et:
        #   fwd = mevcut heading boyunca ileri mesafe
        #   lat = işaretli dik offset (+ = sol, - = sağ)
        # Sadece robotun hemen önünde/yanında bir banttaki (fwd [-0.2, 0.8] m
        # aralığında) ve yanal olarak lat_window içindeki pervazları tut;
        # böylece uzaktaki geometri değil, robotun içinden geçtiği açıklığın
        # duvarları seçilir.
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        left_off = float("inf")   # soldaki en yakın pervaz (en küçük +lat)
        right_off = float("inf")  # sağdaki en yakın pervaz (en küçük |lat|)
        lat_window = 0.6
        for wx, wy, _t in self.obstacle_memory:
            dx = wx - self.x
            dy = wy - self.y
            fwd = dx * c + dy * s
            lat = -dx * s + dy * c
            if fwd < -0.2 or fwd > 0.8:
                continue
            if abs(lat) > lat_window:
                continue
            if lat > 0.0:
                if lat < left_off:
                    left_off = lat
            else:
                if -lat < right_off:
                    right_off = -lat

        # Bir merkez tanımlamak için HER İKİ tarafta da hatırlanan bir pervaz gerekir.
        if left_off == float("inf") or right_off == float("inf"):
            return 0.0, left_off, right_off

        # Yalnızca açıklık en az bir tarafta gerçekten darsa harekete geç.
        if min(left_off, right_off) > self.mem_center_trigger:
            return 0.0, left_off, right_off

        # left_off < right_off ise sol pervaz daha yakın => robot merkezin
        # solunda => sağa dön (negatif). bias işareti: (left_off -
        # right_off) > 0 sağ pervaz daha yakın olduğunda => sola dön (+).
        diff = left_off - right_off
        bias = self.mem_center_gain * diff
        bias = max(-self.mem_center_max, min(self.mem_center_max, bias))
        return bias, left_off, right_off

    def _publish_door_commit(self, centering_bias: float = 0.0):
        """COMMIT girişinde dondurulan dünya çerçevesi noktasını pure-pursuit
        ile takip ederek commit edilmiş bir kapıdan sabit hızda geçiş.
        Her taramada yeniden hesaplanan bir bearing yerine sabit bir NOKTAYA
        yönelmek, bearing hatasının robot ilerledikçe gerçekten küçülmesi
        anlamına gelir -- tarama başına yeniden hesaplanan bir bearing,
        robotun ileri harekete hiç birikmeyen küçük yandan yana
        düzeltmeler yapmasına yol açıyordu (defalarca gözlemlendi:
        front_min bir kapının yakınında tüm commit timeout'u boyunca sabit
        kalırken net yer değiştirme sıfırdı, üstelik robot hiç duvara
        değmemişti). FGM/DWA engelden kaçınma maliyet fonksiyonu burada
        hâlâ tamamen atlanıyor -- bu bypass'ı gerektiren dönme döngülerini
        önleyen şey de zaten bu.

        centering_bias (rad/s, _memory_centering_bias'tan gelir), canlı
        taramanın pervazları göremediği LiDAR kör noktasındayken robotu
        kapı pervazlarından uzak tutmak için pure-pursuit direksiyonuna
        eklenir. Dondurulmuş hedef ileri hareketi korur; bias ise açıklık
        içindeki yanal konumu düzeltir."""
        dx = self.door_target_x - self.x
        dy = self.door_target_y - self.y
        desired_heading = math.atan2(dy, dx)
        heading_error = _angle_diff(desired_heading, self.yaw)
        w = max(-self.door_center_max_w,
                min(self.door_center_max_w,
                    heading_error * self.door_center_gain + centering_bias))

        # Nokta yana kaydığında yavaşla -- heading_error büyükken tam
        # door_commit_speed ile sürmek robotu, hedef çizgiye dönebileceğinden
        # daha hızlı ileri gönderir ve kapı kenarlarına çarpılır. Sıfıra
        # değil bir tabana kadar ölçeklenir, böylece düzeltirken de ilerlemeye devam eder.
        err_frac = min(1.0, abs(heading_error) / math.radians(self.door_max_lock_bearing_deg))
        speed_scale = 1.0 - 0.6 * err_frac
        v = self.door_commit_speed * speed_scale

        self.cmd_linear_x  = v
        self.cmd_angular_z = w
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _publish_door_align(self, target_bearing_rad: float, front_min: float):
        """DWA'nın atlandığı ALIGN-modu yaklaşımı: tam DWA maliyet aramasını
        çalıştırmak yerine basit bir oransal kontrolle doğrudan FGM gap
        bearing'ine doğru yönel. Deneysel -- fikir şu: kapı tespit edildikten
        sonra robot, DWA'nın her taramada obstacle/heading vb. ağırlıkları
        yeniden hesaplayıp yaklaşımı rayından çıkarmasına izin vermek yerine
        kararlı bir şekilde ona doğru sürülmeli. Hız, ön boşlukla
        sınırlandığından açıklığa yaklaşırken yine de yavaşlar."""
        w = max(-self.door_center_max_w,
                min(self.door_center_max_w, target_bearing_rad * self.door_center_gain))
        # Açıklık yaklaştıkça yavaşla; yaklaşım sırasında asla durmasın diye
        # bir ileri hız tabanı koru.
        if front_min >= 1.5:
            v = self.door_commit_speed
        else:
            v = max(self.v_min_forward, self.door_commit_speed * (front_min / 1.5))
        self.cmd_linear_x  = v
        self.cmd_angular_z = w
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # ── Ana tarama callback'i ─────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        if self._is_shutdown:
            return
        now = self._now()

        if not self.scan_received:
            self.scan_received = True
            self.start_time    = now
            self.get_logger().info("[FgmDwaRobot] First scan received. 60-second run started.")
            self._log("FIRST SCAN — timer started")

        elapsed = now - self.start_time
        if elapsed >= self.run_duration:
            self._shutdown()
            return

        n = len(msg.ranges)
        clean = self._clean_ranges(msg.ranges)
        front_idxs = [int(o * n / 360) % n for o in range(-20, 21)]
        front_min  = min(clean[i] for i in front_idxs)
        if front_min < self.min_front_min_seen:
            self.min_front_min_seen = front_min

        # Wedge recovery
        if self.recover_until is not None:
            if now < self.recover_until:
                self._publish_recover(front_min)
                return
            self.recover_until = None
            self.wedge_count = 0

        if front_min < self.wedge_front_dist:
            self.wedge_count += 1
        else:
            self.wedge_count = 0

        if self.wedge_count >= self.wedge_consecutive:
            self.recover_until = now + self.recover_seconds
            self.wedge_count   = 0
            self.wedge_trigger_count += 1
            self.obstacle_memory = []  # geri gitmek hatırlanan pervaz geometrisini geçersiz kılar
            self.get_logger().info(f"[FgmDwaRobot] Wedged — front_min={front_min:.2f}. Backing up.")
            self._log(f"RECOVER triggered | front_min={front_min:.2f}")
            self._publish_recover(front_min)
            return

        # ── Door-commit durum makinesi ────────────────────────────────────────
        # Wedge-recovery'den sonra (önceliği korur) ve DWA'dan önce çalışır.
        # COMMIT, FGM/DWA'yı tamamen atlayarak COMMIT girişinde dondurulan
        # bir dünya çerçevesi noktasını pure-pursuit ile takip eder (bkz.
        # _publish_door_commit). ALIGN ise akışa devam eder, böylece normal
        # FGM/DWA tamamen değişmeden çalışır.

        if self.door_mode is None and self.have_odom:
            net_disp = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
            if net_disp > self.door_trigger_displacement:
                self.door_mode = "ALIGN"
                self.get_logger().info(f"[FgmDwaRobot] Door-mode ALIGN engaged (net_disp={net_disp:.2f}m).")
                self._log(f"TRANSITION door_mode None -> ALIGN | net_disp={net_disp:.2f}")

        # Obstacle memory: bir kapıya yaklaşırken/geçerken, her yakın dönüşün
        # dünya çerçevesindeki konumunu hatırla; böylece door-commit birkaç
        # an sonra LiDAR'ın 0.12m'lik kör noktasına düşen pervazlara göre
        # ortalanabilsin (bkz. _memory_centering_bias). Sadece door_mode'da
        # aktiftir; açık koridorda hiçbir maliyeti yoktur ve normal FGM/DWA
        # sürüşünü etkileyemez.
        if self.door_mode is not None and self.have_odom:
            # Önce süresi dolmuş noktaları temizle (TTL penceresi).
            cutoff = now - self.mem_ttl
            if self.obstacle_memory:
                self.obstacle_memory = [p for p in self.obstacle_memory if p[2] >= cutoff]
            two_pi_over_n = 2.0 * math.pi / n
            for i in range(0, n, 2):  # alt örnekleme: bir atlayarak her ışın
                r = clean[i]
                if self.mem_min_range <= r <= self.mem_max_range:
                    ang = self.yaw + i * two_pi_over_n
                    wx = self.x + r * math.cos(ang)
                    wy = self.y + r * math.sin(ang)
                    self.obstacle_memory.append((wx, wy, now))

        radians_per_sample = 2.0 * math.pi / n
        extended = self._disparity_extend(clean, radians_per_sample)

        if self.door_mode == "COMMIT":
            committed_too_long = (
                self.door_commit_start_time is not None
                and now - self.door_commit_start_time > self.door_commit_timeout
            )
            if front_min > self.door_exit_front_dist or committed_too_long:
                self.door_mode = None
                self.door_commit_start_time = None
                self.door_commit_stall_xy = None
                self.door_commit_recover_until = None
                self.obstacle_memory = []  # kapı geçildi; eski pervazlar kalıntı olarak kalmamalı
                if committed_too_long:
                    # Güvenlik ağı: robot gerçekten bir kolon kenarına
                    # takılmışsa front_min süresiz olarak sabit kalabilir.
                    # Timeout, kontrolü FGM/DWA'ya geri verir; o da en azından
                    # LiDAR'ın şu an gördüklerine tepki verebilir.
                    self.get_logger().info("Door commit timed out, forcing resume.")
                    self._log(f"TRANSITION door_mode COMMIT -> None | TIMEOUT front_min={front_min:.2f}")
                else:
                    self.get_logger().info("Door cleared, resuming normal navigation.")
                    self._log(f"TRANSITION door_mode COMMIT -> None | front_min={front_min:.2f}")
                # akışa devam et: aynı taramada normal FGM/DWA'ya geri dön
            else:
                # Commit içi stall kurtarma: robot gerçekten bir kolon
                # kenarına takılmışsa front_min, tüm commit timeout'u
                # boyunca net yer değiştirme sıfırken wedge_front_dist'in
                # hemen üzerinde kalabilir (örn. 0.20-0.21m, TIGHT ama
                # wedged değil) -- dondurulmuş bir hedef noktaya geçtikten
                # sonra bile defalarca gözlemlendi, bu da robotun kötü bir
                # bearing'in peşinden koşmadığını, gerçekten sıkıştığını
                # gösteriyor. wedge-recovery'nin tek başına front_min
                # kontrolü bu bandı kaçırır. Kısa bir kayan pencerede ham
                # XY yer değiştirmesi yerine HEDEFE OLAN MESAFE kontrol
                # edilir. Ham yer değiştirmenin, robot sıkışmışken bile
                # ilerleme tabanının üzerinde kaldığı görüldü -- bir kolon
                # kenarı etrafında dönme/salınma, dondurulmuş hedefe hiç
                # yaklaşmadan gerçek mesafe kat eder, dolayısıyla düz bir
                # hypot(dx,dy) kontrolü hiç tetiklenmezdi. Hedefe olan
                # mesafe, ancak robot gerçekten gitmek istediği yere
                # yaklaştığında düşer.
                if self.door_commit_recover_until is not None and now < self.door_commit_recover_until:
                    self._publish_recover(front_min)
                    return
                self.door_commit_recover_until = None

                dist_to_target = math.hypot(self.door_target_x - self.x, self.door_target_y - self.y)

                # Bir noktaya doğru pure-pursuit, robot oraya neredeyse
                # ulaştığında bozulur -- sıfıra yakın bir vektörün atan2'si
                # sayısal olarak kararsızdır, dolayısıyla heading_error
                # çılgınca savrulur (doğrudan gözlemlendi: dist_to_target
                # 0.01m'ye düştü, ardından heading_err_deg iki tick içinde
                # -3'ten -63'e, oradan -128'e sıçradı; robot tam kendi
                # hedefine ulaştığı anda yerinde dönmeye başladı). Yakınlaştığında,
                # hedefi (şimdi bozulmuş olan mevcut pursuit vektöründen
                # değil) dondurulmuş COMMIT-girişi heading'i boyunca daha
                # ileriye it; böylece yönelinecek her zaman kararlı bir
                # nokta önde olur.
                if dist_to_target < 0.30:
                    self.door_target_x = self.x + DOOR_COMMIT_REACH * math.cos(self.door_commit_heading)
                    self.door_target_y = self.y + DOOR_COMMIT_REACH * math.sin(self.door_commit_heading)
                    dist_to_target = DOOR_COMMIT_REACH

                if self.door_commit_stall_xy is None:
                    self.door_commit_stall_xy = (now, dist_to_target, self.path_length)
                else:
                    t0, d0, path0 = self.door_commit_stall_xy
                    if now - t0 > self.door_stall_check_sec:
                        closed_in = d0 - dist_to_target
                        path_traveled = self.path_length - path0
                        # Hedefin yakınında dönme (örn. bir kolon köşesine
                        # kilitlenip dönme), dist_to_target neredeyse hiç
                        # değişmezken gerçek mesafe kat eder -- closed_in tek
                        # başına yavaş bir sürüklenmede RECOVER eşiğini
                        # ancak kıl payı kaçırır, ama closed_in'in birkaç
                        # katı kadar yol kat etmiş ama farkı kapatmamış bir
                        # robot açıkça daireler çiziyordur, yavaşça
                        # ilerlemiyordur. closed_in teknik olarak sıfırdan
                        # farklı olsa bile bu durumu işaretle.
                        circling = (
                            path_traveled > 3.0 * max(self.door_stall_min_progress, closed_in)
                            and closed_in < self.door_stall_min_progress * 2.0
                        )
                        if closed_in < self.door_stall_min_progress or circling:
                            self.door_commit_recover_until = now + self.recover_seconds
                            self.wedge_trigger_count += 1
                            self.get_logger().info(
                                f"[FgmDwaRobot] Door-commit stalled (closed_in={closed_in:.2f}m, "
                                f"path_traveled={path_traveled:.2f}m, circling={circling}). Backing up."
                            )
                            self._log(
                                f"RECOVER triggered (door-commit stall) | front_min={front_min:.2f} "
                                f"closed_in={closed_in:.2f} path_traveled={path_traveled:.2f} circling={circling}"
                            )
                            self.door_commit_stall_xy = None
                            self._publish_recover(front_min)
                            return
                        self.door_commit_stall_xy = (now, dist_to_target, self.path_length)

                centering_bias, mem_left, mem_right = self._memory_centering_bias()

                if self.last_commit_debug_log_t is None or now - self.last_commit_debug_log_t > 0.5:
                    self.last_commit_debug_log_t = now
                    dx_dbg = self.door_target_x - self.x
                    dy_dbg = self.door_target_y - self.y
                    heading_err_dbg = math.degrees(_angle_diff(math.atan2(dy_dbg, dx_dbg), self.yaw))
                    self._log(
                        f"COMMIT tick | xy=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f} "
                        f"heading_err_deg={heading_err_dbg:.1f} front_min={front_min:.2f} "
                        f"dist_to_target={dist_to_target:.2f}"
                    )
                    self._log(
                        f"COMMIT centering | mem_left={mem_left:.2f} mem_right={mem_right:.2f} "
                        f"bias={centering_bias:.2f} mem_pts={len(self.obstacle_memory)}"
                    )

                # COMMIT girişinde belirlenen dondurulmuş dünya çerçevesi
                # noktasına doğru yönel -- burada FGM yeniden hesaplaması
                # yok (bu state'in var olma amacı zaten bu bypass) -- ayrıca
                # robotu LiDAR kör noktasındaki pervazlardan uzak tutan
                # memory tabanlı bir ortalama bias'ı ekleniyor.
                self._publish_door_commit(centering_bias)
                return

        bearing_deg, gap_depth, gap_half_width_deg = self._best_gap_bearing_deg(extended, n)
        # Sadece FGM bearing hedefine alçak geçiren filtre uygula (DWA'nın
        # komut ettiği açısal hıza değil). bearing_deg, +/-100 derecelik
        # ileri arama penceresi (GAP_SEARCH_HALF_DEG) içinde düz bir derece
        # offset'idir, asla tam 0-360 derece bir heading değildir, dolayısıyla
        # asla +/-100 dereceyi aşıp sarmaz -- iki yakın ham okuma arasında
        # doğrusal EMA harmanlaması burada her zaman doğrudur, açı-sarma
        # (angle-wraparound) işlemine gerek yoktur (örneğin +179 ve -179
        # derece pusula yönlerini harmanlamanın aksine).
        self.smoothed_bearing_deg = (
            self.bearing_smoothing_alpha * bearing_deg
            + (1.0 - self.bearing_smoothing_alpha) * self.smoothed_bearing_deg
        )
        target_bearing_rad = math.radians(self.smoothed_bearing_deg)

        yaw_err = abs(_angle_diff(self.desired_yaw, self.yaw))
        commit_gates_pass = (
            self.door_mode == "ALIGN" and front_min < self.door_commit_front_dist
            and yaw_err < math.radians(self.door_commit_max_yaw_deg)
            and abs(self.cmd_angular_z) < self.door_commit_max_wz
            and abs(self.measured_yaw_rate) < self.door_commit_max_measured_wz
        )
        if self.door_mode == "ALIGN":
            # Commit koşullarının yalnızca bir değil, birkaç ARDIŞIK tarama
            # boyunca sağlanmasını iste. Tek bir şanslı/şanssız tarama
            # (LiDAR gürültüsü, anlık bir yaw sıçraması) daha önce tüm kapı
            # sonucunu belirliyordu -- iyi görünen tek bir tick'e göre
            # commit etmek, aynı kod ve parametrelerle run'dan run'a
            # çılgınca tutarsız sonuçlar üretiyordu (bir run'da 24m, bir
            # sonrakinde 11m). Birkaç tick üzerinden debounce yapmak,
            # dondurulmuş hedef noktanın gerçekten oturmuş bir yaklaşımdan
            # belirlenmesini sağlar; bu da gürültülü sensör verisinin tek
            # seferlik bir örneğinden çok daha tekrarlanabilirdir.
            self.door_commit_below_count = (
                self.door_commit_below_count + 1 if commit_gates_pass else 0
            )
            if front_min < self.door_commit_front_dist and not commit_gates_pass:
                self._log(
                    f"ALIGN gate blocked | front_min={front_min:.2f} yaw_err_deg={math.degrees(yaw_err):.1f} "
                    f"cmd_wz={self.cmd_angular_z:.2f} measured_wz={self.measured_yaw_rate:.2f} "
                    f"gap_half_width_deg={gap_half_width_deg:.0f}"
                )
        else:
            self.door_commit_below_count = 0

        if self.door_mode == "ALIGN" and self.door_commit_below_count >= self.door_commit_confirm_ticks:
            self.door_mode = "COMMIT"
            self.door_commit_start_time = now
            self.door_commit_below_count = 0
            max_lock_rad = math.radians(self.door_max_lock_bearing_deg)
            clamped_bearing = max(-max_lock_rad, min(max_lock_rad, target_bearing_rad))
            # _publish_door_commit'in pure-pursuit yapabilmesi için (yalnızca
            # bir heading değil) bir dünya çerçevesi hedef noktasını
            # dondur -- tarama başına bir bearing'e yönelmek, robotun ileri
            # harekete hiç birikmeyen küçük yandan yana düzeltmeler
            # yapmasına yol açıyordu (defalarca gözlemlendi: front_min tüm
            # commit timeout'u boyunca net yer değiştirme sıfırken
            # 0.20-0.21m'de sabit kaldı). Erişim mesafesi DOOR_COMMIT_REACH
            # ile sınırlıdır ama gap'in ölçülen kendi derinliğini de
            # aşamaz -- en derin örneği sabit erişimden daha sığ olan bir
            # run (örn. kapı yakın, uzak değil) aksi halde hedefi, run'ın
            # gerçekten geçilebilir olarak doğruladığı yerin ötesine,
            # doğrulanmamış bir alana yansıtırdı.
            commit_heading = self.yaw + clamped_bearing
            self.door_commit_heading = commit_heading
            reach = min(DOOR_COMMIT_REACH, max(0.5, gap_depth * 0.9))
            self.door_target_x = self.x + reach * math.cos(commit_heading)
            self.door_target_y = self.y + reach * math.sin(commit_heading)
            self.get_logger().info("Committing to door.")
            self._log(
                f"TRANSITION door_mode ALIGN -> COMMIT | front_min={front_min:.2f} "
                f"target_xy=({self.door_target_x:.2f},{self.door_target_y:.2f}) "
                f"reach={reach:.2f} gap_half_width_deg={gap_half_width_deg:.0f}"
            )
            commit_bias, _ml, _mr = self._memory_centering_bias()
            self._publish_door_commit(commit_bias)
            return

        # ALIGN-modu DWA bypass'ı (deneysel): kapıya yaklaşılırken, tam DWA
        # araması yerine basit oransal kontrolle doğrudan FGM gap
        # bearing'ine doğru sür; böylece DWA her taramada yeniden
        # ağırlıklandırıp rayından çıkmak yerine robot açıklığa kararlı
        # şekilde commit eder. Yalnızca ALIGN'de aktiftir; None-modu (normal
        # koridor) aşağıda hâlâ tam FGM/DWA kullanır.
        if self.door_mode == "ALIGN":
            self._publish_door_align(target_bearing_rad, front_min)
            label = self._state_label(front_min, self.robot_radius, self.cmd_linear_x, self.max_vel)
            if label != self.last_label:
                self._log(f"TRANSITION {self.last_label} → {label} (ALIGN-bypass) | front_min={front_min:.2f}")
                self.last_label = label
            if self.have_odom and (self.last_xy_log_t is None or now - self.last_xy_log_t >= 1.0):
                self.last_xy_log_t = now
                self._log(
                    f"XY t={elapsed:5.1f} x={self.x:.3f} y={self.y:.3f} "
                    f"yaw={math.degrees(self.yaw):.1f} state=ALIGN"
                )
            return

        if self.last_scan_time is None:
            dt = 0.1
        else:
            dt = max(0.02, min(0.2, now - self.last_scan_time))
        self.last_scan_time = now

        # Kartezyen engeller (hız için her 4. noktada bir alt örneklenmiş)
        obstacles_cartesian = []
        for i in range(0, n, 4):
            r = clean[i]
            if r < 3.5:
                angle = i * radians_per_sample
                obstacles_cartesian.append((r * math.cos(angle), r * math.sin(angle)))

        # Ön boşluğa bağlı dinamik hız sınırı
        if front_min >= 2.0:
            current_max_vel = self.max_vel
        elif front_min <= 0.5:
            current_max_vel = self.v_min_forward
        else:
            ratio = (front_min - 0.5) / 1.5
            current_max_vel = self.v_min_forward + ratio * (self.max_vel - self.v_min_forward)

        v_lo = max(self.v_min_forward, self.cmd_linear_x - self.max_linear_accel * dt)
        v_hi = min(current_max_vel,    self.cmd_linear_x + self.max_linear_accel * dt)
        if v_lo > v_hi:
            v_lo = v_hi = max(self.v_min_forward, min(current_max_vel, self.cmd_linear_x))

        w_lo = max(-self.max_angular, self.cmd_angular_z - self.max_angular_accel * dt)
        w_hi = min(self.max_angular,  self.cmd_angular_z + self.max_angular_accel * dt)

        v_candidates = _linspace(v_lo, v_hi, self.v_samples)
        w_candidates = _linspace(w_lo, w_hi, self.w_samples)

        best_cost = None
        best_v, best_w = self.v_min_forward, 0.0
        current_yaw = self.yaw if self.have_odom else None
        current_y   = (self.y - self.start_xy[1]) if self.have_odom else None
        for v in v_candidates:
            for w in w_candidates:
                cost = self._trajectory_cost(v, w, target_bearing_rad, obstacles_cartesian,
                                              self.cmd_angular_z, current_yaw, current_y)
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best_v, best_w = v, w

        self.cmd_linear_x  = best_v
        self.cmd_angular_z = best_w

        cmd = Twist()
        cmd.linear.x  = self.cmd_linear_x
        cmd.angular.z = self.cmd_angular_z
        self.cmd_pub.publish(cmd)

        label = self._state_label(front_min, self.robot_radius, best_v, self.max_vel)

        if label != self.last_label:
            self.get_logger().info(
                f"[FgmDwaRobot] {self.last_label} → {label}  "
                f"front_min={front_min:.2f} gap_bearing={bearing_deg:.0f}°"
            )
            self._log(
                f"TRANSITION {self.last_label} → {label} | front_min={front_min:.2f} "
                f"gap_bearing={bearing_deg:.0f} gap_depth={gap_depth:.2f}"
            )
            self.last_label = label

        if int(elapsed) % 5 == 0 and round(elapsed * 10) % 10 == 0:
            self._log(
                f"t={elapsed:5.1f}s  {label:<7} v={best_v:.2f} w={best_w:.2f} "
                f"gap_bearing={bearing_deg:.0f} gap_depth={gap_depth:.2f} front_min={front_min:.2f}"
            )

        if self.have_odom and (self.last_xy_log_t is None or now - self.last_xy_log_t >= 1.0):
            self.last_xy_log_t = now
            self._log(f"XY t={elapsed:5.1f} x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f} state={label}")

    # ── Kapatma ───────────────────────────────────────────────────────────────

    def _shutdown(self):
        if self._is_shutdown:
            return
        self._is_shutdown = True
        self.get_logger().info("Time limit reached. Stopping...")
        self._log("TIME LIMIT — stopping")

        if self.have_odom:
            net_disp = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
            self._log(
                f"SUMMARY net_displacement={net_disp:.2f}m path_length={self.path_length:.2f}m "
                f"min_front_min_seen={self.min_front_min_seen:.2f}m wedge_triggers={self.wedge_trigger_count} "
                f"final_xy=({self.x:.2f},{self.y:.2f}) final_yaw={math.degrees(self.yaw):.1f}"
            )
            self.get_logger().info(
                f"[FgmDwaRobot] SUMMARY net_disp={net_disp:.2f}m path={self.path_length:.2f}m "
                f"min_front={self.min_front_min_seen:.2f}m wedges={self.wedge_trigger_count}"
            )

        # /scan ve /odom aboneliklerini kapat (ödev: scan'den unsubscribe).
        self.destroy_subscription(self.scan_sub)
        self.destroy_subscription(self.odom_sub)
        self.get_logger().info("Unsubscribed from /scan.")
        self.cmd_pub.publish(Twist())

        # ── Gazebo 'sun' ışığını DeleteLight servisiyle sil (ödev gereği) ──
        # ROS 1 rospy.ServiceProxy karşılığı: create_client + call_async +
        # spin_until_future_complete ile senkron bekleme.
        try:
            self._delete_sun_light()
        except Exception as exc:
            self.get_logger().warn(f"[FgmDwaRobot] Could not delete light: {exc}")
            self._log(f"WARNING delete_light: {exc}")

        self._log("RUN END")
        self._logf.close()
        # roslaunch required=true yerine: rclpy'yi durdur (spin döngüsü biter).
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
            self.get_logger().info("Deleted light 'sun'.")
            self._log("Deleted 'sun' light")
        else:
            raise RuntimeError("delete_light call returned no result")


def main(args=None):
    rclpy.init(args=args)
    node = FgmDwaRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
