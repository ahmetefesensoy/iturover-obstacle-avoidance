#!/usr/bin/env python3
"""
Çözüm 2 — Follow the Gap Method (FGM) + Disparity Extender
ITU Rover Engel Kaçınma (Obstacle Avoidance) Ödevi

Sade, tek davranışlı reaktif kontrolcü: her taramada (scan) önce LiDAR
verisini temizliyoruz, ardından robotun genişliğine göre dar boşlukların
(gap) doğru şekilde kapatılması için bir disparity extender çalıştırıyoruz,
ileri yöndeki en geniş geçilebilir boşluğu seçip basit bir oransal (proportional)
kontrolle bu boşluğun en derin noktasına doğru yönlendiriyoruz. Hız, ileri
yöndeki boşluğa (clearance) göre ölçekleniyor. Robot burnunu bir şeye
kıstırırsa (pin) küçük bir wedge-recovery devreye girip kısa süre geri
gidiyor. Kapı-modu (door-mode) durum makinesi yok, DWA araması yok, engel
belleği (obstacle memory) yok -- robot her kapıyı aynı şekilde ele alıp
referans demonun sürdüğü gibi ortalayarak geçiyor.
"""

import math
import os
import rospy
from datetime import datetime
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# İleri arama penceresi: boşluklar (gap) sadece tam ileri yönden bu kadar
# derece sapmaya kadar aranır.
GAP_SEARCH_HALF_DEG = 100

# Merkezi, tam ileri yönden bu dereceden fazla sapan run'lar yanal/arka
# olarak kabul edilir ve yalnızca son çare olarak kullanılır. Merkezden bu
# kadar uzak bir run neredeyse hiçbir zaman ileri yol değildir -- genellikle
# robot yaklaştığında en geniş açıklık sanılan bir kapı pervazı ya da yan
# duvardır -- ve bunun peşinden gitmek gerçek boşluğu (gap) geçmek yerine
# robotu döndürüp durmasına yol açar.
FORWARD_BEARING_LIMIT_DEG = 70


def _angle_diff(a: float, b: float) -> float:
    """a-b arasındaki en kısa işaretli fark, [-pi, pi] aralığına sarılmış (wrapped) halde."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class FgmRobot:

    def __init__(self):
        rospy.init_node("reactive_robot", anonymous=False)

        # ── Parametreler ──────────────────────────────────────────────────────
        self.max_vel        = rospy.get_param("~max_vel",        0.65)
        self.max_angular    = rospy.get_param("~max_angular",    2.84)
        self.v_min_forward  = rospy.get_param("~v_min_forward",  0.12)
        self.gap_threshold  = rospy.get_param("~gap_threshold",  0.30)
        self.min_gap_width  = rospy.get_param("~min_gap_width",  0.40)
        self.robot_radius   = rospy.get_param("~robot_radius",   0.12)

        # Direksiyon (steering): gap bearing üzerinde oransal kazanç (proportional gain),
        # max_angular ile sınırlandırılır (clamp).
        self.steer_gain     = rospy.get_param("~steer_gain",     1.6)
        # Gap bearing üzerinde EMA yumuşatma (0 = yok, 1 = donuk/frozen). Taramadan
        # taramaya oluşan titremeyi (jitter), gerçek dönüşlerde gecikmeye yol
        # açmadan hafifletmek için hafif bir filtre.
        self.bearing_smoothing_alpha = rospy.get_param("~bearing_smoothing_alpha", 0.35)
        self.gap_forward_bias = rospy.get_param("~gap_forward_bias", 0.02)

        # Hız şekillendirme: ileri koridor en az slow_down_dist kadar açıksa
        # tam hız; ön taraf daraldıkça v_min_forward'a doğru doğrusal olarak
        # düşer, dönüşlerde de yön hatasına (heading error) göre yavaşlanır.
        self.slow_down_dist   = rospy.get_param("~slow_down_dist",   2.0)
        self.turn_slow_gain   = rospy.get_param("~turn_slow_gain",   0.55)

        # Wedge recovery: ön boşluk (front clearance) birkaç tick boyunca kıstırılmış
        # (pinned) kalırsa kısa süreliğine geri gider.
        # Basit door-commit: dar ama derin bir boşluktan (gap) sapmak yerine
        # dosdoğru geçer. Tamamen geometriye dayalı olarak her taramada tetiklenir.
        self.door_gap_half_deg   = rospy.get_param("~door_gap_half_deg",   30.0)
        self.door_min_depth      = rospy.get_param("~door_min_depth",      1.0)
        self.door_near_dist      = rospy.get_param("~door_near_dist",      1.6)
        self.door_max_bearing_deg = rospy.get_param("~door_max_bearing_deg", 45.0)
        self.door_steer_gain     = rospy.get_param("~door_steer_gain",     1.0)
        self.door_commit_speed   = rospy.get_param("~door_commit_speed",   0.30)
        self.door_reach          = rospy.get_param("~door_reach",          1.6)
        self.door_reached_dist   = rospy.get_param("~door_reached_dist",   0.5)
        self.door_exit_front     = rospy.get_param("~door_exit_front",     1.5)
        self.door_min_progress   = rospy.get_param("~door_min_progress",   1.2)
        self.door_stall_sec      = rospy.get_param("~door_stall_sec",      1.5)
        self.door_stall_min_move = rospy.get_param("~door_stall_min_move", 0.15)
        # Bir commit bittikten sonra, robotun önce düz FGM altında mevcut gap
        # bearing'e dönüşünü gerçekten tamamlayabilmesi için kısa bir süre
        # yeniden tetiklenmeyi erteliyoruz. Bu olmazsa, dönüş ortasında biten
        # bir commit (bir sonraki kapı/duvar yaw hâlâ dönerken tekrar
        # door_shape olarak okunur) yeni bir heading'i robotun O ANKİ, henüz
        # dönmemiş yaw'ında dondurur -- böylece robot yanlış yöne kilitlenip
        # dönüşü tamamlamak yerine düz gider.
        self.door_cooldown_sec   = rospy.get_param("~door_cooldown_sec",   0.8)
        self.door_cooldown_until = None
        self.door_active         = False
        self.door_target_x       = 0.0
        self.door_target_y       = 0.0
        self.door_commit_heading = 0.0
        self.door_entry_xy       = (0.0, 0.0)
        self.door_stall_ref      = None

        self.wedge_front_dist = rospy.get_param("~wedge_front_dist", 0.20)
        self.wedge_consecutive = int(rospy.get_param("~wedge_consecutive", 4))
        self.recover_seconds  = rospy.get_param("~recover_seconds",  0.6)
        self.recover_linear   = rospy.get_param("~recover_linear",   -0.14)
        self.recover_angular  = rospy.get_param("~recover_angular",  0.0)

        # ── Hareket yumuşatma (yayınlanan v,w üzerinde slew-rate limiti + EMA) ──
        # Slew limitleri birim/saniye cinsindendir (v için m/s^2, w için rad/s^2).
        # Düşük değer = daha yumuşak/daha gecikmeli. Açık koridor limitleri sıkı
        # (çok yumuşak); door-commit limitleri ise pure-pursuit dönüşlerinin
        # keskin kalabilmesi için daha gevşektir.
        self.v_slew        = rospy.get_param("~v_slew",        0.9)
        self.w_slew        = rospy.get_param("~w_slew",        3.5)
        self.v_alpha       = rospy.get_param("~v_alpha",       0.35)
        self.w_alpha       = rospy.get_param("~w_alpha",       0.35)
        self.door_v_slew   = rospy.get_param("~door_v_slew",   2.0)
        self.door_w_slew   = rospy.get_param("~door_w_slew",   8.0)
        self.door_v_alpha  = rospy.get_param("~door_v_alpha",  0.6)
        self.door_w_alpha  = rospy.get_param("~door_w_alpha",  0.6)
        self.v_cmd_prev    = None
        self.w_cmd_prev    = None
        self.v_smoothed    = None
        self.w_smoothed    = None
        self._last_pub_t   = None

        self.run_duration   = rospy.get_param("~run_duration",   60.0)
        self.log_dir        = rospy.get_param("~log_dir", os.path.expanduser("~/rover_run_logs"))

        # ── Koşu (run) durumu ─────────────────────────────────────────────────
        self.start_time      = None
        self.scan_received   = False
        self.smoothed_bearing_deg = 0.0
        self.wedge_count     = 0
        self.recover_until   = None
        self.last_label      = "INIT"

        # Teşhis (diagnostics)
        self.have_odom       = False
        self.x = self.y = self.yaw = 0.0
        self.start_xy        = None
        self.prev_xy         = None
        self.path_length     = 0.0
        self.min_front_min_seen = float("inf")
        self.wedge_trigger_count = 0

        # ── Loglama ───────────────────────────────────────────────────────────
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(self.log_dir, f"run_{stamp}.log")
        self._logf = open(log_path, "w")
        self._log(
            f"RUN START FGM | max_vel={self.max_vel} max_angular={self.max_angular} "
            f"v_min_forward={self.v_min_forward} gap_threshold={self.gap_threshold} "
            f"min_gap_width={self.min_gap_width} robot_radius={self.robot_radius} "
            f"steer_gain={self.steer_gain} bearing_smoothing_alpha={self.bearing_smoothing_alpha} "
            f"slow_down_dist={self.slow_down_dist} turn_slow_gain={self.turn_slow_gain} "
            f"wedge_front_dist={self.wedge_front_dist} wedge_consecutive={self.wedge_consecutive} "
            f"recover_seconds={self.recover_seconds} recover_linear={self.recover_linear}"
        )
        rospy.loginfo(f"[FgmRobot] Log → {log_path}")

        # ── ROS G/Ç (I/O) ─────────────────────────────────────────────────────
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.on_shutdown(self._stop_robot)

    # ── Loglama ──────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logf.write(f"[{ts}] {msg}\n")
        self._logf.flush()

    # ── Odometry (sadece teşhis amaçlı) ───────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw  = math.atan2(siny_cosp, cosy_cosp)

        if not self.have_odom:
            self.have_odom = True
            self.start_xy  = (self.x, self.y)
            self.prev_xy   = (self.x, self.y)
            return

        self.path_length += math.hypot(self.x - self.prev_xy[0], self.y - self.prev_xy[1])
        self.prev_xy = (self.x, self.y)

    # ── Tarama (scan) ön işleme ───────────────────────────────────────────────

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
        Disparity extender: robotun yarı genişliğini (half-width) kapsayacak
        açısal yay (arc) boyunca yakın okumayı yayarak, dar boşlukların
        (gap) planlayıcı (planner) onları görmeden önce doğru şekilde
        geçilemez olarak işaretlenmesini sağlar.
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

    # ── FGM boşluk (gap) seçimi ───────────────────────────────────────────────

    def _best_gap_bearing_deg(self, extended: list, n: int) -> tuple:
        """
        İleri penceredeki (fiziksel yay uzunluğuna göre) en geniş geçilebilir
        run'ı bulur ve içindeki en uzak noktayı hedefler.
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
            # En geniş run'ı tercih et, ama merkezi tam ileri yönden uzak
            # olan run'ları hafifçe cezalandır; böylece hedef, engeller
            # geçerken sağa sola savrulmaz (koridor zigzag'inin kaynağı).
            s, e = run
            center_offset_deg = abs(offsets[(s + e) // 2])
            return physical_width(run) - self.gap_forward_bias * center_offset_deg

        # İleri yöndeki run'lara kesin öncelik ver; yalnızca hiç ileri run
        # yoksa yanal bir run'a geri dön. Böylece geniş ama yanal bir açıklık
        # (en geniş boşluk sanılan bir pervaz) doğrudan kazanıp robotu
        # döndüremez.
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

    # ── Kontrol ──────────────────────────────────────────────────────────────

    def _publish(self, v: float, w: float, label: str):
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        if label != self.last_label:
            self._log(f"STATE {self.last_label} -> {label} | v={v:.2f} w={w:.2f}")
            self.last_label = label

    def _publish_recover(self, front_min: float):
        cmd = Twist()
        cmd.linear.x  = self.recover_linear
        cmd.angular.z = self.recover_angular
        self.cmd_pub.publish(cmd)
        if self.last_label != "RECOVER":
            self._log(f"STATE {self.last_label} -> RECOVER | front_min={front_min:.2f}")
            self.last_label = "RECOVER"

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ── Ana tarama (scan) callback'i ──────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        now = rospy.Time.now().to_sec()

        if not self.scan_received:
            self.scan_received = True
            self.start_time    = now
            rospy.loginfo("[FgmRobot] First scan received. Run started.")
            self._log("FIRST SCAN — timer started")

        if now - self.start_time >= self.run_duration:
            self._shutdown()
            return

        n = len(msg.ranges)
        clean = self._clean_ranges(msg.ranges)

        # İleri boşluk (forward clearance) = tam ileri yöndeki +/-20 derecelik
        # koni içindeki en yakın dönüş (return).
        front_idxs = [int(o * n / 360) % n for o in range(-20, 21)]
        front_min  = min(clean[i] for i in front_idxs)
        if front_min < self.min_front_min_seen:
            self.min_front_min_seen = front_min

        # ── Wedge recovery ────────────────────────────────────────────────────
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
            self.door_active   = False  # geri gitmek dondurulmuş door hedefini geçersiz kılar
            rospy.loginfo(f"[FgmRobot] Wedged — front_min={front_min:.2f}. Backing up.")
            self._log(f"RECOVER triggered | front_min={front_min:.2f}")
            self._publish_recover(front_min)
            return

        # ── FGM: en geniş ileri boşluğu (gap) seç, en derin noktasını hedefle ─
        radians_per_sample = 2.0 * math.pi / n
        extended = self._disparity_extend(clean, radians_per_sample)
        bearing_deg, gap_depth, gap_half_width_deg = self._best_gap_bearing_deg(extended, n)

        # Bearing üzerinde hafif EMA (düz bir +/- derece ofseti, asla sarmalanmaz/wrap).
        self.smoothed_bearing_deg = (
            self.bearing_smoothing_alpha * bearing_deg
            + (1.0 - self.bearing_smoothing_alpha) * self.smoothed_bearing_deg
        )
        bearing_rad = math.radians(self.smoothed_bearing_deg)

        # ── Door-commit (dondurulmuş hedefli pure-pursuit) ────────────────────
        # Bir kapı, robotun yakınında DAR ama DERİN bir boşluk (gap) gibi
        # görünür: sadece birkaç derece genişliğinde (iki yanında pervazlar)
        # ama duvarın çok ötesine uzanan. Düz FGM böyle bir boşlukta sertçe
        # yavaşlayıp keskin dönüş yapar ("çok dar, sap geç"), bu yüzden
        # açıklığın etrafında döner durur -- ve burnunu içeri soksa bile,
        # boşluk uzak tarafta açıldığı anda FGM yeni bir en geniş boşluk seçip
        # robotu geri döndürür (gözlem: robot kapıyı geçerken front_min 1.6'ya
        # sıçrıyor, sonra robot GERİYE gidiyor). Çözüm, kapıyı algıladığımız
        # anda açıklığın tam ilerisinde dünya-çerçevesinde (world-frame) bir
        # nokta dondurmak ve ardından robot bu noktayı geçene kadar (her
        # taramadaki gap bearing'i değil) O sabit noktayı pure-pursuit ile
        # takip etmektir. Sabit bir nokta geçiş sırasında yeniden seçilip
        # değiştirilemeyeceğinden, robot geri dönmek yerine geçişi tamamlar.
        # Bu, eski karmaşık kontrolcünün durum makinesi / kapılar (gates) /
        # bellek / stall mekanizması olmadan gerçekten işe yarayan tek parçasıdır.
        door_shape = (
            gap_half_width_deg <= self.door_gap_half_deg
            and gap_depth >= self.door_min_depth
            and front_min <= self.door_near_dist
            and abs(self.smoothed_bearing_deg) <= self.door_max_bearing_deg
        )

        in_cooldown = (
            self.door_cooldown_until is not None and now < self.door_cooldown_until
        )

        if not self.door_active and not in_cooldown and door_shape and self.have_odom:
            # Commit heading'i ve gap bearing boyunca sabit bir mesafe (reach)
            # ilerideki hedef noktayı dondur; çıkmadan önce gerçek bir ileri
            # ilerleme (progress) şart koşabilmek için nereden girdiğimizi kaydet.
            self.door_commit_heading = self.yaw + bearing_rad
            self.door_target_x = self.x + self.door_reach * math.cos(self.door_commit_heading)
            self.door_target_y = self.y + self.door_reach * math.sin(self.door_commit_heading)
            self.door_entry_xy = (self.x, self.y)
            self.door_stall_ref = None
            self.door_active = True
            self._log(
                f"DOOR COMMIT start | target=({self.door_target_x:.2f},"
                f"{self.door_target_y:.2f}) reach={self.door_reach:.2f} "
                f"gap_depth={gap_depth:.2f} front_min={front_min:.2f}"
            )

        if self.door_active:
            dx = self.door_target_x - self.x
            dy = self.door_target_y - self.y
            dist_to_target = math.hypot(dx, dy)
            pursuit_bearing = _angle_diff(math.atan2(dy, dx), self.yaw)

            # Robot hedefe yaklaştığında, DONDURULMUŞ commit heading'i boyunca
            # hedefi daha da ileriye yeniden izdüşürür (re-project) -- robotun
            # "yaklaşıp sonra geri dönmesine" yol açan eksik parçaydı bu.
            # Bu olmadan, door-commit robot noktaya ulaştığı anda (dist <
            # reached), tam açıklığın ağzında SONA eriyordu ve FGM hemen yeni
            # bir boşluk seçip robotu geri döndürüyordu. Waypoint'i aynı
            # heading boyunca öteye itmek, kovalanacak sabit bir nokta
            # sağlıyor; böylece robot dosdoğru İÇERİDEN GEÇİP diğer tarafa çıkıyor.
            if dist_to_target <= self.door_reached_dist:
                self.door_target_x = self.x + self.door_reach * math.cos(self.door_commit_heading)
                self.door_target_y = self.y + self.door_reach * math.sin(self.door_commit_heading)
                dx = self.door_target_x - self.x
                dy = self.door_target_y - self.y
                dist_to_target = self.door_reach
                pursuit_bearing = _angle_diff(math.atan2(dy, dx), self.yaw)

            # Stall guard: robot bir pervaza sürtündüğünde LiDAR kör noktası
            # (blind spot) front_min'i ~0.2m'de kıstırır (pin), bu yüzden
            # normal çıkış testi (front >= door_exit_front) orada HİÇBİR ZAMAN
            # tetiklenmez -- robot da dondurulmuş heading boyunca düz gidip
            # duvara sürekli sürtünür (gözlem: yaw -112 derecede kilitli,
            # front 0.20'de takılı, net mesafe sürünerek ilerliyordu). Bunu
            # tespit et: robot door_stall_sec süresince door_stall_min_move
            # kadar ilerlemediyse door-commit'i iptal et ve geri gidip FGM'nin
            # yeniden yaklaşmasına izin veren wedge-recovery'ye devret.
            if self.door_stall_ref is None:
                self.door_stall_ref = (now, self.x, self.y)
            else:
                t0, sx, sy = self.door_stall_ref
                moved = math.hypot(self.x - sx, self.y - sy)
                if moved >= self.door_stall_min_move:
                    self.door_stall_ref = (now, self.x, self.y)
                elif now - t0 >= self.door_stall_sec:
                    self.door_active = False
                    self.door_stall_ref = None
                    self.door_cooldown_until = now + self.door_cooldown_sec
                    self._log(f"DOOR COMMIT abort (stalled) | front_min={front_min:.2f} moved={moved:.2f}")

            # Girişten itibaren gerçek bir mesafe kat ettiğimizde ya da
            # commit hattından umutsuzca saptığımızda çık. Artık tetikleyici
            # yalnızca mesafe -- front_min DEĞİL. Daha önce front_min>=
            # door_exit_front de gerekiyordu, ama LiDAR kör noktası dar bir
            # açıklığın tüm geçişi boyunca front_min'i 0.15-0.2m civarında
            # kıstırdığından bu koşul neredeyse hiç sağlanmıyor ve robot
            # dondurulmuş heading boyunca kapıyı çoktan geçip karşı tarafta
            # ne varsa ona doğru düz gitmeye devam ediyordu (doğrudan gözlem:
            # "ilk delikten girip dümdüz duvara çarpıyor"). door_min_progress
            # kat edildiğinde kapı artık geride demektir, front_min o anda ne
            # okursa okusun -- bu yüzden koşulsuz olarak kontrolü FGM'ye geri ver.
            progressed = math.hypot(self.x - self.door_entry_xy[0],
                                    self.y - self.door_entry_xy[1])
            if self.door_active and (progressed >= self.door_min_progress
                    or abs(pursuit_bearing) > math.radians(100.0)):
                self.door_active = False
                self.door_stall_ref = None
                self.door_cooldown_until = now + self.door_cooldown_sec
                self._log(
                    f"DOOR COMMIT end | progressed={progressed:.2f} "
                    f"front_min={front_min:.2f}"
                )

        if self.door_active:
            w = self.door_steer_gain * pursuit_bearing
            w = max(-self.max_angular, min(self.max_angular, w))
            v = self.door_commit_speed
            label = "DOOR"
        else:
            # ── Direksiyon: gap bearing ile orantılı ──────────────────────────
            w = self.steer_gain * bearing_rad
            w = max(-self.max_angular, min(self.max_angular, w))

            # ── Hız: ileri boşlukla ölçeklen, dönüşlerde ek yavaşlama ─────────
            clear_frac = min(1.0, front_min / self.slow_down_dist)
            turn_frac  = 1.0 - self.turn_slow_gain * min(1.0, abs(bearing_rad) / (math.pi / 2.0))
            v = self.max_vel * clear_frac * turn_frac
            v = max(self.v_min_forward, v)
            label = "CRUISE" if front_min >= self.slow_down_dist else "AVOID"

        is_door = self.door_active

        # ── Hareket yumuşatma: v ve w üzerinde slew-rate limiti + EMA ─────────
        # İki aşama hareketi, takılmadan (stall) pürüzsüz gösterir:
        #   1. Slew-rate limiti -- v ve w'nin her tick'te ne kadar değişebileceğini
        #      sınırlar, böylece tekerlekler her hız/dönüş değişimine aniden
        #      atlamak yerine yumuşakça geçer (görsel olarak en büyük kazanç
        #      burada; gerçek ivmelenme limitleri gibi davranır).
        #   2. Hafif EMA -- slewing sonrası kalan küçük artık titremeyi (jitter)
        #      yuvarlar.
        # Durum ilk komuttan (0'dan değil) beslenir, böylece robot çıkışta
        # sürünmez. Door-commit, kasıtlı pure-pursuit dönüşünün düzleşmemesi
        # için daha gevşek (daha duyarlı) limitler kullanır.
        if self.v_cmd_prev is None:
            self.v_cmd_prev = v
            self.w_cmd_prev = w
            self.v_smoothed = v
            self.w_smoothed = w

        dt = now - self._last_pub_t if self._last_pub_t is not None else 0.1
        dt = max(1e-3, min(0.5, dt))
        self._last_pub_t = now

        v_slew = self.door_v_slew if is_door else self.v_slew
        w_slew = self.door_w_slew if is_door else self.w_slew
        va     = self.door_v_alpha if is_door else self.v_alpha
        wa     = self.door_w_alpha if is_door else self.w_alpha

        # Aşama 1: hedef komuta doğru slew-rate limiti uygula.
        v_step = max(-v_slew * dt, min(v_slew * dt, v - self.v_cmd_prev))
        w_step = max(-w_slew * dt, min(w_slew * dt, w - self.w_cmd_prev))
        self.v_cmd_prev += v_step
        self.w_cmd_prev += w_step

        # Aşama 2: slew uygulanmış komutun üzerine hafif EMA.
        self.v_smoothed = va * self.v_cmd_prev + (1.0 - va) * self.v_smoothed
        self.w_smoothed = wa * self.w_cmd_prev + (1.0 - wa) * self.w_smoothed

        self._publish(self.v_smoothed, self.w_smoothed, label)

        # Sınırlandırılmış (throttled) telemetri (~2 Hz).
        if not hasattr(self, "_last_tick_log") or now - self._last_tick_log > 0.5:
            self._last_tick_log = now
            elapsed = now - self.start_time
            net = 0.0
            if self.start_xy is not None:
                net = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
            self._log(
                f"t={elapsed:5.1f}s {label} v={v:.2f} w={w:.2f} "
                f"gap_bearing={self.smoothed_bearing_deg:.0f} gap_depth={gap_depth:.2f} "
                f"gap_half_w={gap_half_width_deg:.0f} front_min={front_min:.2f} "
                f"door={int(is_door)} "
                f"xy=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.0f} net={net:.2f}"
            )

    def _shutdown(self):
        self._stop_robot()
        net = 0.0
        if self.start_xy is not None:
            net = math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
        self._log(
            f"SUMMARY net_displacement={net:.2f}m path_length={self.path_length:.2f}m "
            f"min_front_min_seen={self.min_front_min_seen:.2f}m "
            f"wedge_triggers={self.wedge_trigger_count} "
            f"final_xy=({self.x:.2f},{self.y:.2f}) final_yaw={math.degrees(self.yaw):.1f}"
        )
        rospy.loginfo(
            f"[FgmRobot] Run complete. net_disp={net:.2f}m path={self.path_length:.2f}m "
            f"wedges={self.wedge_trigger_count}"
        )
        self._logf.close()
        rospy.signal_shutdown("run_duration reached")


def main():
    node = FgmRobot()
    rospy.spin()


if __name__ == "__main__":
    main()
