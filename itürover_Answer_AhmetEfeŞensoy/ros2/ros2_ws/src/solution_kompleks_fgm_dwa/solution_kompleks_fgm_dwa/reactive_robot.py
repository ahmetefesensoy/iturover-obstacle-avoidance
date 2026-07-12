#!/usr/bin/env python3
"""
Çözüm 2 — FGM + DWA hibriti (Follow the Gap Method + Dynamic Window
Approach) — İTÜ Rover Engelden Kaçınma Ödevi

ELI5 özet: Robotun tek gözü LiDAR (lazerle mesafe ölçen, çepeçevre
tarayan sensör). FGM = "önümdeki en uygun boşluğun ortasına doğru dön".
DWA = "olası (hız, dönüş) ikililerini kafamda kısa süre simüle edeyim,
çarpmayan ve hedefe iyi bakan adayı seçeyim". Dar kapılar içinse ayrı
bir kapı-geçiş durum makinesi var: ALIGN (kapıya hizalan) -> COMMIT
(kararlılıkla içinden geç).
"""

import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteLight

# ROS 2 portu: rospy → rclpy.node.Node. get_param→self._param, Time.now→self._now,
# loginfo/logwarn→get_logger(), ServiceProxy→create_client, spin/shutdown→rclpy.

# İleri arama penceresi: boşluk (gap) ararken sadece burnun +/- bu kadar
# derecelik konisine bak. NEDEN: deneme 47'de 100->85'e daraltıldı; +/-100
# iken seçici, yalpalama sırasında en uçtaki yan boşluklara atlıyordu
# (loglarda bearing +/-100). Her yan atlayış robotu hattan yana sürükler,
# ortalayıcı mekanizma (centering bias) da bununla sürekli kavga eder.
GAP_SEARCH_HALF_DEG = 85

# COMMIT sırasında yeniden nişan alma penceresi: kapıya zaten hizalıyken
# geçilebilir boşluğu yalnızca bu koni içinde ara; böylece S-geçidin
# (şikan) ikinci kıvrımı, duvarın yanındaki açıklıkla karıştırılmaz.
# NEDEN: deneme 21'de 40->60'a genişletildi; S-kanallarda "ışık" geçiş
# ortasında +/-40'ın dışına kayıyor ve commit kör kalıp ölüyordu
# (deneme 20: 12 commit fallback, 3/3 timeout).
DOOR_COMMIT_GAP_HALF_DEG = 60


def _angle_diff(a: float, b: float) -> float:
    """a-b açı farkını en kısa yönden, [-pi, pi] aralığına sarıp verir."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# _linspace: lo'dan hi'ye n adet eşit aralıklı değer üretir (numpy'siz).
# n<=1 veya hi<=lo gibi uç durumlarda tek elemanlı liste döndürür.
def _linspace(lo: float, hi: float, n: int) -> list:
    if n <= 1 or hi <= lo:
        return [max(lo, min(hi, lo))]
    step = (hi - lo) / (n - 1)
    return [lo + i * step for i in range(n)]


# ─── BÖLÜM: Ana robot sınıfı ───
# Algılama (LiDAR temizliği), planlama (FGM+DWA), kapı durum makinesi,
# kurtarma davranışları ve tüm telemetri bu tek ROS düğümünde yaşar.
class FgmDwaRobot(Node):

    def __init__(self):
        super().__init__("reactive_robot")
        self._is_shutdown = False

        # ── BÖLÜM: Parametreler (hepsi launch parametresiyle değiştirilebilir) ──
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
        self.lateral_weight       = self._param("lateral_weight",       0.05)
        self.max_linear_accel     = self._param("max_linear_accel",     2.0)
        self.max_angular_accel    = self._param("max_angular_accel",    8.0)
        self.robot_radius         = self._param("robot_radius",         0.12)
        self.run_duration         = self._param("run_duration",         60.0)
        self.log_dir              = self._param("log_dir", os.path.expanduser("~/rover_run_logs"))
        self.bearing_smoothing_alpha = self._param("bearing_smoothing_alpha", 0.20)

        # ── ALT BÖLÜM: Kapı-geçiş (door-commit) parametreleri ──
        # Kapı-geçiş: dar açıklıklar için özel geçiş modu. Tetik tamamen
        # sensörden gelir (haritadan bağımsız): LiDAR birkaç tarama üst
        # üste "eldeki en iyi boşluk dar" derse, rotanın neresinde olursa
        # olsun ALIGN devreye girer; sahne yeniden açılınca bırakır.
        # NEDEN: eski door_trigger_displacement odometri eşiği bu
        # haritanın kapı konumuna göre ezberlenmişti, genellenemiyordu.
        self.door_trigger_gap_width    = self._param("door_trigger_gap_width",    0.90)
        self.door_release_gap_width    = self._param("door_release_gap_width",    1.30)
        # "Kapıdan sızan ışık" testi: gerçek bir geçitte dar açıklığın
        # İÇİNDEN derin lazer dönüşleri görünür; duvar/engel köşesinde ise
        # derin hiçbir şey yoktur. Hem tetik hem commit bunu şart koşar;
        # aktif commit de derinlik kaybolursa iptal edilir. NEDEN: koşu
        # 20260710_174700'deki burun üstü 0.19m çakılmada konide tek bir
        # derin ışın bile yoktu.
        self.door_through_depth        = self._param("door_through_depth",        2.0)
        # Uyarlanır ışık eşiği: "ışık" = duvar yüzünden en az bu pay kadar
        # derin ışın (tavan: door_through_depth, taban: 1.2m).
        self.door_depth_margin         = self._param("door_depth_margin",         0.9)
        # Robottan dar bir ışık, geçit değil iki kenar arasındaki kıymık
        # bir aralıktır. NEDEN: deneme 1, 0.04-0.06m'lik ışıklara commit
        # atıp son-faz kaosuna savruldu. Ayrıca commit, ışığın tek şanslı
        # parıltı değil 3 ardışık taramada kararlı olmasını da ister.
        self.door_min_light_width      = self._param("door_min_light_width",      0.30)
        self.door_only_exit_frac       = self._param("door_only_exit_frac",       0.25)
        # ALIGN AKTİF bir durumdur: ışığın merkezine doğru yavaşça kavis
        # çiz, ışık tam karşına gelince COMMIT'e geç. NEDEN: deneme 2'de
        # ALIGN pasifti (kontrol blob-FGM/DWA'ya düşüyordu) ve robot duvar
        # önünde 24.5s çırpındı; üç commit kapısı asla aynı anda tutmadı.
        # Hız hep >0 kalır (ödev, robot süpürge gibi yerinde dönmeyi
        # yasaklıyor).
        self.door_align_speed          = self._param("door_align_speed",          0.12)
        self.door_align_gain           = self._param("door_align_gain",           1.2)
        self.door_align_max_w          = self._param("door_align_max_w",          0.9)
        self.door_align_timeout        = self._param("door_align_timeout",        8.0)
        self.door_retrigger_cooldown   = self._param("door_retrigger_cooldown",   2.0)
        self.door_cooldown_until       = 0.0
        self._last_light_sign          = 1.0
        self._light_ok_streak          = 0
        self._frame_ok_streak          = 0
        # ALIGN direksiyonu ve commit açı kapısı için EMA (üstel kayan
        # ortalama) ile yumuşatılmış ışık yönü. NEDEN: 3. kapıda ham ışık
        # taramadan taramaya +/-40 derece zıplıyor; ham ışık tam ortalansa
        # bile hizalanma kavisin dönüşü w, 0.6'lık commit kapısının altına
        # tam o anda inemiyordu (deneme 55: 12.6s ALIGN, kapılar hiç
        # çakışmadı).
        self.smoothed_align_light_deg  = 0.0
        self._commit_no_depth_streak   = 0
        self._commit_wide_streak       = 0

        # Sert U-dönüşü duvarı: bitiş yönü eksenden bu açıdan fazla sapan
        # DWA yörüngeleri doğrudan reddedilir (kullanıcı şartı: geriye
        # dönüp dolanmak kesinlikle yasak).
        self.uturn_hard_deg            = self._param("uturn_hard_deg",            100.0)
        self._uturn_hard_rad           = math.radians(self.uturn_hard_deg)
        self.door_trigger_streak       = int(self._param("door_trigger_streak",   3))
        self.door_release_streak       = int(self._param("door_release_streak",   5))
        self.door_commit_front_dist    = self._param("door_commit_front_dist",    1.0)
        self.door_exit_front_dist      = self._param("door_exit_front_dist",      0.8)
        self.door_commit_speed         = self._param("door_commit_speed",         0.45)
        self._door_narrow_streak       = 0
        self._door_open_streak         = 0
        self.door_center_gain          = self._param("door_center_gain",          0.8)
        self.door_center_max_w         = self._param("door_center_max_w",         0.9)
        self.door_max_lock_bearing_deg = self._param("door_max_lock_bearing_deg", 15.0)
        # Commit kararı, boşluk LiDAR'da TAM KARŞIDA görünürken verilir
        # (robot çerçevesindeki açı); odom'un dünya yönüne bakılMAZ.
        # NEDEN: eski yaw_err<45deg kapısı, 20260710_173711 koşusunda
        # robot kapı yüzünde boşluğa bakarken bile her commit'i engelledi;
        # enkoder yaw'ı (yalpalama + teker kayması) 50-120 derece sapık
        # okurken sensörün gördüğü boşluk açısı ~0'dı. Sensör kapısı
        # kaymadan etkilenmez ve haritadan bağımsızdır.
        self.door_commit_max_bearing_deg = self._param("door_commit_max_bearing_deg", 25.0)
        self.door_commit_max_wz        = self._param("door_commit_max_wz",        0.6)
        self.door_commit_timeout       = self._param("door_commit_timeout",       4.0)
        # Dünya çerçevesinde commit sınırlama (deneme 311): ± kilit robot
        # çerçevesindedir; gövde commit ortasında dönerse kör "en derin
        # ışına nişan al" mantığı ışınları rotadan istediği kadar uzağa
        # kovalayabilir (deneme 310: gövde eksenden 93° saptı, U-dönüş
        # bekçisi commit SIRASINDA patladı, 6.6 s'de DEPTH_LOST geldi).
        # Çözüm: hedef, est_heading+bearing toplamı max_world_off içinde
        # kalacak şekilde kırpılır; gövdenin kendisi abort_off'u geçtiyse
        # commit zaten batmıştır — hemen bırak ve kısa bir bekleme
        # (cooldown) koy; robot geri dönünce (gerçek) ışık körlemesine
        # sürtmek yerine yeniden yakalanabilsin.
        self.door_commit_max_world_off_deg = self._param("door_commit_max_world_off_deg", 65.0)
        self.door_commit_abort_off_deg = self._param("door_commit_abort_off_deg", 78.0)
        self.door_stall_nudge_delay    = self._param("door_stall_nudge_delay",    1.2)
        self.door_stall_front_dist     = self._param("door_stall_front_dist",     0.25)
        self.door_stall_nudge_w        = self._param("door_stall_nudge_w",        0.35)
        # IFGM fayda puanıyla boşluk seçimi (Hossain vd., J. Field
        # Robotics 2021, Denk. 13: U = m1*gap_size + m2*(pi - delta);
        # köken: FGM-I, Demir & Sezer 2017). Boşluk, fiziksel genişliği
        # ARTI koridor eksenine (robot çerçevesindeki hedef yönü) hizası
        # ile puanlanır; eski "dümdüz ileri" doğrusal önyargının
        # (gap_forward_bias) yerini aldı. Koridorun gerisine bakan geniş
        # bir boşluk artık hizadan neredeyse sıfır puan alır. NEDEN:
        # ölçülen hata tam buydu — son kapılarda geniş-ama-geriye açık
        # alana doğru U dönüşleri.
        self.gap_align_weight          = self._param("gap_align_weight",          0.45)
        # Planlayıcı kolu seçimi: use_dwa=false ise FGM-I DOĞRUDAN sürüş
        # çalışır — fayda puanlı boşluk açısı P-kontrolcüden geçer; hız
        # valisi ve ivme sınırı uygulanır, yörünge örnekleme ve DWA
        # maliyet fonksiyonu yoktur (kullanıcının istediği karşılaştırma
        # kolu). Kapı mekanizması, kurtarma ve telemetri aynen kalır.
        self.use_dwa                   = self._param("use_dwa",                   True)
        self.fgm_p_gain                = self._param("fgm_p_gain",                1.6)
        # Hibrit mod: açık arazide FGM-I doğrudan sürüş (kapılara varışı
        # 8-10s daha hızlı), manzara daraldığında (duvar yüzleri/odalar —
        # FGM-I'nin her şeyi kaybettiği yerler) DWA örnekleme. Geçiş,
        # deep_frac ölçütü + histerezis ile yapılır. Açıkken use_dwa'yı
        # ezer.
        self.hybrid_mode               = self._param("hybrid_mode",               False)
        self.hybrid_open_frac          = self._param("hybrid_open_frac",          0.30)
        self.hybrid_close_frac         = self._param("hybrid_close_frac",         0.20)
        self._hybrid_fgm_active        = False
        # SWEEP: "kapıyı görmüyorum — görene kadar duvar boyunca yürü"
        # davranışı (kullanıcının tarif ettiği insan mantığı). Sadece
        # dar/DWA rejiminde aktiftir; açık arazi davranışına dokunmaz.
        # Geçilebilir ışık yoksa duvar yüzüne paralel, belli mesafeden
        # tarayarak ilerle; koridor kenarlarında yön değiştir (zigzag);
        # önde geçilebilir bir ışık belirdiği an ALIGN'a devret.
        self.sweep_mode                = self._param("sweep_mode",                True)
        self.sweep_speed               = self._param("sweep_speed",               0.35)
        self.sweep_heading_deg         = self._param("sweep_heading_deg",         75.0)
        self.sweep_timeout             = self._param("sweep_timeout",             12.0)
        self.sweep_max_flips           = int(self._param("sweep_max_flips",       3))
        # 80/20 görev çevrimi: her sweep_period'un sweep_main_frac kadarını
        # ferah tarafta geçir, sonra kısaca öbür yöne dön (LiDAR bakışı) —
        # her kenar temasında bir o yana bir bu yana zıplamak yerine.
        self.sweep_period              = self._param("sweep_period",              5.0)
        self.sweep_main_frac           = self._param("sweep_main_frac",           0.8)
        # SWEEP yalnızca kapılar bölgesine aittir: en az bu kadar kapı
        # commit'i BAŞARIYLA geçilene dek kilitli kalır (haritadan
        # bağımsız ilerleme sinyali — açık arazide geçilecek kapı yok).
        self.sweep_min_doors           = int(self._param("sweep_min_doors",       1))
        self._sweep_dir                = 1.0
        self._sweep_flips              = 0
        self._sweep_start              = None
        self._sweep_last_flip          = 0.0
        self._face_dwell_start         = None
        self._last_sweep_dir           = 1.0
        self._last_sweep_exit          = None
        self._mouth_streak             = 0
        self._sweep_stall_streak       = 0
        self._thread_start             = None
        self._thread_block_until       = 0.0
        self._thread_bearing_ema       = None
        self._face_dwell_now           = 0.0
        self._no_door_streak           = 0
        # Lazerle koridor ortalama: yan boşluk dengesizliğinin metresi
        # başına hedef açıya eklenen derece (üst sınırlı).
        self.center_bias_gain          = self._param("center_bias_gain",          8.0)
        self.center_bias_max_deg       = self._param("center_bias_max_deg",       12.0)

        # Boşluk seçiminde histerezis: bir önce seçilen boşluk puan bonusu
        # taşır; rakip onu ancak gerçek bir farkla yenebilir. NEDEN: toplu
        # telemetri, karşı yakalardaki başabaş boşlukların (puan farkı
        # ~0.1) her taramada kazanan değiştirdiğini gösterdi — EMA'lı açı
        # bu gelgiti kovalayınca koridor zigzagı doğuyor. gap_keep_bonus,
        # koşu puanıyla aynı birimdedir (fiziksel boşluk genişliği,
        # metre); 0.0 histerezisi tamamen kapatır.
        self.gap_keep_bonus            = self._param("gap_keep_bonus",            0.35)
        self.gap_keep_match_deg        = self._param("gap_keep_match_deg",        20.0)
        self.fgm_prev_center_deg       = None   # ana aramanın koltuk sahibi boşluğu
        self.commit_prev_center_deg    = None   # commit konisinin koltuk sahibi

        # Dinamik DWA ağırlıkları (Han vd., Sci Rep 2025 "hybrid RRT-DWA"
        # çalışmasının iyileştirilmiş-DWA yarısından; T-RO 2024'ün "sabit
        # ağırlıklar zayıf halka" bulgusu ve Ogren & Leonard 2005'in dar
        # koridorda hiçbir sabit dengenin çalışmadığı ispatıyla destekli):
        # k = 1 - exp(-front_min / weight_decay_dist) ağırlıkları ölçekler.
        # Açık alanda (k~1) bugünkü davranış aynen kalır; engel yakınında
        # (k~0) hız ödülü söner, yön/süreklilik ikiye katlanır — böylece
        # örnekleyici kapı içinde hız uğruna boşluktan feragat etmez.
        # 0.0 bu programlamayı kapatır (sabit ağırlıklar).
        self.weight_decay_dist         = self._param("weight_decay_dist",         0.8)

        # Hız valisi (governor): azami hızı ön boşluğa göre doğrusal
        # kısar. Telemetri: ortalama hız 0.37 (tavan 0.65) ve 60s'nin
        # 33s'i AVOID modunda — tam hız eskiden 2.0m ön boşluk istiyordu,
        # dağınık koridor bunu neredeyse hiç sunmaz. Engel-yakını rejim
        # artık dinamik DWA ağırlıklarının işi; vali sadece dıştaki
        # güvenlik zarfı olarak kalır. Toplu test: full_dist'i 1.4'e
        # gevşetmek duvar sürtme kaymasını neredeyse ikiye katladı ve
        # feci savrulmalar üretti (batch 20260710_200834); 2.0 yük
        # taşıyan değerdir.
        self.governor_full_dist        = self._param("governor_full_dist",        2.0)
        self.governor_floor_dist       = self._param("governor_floor_dist",       0.5)

        # Şartlı sprint: max_vel'i her yerde 0.90'a çekmek (batch
        # 20260710_202630) de valiyi gevşetmek (batch 20260710_200834) de
        # arazi geçişini duvara vurdu — hız seçici olmak zorunda. Sprint,
        # tavanı SADECE hizalı ve önü açıkken max_vel üstüne kaldırır:
        # ön açık, hedef açı hemen hemen tam karşıda, komutlanan dönüş
        # hızı düşük. sprint_vel <= max_vel özelliği kapatır.
        self.sprint_vel                = self._param("sprint_vel",                0.90)
        self.sprint_front_dist         = self._param("sprint_front_dist",         1.8)
        self.sprint_bearing_deg        = self._param("sprint_bearing_deg",        12.0)
        self.sprint_max_w              = self._param("sprint_max_w",              0.4)
        self.eff_heading_weight        = self.heading_weight
        self.eff_speed_weight          = self.speed_weight
        self.eff_continuity_weight     = self.continuity_weight

        # ── ALT BÖLÜM: Sıkışma kurtarması (wedge recovery) ──
        # Ön mesafe birkaç tik boyunca dipte kalırsa kısa süre geri git.
        self.wedge_front_dist     = self._param("wedge_front_dist",     0.18)
        self.wedge_consecutive    = int(self._param("wedge_consecutive", 5))
        self.recover_seconds      = self._param("recover_seconds",      0.7)
        self.recover_linear       = self._param("recover_linear",       -0.13)
        self.wedge_count          = 0
        self.recover_until        = None
        # Kademeli kurtarma: 9cm düz geri gitmek bir cepten çıkaramaz
        # (deneme 30: ilk rover'da tüm koşu boyunca sıkışma, 10.8m teker
        # kayması, 39s TIGHT). Kısa aralıkla tekrarlanan kurtarmalar
        # gitgide uzar ve dönüş bileşenini bir sağa bir sola ekler.
        self._recover_recent      = []   # son tetiklerin zaman damgaları
        self._recover_turn_sign   = 1.0
        self._recover_w           = 0.0
        # Sadece-lazerle gerçek sıkışma dedektörü: hareket komutu varken
        # ardışık taramalar hiç değişmiyorsa (ortalama |dRange| ~ 0) robot
        # fiziken çakılıdır — komutu hangi mod veriyor olursa olsun.
        # Odometrinin göremediği (tekerler kayar) ve gt'nin kontrolde
        # kullanılamadığı 8-10s'lik commit/TIGHT rehine anlarını yakalar.
        self._prev_scan_full      = None
        self._static_streak       = 0
        # SIDE_PIN bekçisi durumu: ileri komutun kesintisiz başlangıcı
        # ve 2 s'lik dönme-telafili karşılaştırma için referans tarama.
        self._fwd_cmd_since       = None
        self._pin_ref_scan        = None
        self._pin_ref_t           = 0.0

        # Hedef yaw = 0 (koridor yönü; robot odom çerçevesine hizalı doğar)
        self.desired_yaw          = 0.0

        # ── BÖLÜM: Koşu durumu (çalışma anındaki değişkenler) ─────────────────
        self.start_time           = None
        self.scan_received        = False
        self.last_scan_time       = None
        self.cmd_linear_x         = 0.0
        self.cmd_angular_z        = 0.0
        self.last_label           = "INIT"

        # Düğümün KENDİ hız komutlarından iç ölü hesap (dead reckoning) —
        # ödevin katı "yalnız lazerle algıla" yorumu altında kontrol için
        # yön/konum ön bilgisi. Hiçbir kontrol terimi /odom veya /imu'dan
        # beslenmez. NEDEN: enkoder yaw'ı teker kaymasıyla çürüyor (duvar
        # sıyırmalarından sonra 50-120 derece sapma görüldü); "ileri"
        # yana, hatta geriye bakar oldu ve robot koridorun gerisine
        # sürüldü. Komut entegrasyonu da temasta kayar ama en azından
        # sensör kuralına uygunluk sorusu taşımaz.
        self.dr_heading           = 0.0
        self.dr_x                 = 0.0
        self.dr_y                 = 0.0
        # Tarama eşleştirmeli yön (est_heading): dönüş, ardışık lazer
        # taramalarından (dairesel korelasyon) doğrudan ölçülür; %100
        # /scan kaynaklıdır. Komut entegrasyonu (dr_heading) temasta yalan
        # söyler — takılı bir commit, tekerlerin hiç yapmadığı dönüşü
        # entegre eder; 20260710_181026 koşusundaki timeout sonrası
        # -136 derecelik savrulma tam buydu. Tarama, dünyanın gerçekte
        # ne yaptığını ölçer.
        self.est_heading          = 0.0
        self._heading_reject_streak = 0
        self._commit_entry_front  = None
        # Commit'in gerçekten bir kapı düzleminden geçtiğinin lazer
        # kanıtı: gerçek bir kapının direkleri (0.5-0.8 m genişlik) geçiş
        # sırasında gövdenin yan sektörlerinden < ~0.55 m mesafeyle
        # süpürülür. Hiç sıyırmayan commit hiç geçmemiştir (deneme 311:
        # duvar-1 commit'i, çerçeve 1.11m'deyken hâlâ gerideki tarafta
        # dururken CLEARED dedi; FGM koşunun kalanında sağ girintide
        # gezindi).
        self._commit_side_graze   = True
        self._commit_scan_count   = 0
        self.smoothed_bearing_deg = 0.0
        self.smoothed_commit_bearing_deg = 0.0

        # Kapı durum makinesi: None | "ALIGN" (hizalan) | "COMMIT" (dal)
        self.door_mode               = None
        self.door_commit_start_time  = None  # COMMIT'e giriş anı (rospy); timeout emniyet ağı

        # Tanılama değişkenleri (kontrolde kullanılmaz, sadece gözlem)
        self.have_odom            = False
        self.x = self.y = self.yaw = 0.0
        self.odom_v = self.odom_w = 0.0
        self.start_xy             = None
        self.prev_xy              = None
        self.path_length          = 0.0
        self.min_front_min_seen   = float("inf")
        self.min_front_ctx        = None   # en yakın yaklaşmanın (elapsed, x, y) kaydı
        self.wedge_trigger_count  = 0
        self.last_xy_log_t        = None
        self.last_sample_log_t    = None

        # Telemetri muhasebesi (yalnız gözlem; kontrolü asla beslemez)
        self.scan_count           = 0
        self.scan_gap_count       = 0
        self.last_cb_time         = None   # önceki callback'in sim saati (kopukluk tespiti)
        self.cb_count             = 0
        self.cb_ms_sum            = 0.0
        self.cb_ms_max            = 0.0
        self.state_time           = {}     # her modda geçirilen saniyeler
        self._last_tele_t         = None
        self.door_align_start_time = None
        self.door_commit_count    = 0
        self.door_cleared_count   = 0
        self.door_timeout_count   = 0
        self.door_commit_total_time = 0.0
        self.door_stall_active    = False
        self.last_align_gate_log_t = None
        self.last_contact_yaw_log_t = None
        self.last_mag_log_t        = None
        self.use_arbiter           = self._param("use_arbiter", True)
        self.use_grid_door         = self._param("use_grid_door", True)
        # Grid commit'leri yeniden açıldı (deneme 259): yollar artık
        # boşluk-ağırlıklı (medial eksen) planlanıyor; 236/237'deki
        # kapı çerçevesini sıyıran köşe kesmeleri böyle düzeldi.
        self.use_grid_commit       = self._param("use_grid_commit", True)
        self._arb_last_winner      = "FGM"
        self._arb_last_bearing     = 0.0
        self.last_arb_log_t        = None
        self.corner_avoid_count    = 0
        self.last_corner_log_t     = None
        self._backmo_streak        = 0
        self.backward_motion_count = 0
        self._gt_y_hist            = []
        self.upcourse_motion_count = 0
        self.last_upcourse_log_t   = None
        self._light_mem_abs        = 0.0
        self._light_mem_t          = None
        self.grid_commit_scans     = 0
        self._guard_last_w_sign    = 0
        self._guard_last_flip_t    = 0.0
        self.guard_uturn_count     = 0
        self.guard_back_count      = 0
        self.guard_pinball_count   = 0
        self.last_guard_log_t      = None
        self._last_side_open       = 0.0
        self._pin_last_w_sign      = 0
        self._pin_flip_times       = []
        self._pin_track            = []
        self.pinball_event_count   = 0
        self.last_pinball_log_t    = None

        # Bildirilen iki hata modunu avlamak için telemetri: koridor
        # zigzagı ve son kapılarda çarpma/geri dönüp dolanma.
        # Ground truth simülatörün p3d eklentisinden gelir
        # (ground_truth:=true); kontrolde ASLA kullanılmaz — kontrol
        # kaynağı enkoder /odom kalır — ama enkoder, tekerler duvara
        # karşı boşa dönerken bile saymaya devam eder; çarpışmayı ve
        # gerçek yer değiştirmeyi görmek için gt şarttır.
        self.have_gt              = False
        self.gt_x = self.gt_y = self.gt_yaw = 0.0
        self.gt_speed             = 0.0
        self.gt_start_xy          = None
        self.gt_start_yaw         = None   # dünya çerçevesinde koridor yönü
        self.gt_prev_xy           = None
        self.gt_path_length       = 0.0
        self.min_r360_seen        = float("inf")
        self.gap_switch_count     = 0
        self.fgm_fallback_count   = 0
        self.commit_fallback_count = 0
        self.last_fallback_log_t  = None
        self.stuck_since          = None
        self.stuck_reported       = False
        self.stuck_event_count    = 0
        self.wrongway_since       = None
        self.wrongway_reported    = False
        self.wrongway_event_count = 0
        self.max_net_disp         = 0.0
        self.regress_active       = False
        self.regress_event_count  = 0
        self.w_flip_count         = 0
        self.prev_w_sign          = 0
        self._scanf               = None
        self.last_scan_dump_t     = None

        # ── BÖLÜM: Log dosyalarını aç ─────────────────────────────────────────
        self._open_log()

        # ── BÖLÜM: ROS yayıncıları ve aboneleri ───────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data)
        # Sadece sim ground_truth:=true ile koşarken yayında; yoksa zararsız.
        self.gt_sub   = self.create_subscription(
            Odometry, "/ground_truth_odom", self.ground_truth_callback,
            qos_profile_sensor_data)

        self.get_logger().info("Node started.")
        self.get_logger().info("[FgmDwaRobot] ready — waiting for /scan …")

    # ─── BÖLÜM: ROS 2 yardımcıları ────────────────────────────────────────────

    def _param(self, name: str, default):
        """ROS 1 rospy.get_param("~name", default) kısayolunun ROS 2 karşılığı."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        """Geçerli zaman (saniye, float) — rospy.Time.now().to_sec() karşılığı."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ─── BÖLÜM: Loglama yardımcıları ──────────────────────────────────────────

    def _open_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp    = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._stamp = stamp
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
            f"door_trigger_gap_w={self.door_trigger_gap_width} door_release_gap_w={self.door_release_gap_width} "
            f"door_commit_front={self.door_commit_front_dist} "
            f"door_exit_front={self.door_exit_front_dist} door_commit_speed={self.door_commit_speed} "
            f"door_center_gain={self.door_center_gain} door_center_max_w={self.door_center_max_w} "
            f"gap_align_weight={self.gap_align_weight} gap_keep_bonus={self.gap_keep_bonus} "
            f"gap_keep_match_deg={self.gap_keep_match_deg} weight_decay_dist={self.weight_decay_dist} "
            f"governor_full_dist={self.governor_full_dist} governor_floor_dist={self.governor_floor_dist}"
        )
        self.get_logger().info(f"[FgmDwaRobot] Log → {log_path}")

        # Koşu sonrası grafik çizmek için kontrol döngüsü başına bir CSV
        # satırı. Hangi kontrol yolu işlerse işlesin her scan callback'i
        # bir satır yazar; DWA'ya özgü kolonlar RECOVER/DOOR satırlarında
        # boş kalır.
        tele_path = os.path.join(self.log_dir, f"telemetry_{stamp}.csv")
        self._telef = open(tele_path, "w", buffering=1)
        self._telef.write(
            "t,x,y,yaw_deg,net_disp,odom_v,odom_w,mode,door_mode,front_min,"
            "bearing_raw_deg,bearing_smooth_deg,gap_depth,v_cmd,w_cmd,current_max_vel,"
            "cost_total,cost_obstacle,cost_heading,cost_speed,cost_continuity,"
            "cost_corridor,cost_lateral,n_candidates,n_colliding,"
            "v_at_window_hi,w_at_window_edge,wedge_count,cb_ms,"
            "min_r360,gt_x,gt_y,gt_net_disp,gt_speed,"
            "n_gaps,gap_center_deg,gap_width,fgm_fallback,target_world_deg,"
            "est_head_deg,dr_head_deg,gt_yaw_deg\n"
        )
        self.get_logger().info(f"[FgmDwaRobot] Telemetry → {tele_path}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logf.write(f"[{ts}] {msg}\n")

    def _emit_telemetry(self, now, elapsed, mode, front_min,
                        bearing_raw=None, bearing_smooth=None, gap_depth=None,
                        current_max_vel=None, dwa=None, cb_t0=None,
                        min_r360=None, gap_info=None):
        """Bir CSV telemetri satırı yazar; mod başına süre sayacını da günceller."""
        cb_ms = None
        if cb_t0 is not None:
            cb_ms = (time.perf_counter() - cb_t0) * 1000.0
            self.cb_count += 1
            self.cb_ms_sum += cb_ms
            if cb_ms > self.cb_ms_max:
                self.cb_ms_max = cb_ms

        if self._last_tele_t is not None:
            self.state_time[mode] = self.state_time.get(mode, 0.0) + max(0.0, now - self._last_tele_t)
        self._last_tele_t = now

        net_disp = (math.hypot(self.x - self.start_xy[0], self.y - self.start_xy[1])
                    if self.have_odom else 0.0)
        gt_net = (math.hypot(self.gt_x - self.gt_start_xy[0], self.gt_y - self.gt_start_xy[1])
                  if self.have_gt else None)

        # Gerileme nöbetçisi: net yer değiştirmenin, o ana dek görülen
        # zirvesinin belirgin altına inmesi "geri dönüp dolanma / yanlış
        # yön" imzasıdır.
        disp_for_regress = gt_net if gt_net is not None else net_disp
        if disp_for_regress > self.max_net_disp:
            self.max_net_disp = disp_for_regress
            self.regress_active = False
        elif disp_for_regress < self.max_net_disp - 0.5 and not self.regress_active:
            self.regress_active = True
            self.regress_event_count += 1
            self.get_logger().info(f"[FgmDwaRobot] REGRESS — displacement fell "
                          f"{self.max_net_disp - disp_for_regress:.2f}m below peak "
                          f"{self.max_net_disp:.2f}m (mode={mode}).")
            self._log(f"EVENT REGRESS #{self.regress_event_count} | disp={disp_for_regress:.2f} "
                      f"peak={self.max_net_disp:.2f} mode={mode} yaw={math.degrees(self.yaw):.1f}")

        # Geri hareket nöbetçisi (kullanıcı talimatı telemetrisi): GERÇEK
        # her geri teker hareketini (enkoder v < -0.02, 3 tarama üst üste)
        # ve sürekli geri-rota gt kaymasını (~1 s içinde gt_y 0.06 m'den
        # fazla artış) işaretle. Sadece ölçüm, müdahale yok.
        if self.odom_v < -0.02:
            self._backmo_streak += 1
        else:
            self._backmo_streak = 0
        if self._backmo_streak == 3:
            self.backward_motion_count += 1
            self._log(f"EVENT BACKWARD_MOTION #{self.backward_motion_count} | "
                      f"odom_v={self.odom_v:.3f} mode={mode}")
        if self.have_gt:
            self._gt_y_hist.append((now, self.gt_y))
            self._gt_y_hist = [(t0, y0) for t0, y0 in self._gt_y_hist
                               if now - t0 <= 1.0]
            if (len(self._gt_y_hist) >= 3
                    and self.gt_y - self._gt_y_hist[0][1] > 0.06):
                if (self.last_upcourse_log_t is None
                        or now - self.last_upcourse_log_t >= 2.0):
                    self.last_upcourse_log_t = now
                    self.upcourse_motion_count += 1
                    self._log(f"EVENT UPCOURSE_MOTION #{self.upcourse_motion_count} | "
                              f"dy=+{self.gt_y - self._gt_y_hist[0][1]:.2f} mode={mode}")

        # Pinball nöbetçisi (kullanıcının istediği telemetri): "duvar
        # yüzünde sekme" = 4 s'lik pencerede çok sayıda direksiyon yön
        # değişimi + bolca komutlanan yol ama sıfıra yakın net ilerleme.
        # Sadece tespit — kontrole dokunmaz.
        s_now = (1 if self.cmd_angular_z > 0.15
                 else (-1 if self.cmd_angular_z < -0.15 else 0))
        if s_now != 0:
            if self._pin_last_w_sign != 0 and s_now != self._pin_last_w_sign:
                self._pin_flip_times.append(now)
            self._pin_last_w_sign = s_now
        self._pin_flip_times = [t for t in self._pin_flip_times
                                if now - t < 4.0]
        self._pin_track.append((now, self.dr_x, self.dr_y))
        while self._pin_track and now - self._pin_track[0][0] > 4.0:
            self._pin_track.pop(0)
        if len(self._pin_track) >= 2 and mode != "RECOVER":
            p0 = self._pin_track[0]
            pin_net = math.hypot(self.dr_x - p0[1], self.dr_y - p0[2])
            pin_path = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                           for a, b in zip(self._pin_track, self._pin_track[1:]))
            if (len(self._pin_flip_times) >= 6
                    and pin_path > 0.8 and pin_net < 0.3):
                if (self.last_pinball_log_t is None
                        or now - self.last_pinball_log_t >= 2.0):
                    if (self.last_pinball_log_t is None
                            or now - self.last_pinball_log_t >= 3.0):
                        self.pinball_event_count += 1  # yeni sekme bölümü başladı
                    self.last_pinball_log_t = now
                    self._log(f"EVENT PINBALL #{self.pinball_event_count} | "
                              f"flips={len(self._pin_flip_times)} "
                              f"dr_net={pin_net:.2f} dr_path={pin_path:.2f} "
                              f"mode={mode} front_min={front_min:.2f}")

        # Planlayıcının dünya çerçevesinde yöneldiği yön (yaw + bearing):
        # "koridorun ilerisine nişan" ile "geldiğimiz yöne nişan"ı ayırır.
        target_world_deg = None
        if bearing_smooth is not None:
            target_world_deg = math.degrees(
                _angle_diff(self.yaw + math.radians(bearing_smooth), 0.0))

        def f3(v):
            return "" if v is None else f"{v:.3f}"

        d = dwa or {}
        g = gap_info or {}
        self._telef.write(
            f"{elapsed:.3f},{self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.2f},"
            f"{net_disp:.3f},{self.odom_v:.3f},{self.odom_w:.3f},{mode},"
            f"{self.door_mode or ''},{front_min:.3f},"
            f"{f3(bearing_raw)},{f3(bearing_smooth)},{f3(gap_depth)},"
            f"{self.cmd_linear_x:.3f},{self.cmd_angular_z:.3f},{f3(current_max_vel)},"
            f"{f3(d.get('total'))},{f3(d.get('obstacle'))},{f3(d.get('heading'))},"
            f"{f3(d.get('speed'))},{f3(d.get('continuity'))},{f3(d.get('corridor'))},"
            f"{f3(d.get('lateral'))},{d.get('n_candidates', '')},{d.get('n_colliding', '')},"
            f"{d.get('v_at_hi', '')},{d.get('w_at_edge', '')},{self.wedge_count},{f3(cb_ms)},"
            f"{f3(min_r360)},{f3(self.gt_x if self.have_gt else None)},"
            f"{f3(self.gt_y if self.have_gt else None)},{f3(gt_net)},"
            f"{f3(self.gt_speed if self.have_gt else None)},"
            f"{g.get('n_gaps', '')},{f3(g.get('chosen_center_deg'))},"
            f"{f3(g.get('chosen_width_m'))},"
            f"{'' if not g else int(g.get('fallback', False))},{f3(target_world_deg)},"
            f"{math.degrees(self.est_heading):.2f},{math.degrees(self.dr_heading):.2f},"
            f"{f3(math.degrees(self.gt_yaw) if self.have_gt else None)}\n"
        )

    def _dump_scan(self, elapsed, event, extended):
        """Anomali anında planlayıcının girdisini (disparity sonrası
        mesafeleri) dosyaya kaydeder; karar sonradan masa başında yeniden
        oynatılabilsin. Saniyede en fazla 1 kayıt yapılır."""
        now_wall = time.perf_counter()
        if self.last_scan_dump_t is not None and now_wall - self.last_scan_dump_t < 1.0:
            return
        self.last_scan_dump_t = now_wall
        if self._scanf is None:
            path = os.path.join(self.log_dir, f"scan_dumps_{self._stamp}.csv")
            self._scanf = open(path, "w", buffering=1)
            self.get_logger().info(f"[FgmDwaRobot] Anomaly scan dumps → {path}")
        vals = ",".join(f"{r:.3f}" for r in extended)
        self._scanf.write(f"{elapsed:.3f},{event},{math.degrees(self.yaw):.1f},{vals}\n")

    def _update_anomaly_trackers(self, now, elapsed, mode, front_min, extended):
        """Bildirilen son-kapı hataları için salt-gözlem dedektörleri.

        STUCK: ileri komut var ama ground-truth hızı ~sıfır — tekerler
        dönerken robot duvara çakılı demektir. Enkoder odometrisi bunu
        göremez (odometrySource=encoder kayma sırasında saymayı sürdürür);
        bu dedektör ancak /ground_truth_odom varsa kurulur.
        WRONG WAY: yön, koridor ekseninden >120 derece sapmış halde
        kalıyor — "geri dönüp dolanma" imzası, olay anında yakalanır."""
        pinned = (self.have_gt and self.cmd_linear_x > 0.15 and self.gt_speed < 0.04)
        if pinned:
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > 0.6 and not self.stuck_reported:
                self.stuck_reported = True
                self.stuck_event_count += 1
                self.get_logger().info(f"[FgmDwaRobot] STUCK — cmd v={self.cmd_linear_x:.2f} but ground-truth "
                              f"speed {self.gt_speed:.3f}m/s (mode={mode}, front_min={front_min:.2f}).")
                self._log(f"EVENT STUCK #{self.stuck_event_count} | mode={mode} "
                          f"v_cmd={self.cmd_linear_x:.2f} gt_speed={self.gt_speed:.3f} "
                          f"front_min={front_min:.2f} gt_xy=({self.gt_x:.2f},{self.gt_y:.2f}) "
                          f"yaw={math.degrees(self.yaw):.1f}")
                self._dump_scan(elapsed, "STUCK", extended)
        else:
            self.stuck_since = None
            self.stuck_reported = False

        # desired_yaw odom çerçevesinde sabittir; ground-truth yaw ise
        # dünya çerçevesindedir — o yüzden gt, doğuş anındaki dünya
        # yönüne göre ölçülür.
        if self.have_gt:
            off_corridor = _angle_diff(self.gt_yaw, self.gt_start_yaw)
        else:
            off_corridor = _angle_diff(self.yaw, self.desired_yaw)
        wrong = abs(off_corridor) > math.radians(120.0)
        if wrong:
            if self.wrongway_since is None:
                self.wrongway_since = now
            elif now - self.wrongway_since > 1.0 and not self.wrongway_reported:
                self.wrongway_reported = True
                self.wrongway_event_count += 1
                self.get_logger().info(f"[FgmDwaRobot] WRONG WAY — heading {math.degrees(off_corridor):+.0f}° "
                              f"off-corridor sustained 1s (mode={mode}).")
                self._log(f"EVENT WRONG_WAY #{self.wrongway_event_count} | mode={mode} "
                          f"off_corridor={math.degrees(off_corridor):+.1f} "
                          f"bearing_smooth={self.smoothed_bearing_deg:.1f} "
                          f"xy=({self.x:.2f},{self.y:.2f})")
                self._dump_scan(elapsed, "WRONG_WAY", extended)
        else:
            self.wrongway_since = None
            self.wrongway_reported = False

    # ─── BÖLÜM: Odometri callback'leri (yalnız tanılama, kontrol değil) ──────

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw  = math.atan2(siny_cosp, cosy_cosp)
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z

        if not self.have_odom:
            self.have_odom = True
            self.start_xy  = (self.x, self.y)
            self.prev_xy   = (self.x, self.y)
            return

        self.path_length += math.hypot(self.x - self.prev_xy[0], self.y - self.prev_xy[1])
        self.prev_xy = (self.x, self.y)

    def ground_truth_callback(self, msg: Odometry):
        """Gazebo p3d'den dünya-çerçevesi poz (yalnız telemetri, asla kontrol)."""
        q = msg.pose.pose.orientation
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.gt_yaw = math.atan2(siny_cosp, cosy_cosp)
        lin = msg.twist.twist.linear
        self.gt_speed = math.hypot(lin.x, lin.y)

        if not self.have_gt:
            self.have_gt = True
            self.gt_start_xy = (self.gt_x, self.gt_y)
            self.gt_start_yaw = self.gt_yaw
            self.gt_prev_xy = (self.gt_x, self.gt_y)
            self.get_logger().info("[FgmDwaRobot] Ground-truth odometry available — slip/collision detection armed.")
            return

        self.gt_path_length += math.hypot(self.gt_x - self.gt_prev_xy[0],
                                          self.gt_y - self.gt_prev_xy[1])
        self.gt_prev_xy = (self.gt_x, self.gt_y)

    # ─── BÖLÜM: Tarama ön işleme (LiDAR temizliği ve türetilen sinyaller) ────

    def _corridor_heading(self, clean: list, n: int, prev_heading: float,
                          max_jump_rad: float = math.radians(40.0)):
        """Robotun koridor eksenine göre MUTLAK yönü; her döngüde tarama
        geometrisinden okunur — entegrasyon yok, dolayısıyla kayma yok.

        Fikir: koridorun duvarları görüşteki en uzun yapılardır; yerel
        parça yönlerinin uzunluk-ağırlıklı histogramı (mod 180°) robot
        çerçevesindeki duvar yönü θ_w'de tepe yapar. Robotun koridor
        eksenine göre yönü de -θ_w olur; 180°'lik belirsizlik önceki
        tahminle süreklilikten çözülür. Sahnede oy verecek kadar yapı
        yoksa None döner (çağıran, önceki tahmini elinde tutar).

        Tarihçe: tarama-taramaya dönüş korelasyonunun yerini aldı; o
        yöntem ötelemeden ~0.5°/tarama sapma kapıyor ve 60 s'lik koşuda
        +294°'lik hayalet dönüş biriktiriyordu (kontrol koşusu
        20260710_181817)."""
        res = 2.0 * math.pi / n
        bins = [0.0] * 36  # 180°'yi 5'er derecelik 36 kutuya böl
        prev_pt = None
        for i in range(n):
            r = clean[i]
            if r < 0.15 or r > 4.0:
                prev_pt = None
                continue
            a = i * res
            pt = (r * math.cos(a), r * math.sin(a))
            if prev_pt is not None:
                dx, dy = pt[0] - prev_pt[0], pt[1] - prev_pt[1]
                seg = math.hypot(dx, dy)
                if seg < 0.25:  # boşluk/nesne arası sıçramaları ele
                    ang = math.atan2(dy, dx) % math.pi
                    bins[int(ang / math.pi * 36) % 36] += seg
            prev_pt = pt
        total = sum(bins)
        if total < 1.0:
            return None
        b = max(range(36), key=lambda k: bins[k])
        if bins[b] < 0.25 * total:
            return None  # baskın yön yok (açık dağınıklık) — eskiyi koru
        # dairesel histogramda parabolik tepe inceltme (kutu-altı hassasiyet)
        em, e0, ep = bins[(b - 1) % 36], bins[b], bins[(b + 1) % 36]
        off = 0.5 * (ep - em) / max(1e-9, (2.0 * e0 - em - ep)) if (2.0 * e0 - em - ep) > 1e-9 else 0.0
        theta_w = ((b + 0.5 + off) * (math.pi / 36.0)) % math.pi
        # mod-180 duvar yönünden yön adayları türet
        cand = -theta_w
        best = None
        for c in (cand, cand + math.pi, cand - math.pi):
            if best is None or abs(_angle_diff(c, prev_heading)) < abs(_angle_diff(best, prev_heading)):
                best = c
        # inandırıcı olmayan sıçramaları reddet (kapı yüzlerinde dik kısa
        # duvarlar anlık baskın çıkabilir); ilk temiz tarama yeniden kilitler
        if abs(_angle_diff(best, prev_heading)) > max_jump_rad:
            return None
        return best, bins[b] / total

    @staticmethod
    def _scan_rotation(prev: list, cur: list, n: int, max_shift_deg: int = 25):
        """Ardışık iki tarama arasındaki GERÇEK yaw değişimi; kaba kuvvet
        dairesel çapraz korelasyonla ölçülür (1° adım, 4m'de kırpılmış
        mesafeler üzerinde SAD maliyeti). (dyaw_rad, kalite, best_cost)
        veya None.

        Amaç: KOMUTSUZ dönüşü yakalamak. Düz geri giderken/sürterken
        duvar temasının torku, cmd_w sıfırken robotu fiziksel olarak
        döndürür — komut-tahmini pusula bunu hiç görmez; koridor
        düzeltmesi de (mod-180) dönüş 90°'yi geçince ters yönü kilitler
        (deneme 126/128: sürtme sonrası commit'ler geri yönde "cleared"
        dedi). İşaret kuralı: robot +a (saat tersi) dönerse desenler
        daha küçük robot açılarına kayar; yani cur[i] ≈ prev[i+s],
        s ≈ +a derece."""
        step = max(1, n // 180)  # ~180 örnek yeterli, hepsi değil
        idxs = [i for i in range(0, n, step)
                if prev[i] < 9.5 and cur[i] < 9.5]
        if len(idxs) < 60:
            return None
        shift_unit = int(round(n / 360.0))  # derece başına ışın sayısı
        best_s, best_cost, costs = None, None, []
        for s_deg in range(-max_shift_deg, max_shift_deg + 1):
            s = s_deg * shift_unit
            c = 0.0
            for i in idxs:
                c += abs(min(cur[i], 4.0) - min(prev[(i + s) % n], 4.0))
            c /= len(idxs)
            costs.append(c)
            if best_cost is None or c < best_cost:
                best_cost, best_s = c, s_deg
        if abs(best_s) >= max_shift_deg:  # tepe pencere kenarında: güvenilmez
            return None
        mean_cost = sum(costs) / len(costs)
        quality = (mean_cost - best_cost) / max(1e-6, mean_cost)
        # best_cost da dönüyor (SIDE_PIN bekçisi için): en iyi kaydırma
        # SONRASI kalan ortalama fark ~ metre cinsinden "dönme dışı
        # değişim". Küçükse sahne dönme hariç hiç değişmemiş demektir.
        return math.radians(best_s), quality, best_cost

    def _door_light(self, extended: list, n: int, half_deg: float, front_ref: float,
                    depth_thr: float = None, behind_margin: float = None):
        """Burnun ±half_deg konisinde, depth_thr'den (varsayılan:
        door_through_depth) derin ışınların EN GENİŞ bitişik yayı —
        yani 'kapıdan sızan ışık'. (center_deg, width_m, slit_d) veya
        (None, 0.0, None) döndürür; slit_d = kapı çerçevesi mesafesi
        (derin koşunun kenarındaki en yakın ışın — yarık düzlemi).

        behind_margin verilirse, koşunun EN DERİN ışını slit_d +
        behind_margin'e ulaşmak zorundadır: derinlik ROBOTTAN değil
        YARIKTAN ölçülür. NEDEN: robota göre eşik, sığ bir girintiyi
        uzaktan geçit sanıyordu — deneme 313: duvara 1.8 m mesafede
        frame_min 0.39 (arazi dağınıklığı) eşiği 1.69'a kurdu; sağ
        girintinin 2.1 m'deki arka duvarı, yarığının ardında sadece
        ~0.3 m varken bu eşiği aştı; yanlış kapış koşuyu direğe sürdü.

        Bu, kapı algısının temel taşıdır. FGM'in koşu genişliği burada
        iş görmez: onun geçilebilirlik testi (range > min_gap_width) bir
        DERİNLİK testidir; 0.7m ötedeki duvar yüzü bile 'geçilebilir'
        okunur ve tüm koşular, tam da kapı mantığının dar yarığı görmesi
        gereken anda tek geniş bloba birleşir (telemetri 20260710_182636:
        merkez 0'da 3-4m'lik tek koşu, front_min 0.3'e düşerken)."""
        thr = depth_thr if depth_thr is not None else self.door_through_depth
        half = int(half_deg * n / 360)
        offs = list(range(-half, half + 1))
        best = None  # (ışın_sayısı, başlangıç_k, bitiş_k, slit_d)
        run_start = None
        for k in range(len(offs) + 1):
            deep = (k < len(offs)
                    and extended[offs[k] % n] > thr)
            if deep and run_start is None:
                run_start = k
            elif not deep and run_start is not None:
                s, e = run_start, k - 1
                run_start = None
                length = e - s + 1
                edge_rs = []
                if s - 1 >= 0:
                    edge_rs.append(extended[offs[s - 1] % n])
                if e + 1 < len(offs):
                    edge_rs.append(extended[offs[e + 1] % n])
                slit_d = min(edge_rs) if edge_rs else front_ref
                if behind_margin is not None:
                    deepest = max(extended[offs[j] % n]
                                  for j in range(s, e + 1))
                    if deepest < slit_d + behind_margin:
                        continue  # sığ girinti — geçit değil
                if best is None or length > best[0]:
                    best = (length, s, e, slit_d)
        if best is None:
            return None, 0.0, None
        length, s, e, slit_d = best
        center_deg = (offs[s] + offs[e]) / 2.0 * (360.0 / n)
        # Genişlik YARIK düzleminde hesaplanır: yay, front_ref ile değil
        # kapı çerçevesi mesafesiyle (derin koşuya komşu ışınlar)
        # çarpılır. NEDEN: front_min hizalıyken kapının içindeki
        # derinliği okur (genişlik mesafeyle şişer), burun yakınken de
        # çerçeve kenarını okur (deneme 159: tam karşıda 3.16m'lik
        # kanal için genişlik 0.10 çıktı ve kapış hiç ateşlenmedi).
        # Komşu-ışın ölçeği genişliği kabaca mesafeden bağımsız,
        # fiziksel yarık genişliğine eşit yapar.
        width_m = (length * (2.0 * math.pi / n)
                   * max(0.3, min(slit_d, 2.5)))
        return center_deg, width_m, slit_d

    # _clean_ranges: LiDAR'ın çöp değerlerini (NaN, inf, menzil dışı)
    # range_max ile değiştirir; planlayıcı hep temiz sayılar görür.
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
        Disparity extender (süreksizlik genişletici): iki komşu ışın
        arasında ani derinlik sıçraması varsa, YAKIN okumayı robotun yarı
        genişliğini örtecek açısal yay boyunca komşulara kopyalar.
        Böylece robotun sığamayacağı dar aralıklar, planlayıcı daha
        görmeden "geçilemez" olarak işaretlenmiş olur.
        """
        n = len(ranges)
        extended = list(ranges)
        half_width = self.robot_radius * 1.5

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

    # ─── BÖLÜM: FGM boşluk seçimi (IFGM fayda puanıyla) ──────────────────────

    def _best_gap_bearing_deg(self, extended: list, n: int, search_half_deg: int = GAP_SEARCH_HALF_DEG,
                              prev_center_deg: float = None, goal_dir_rad: float = 0.0) -> tuple:
        """
        İleri penceredeki en iyi geçilebilir koşuyu (run) IFGM faydasıyla
        seçer: fiziksel genişlik + goal_dir_rad'a (koridor ekseninin
        robot çerçevesindeki yönü) hiza puanı. Sonra o koşunun İÇİNDEKİ
        en uzak noktaya nişan alır.
        Hiçbir koşu min_gap_width'i aşamazsa tek en-uzak noktaya düşer.
        prev_center_deg seçim histerezisini açar: merkezi ona
        gap_keep_match_deg içinde kalan koşu gap_keep_bonus kadar fazla
        puan alır.
        """
        offsets = list(range(-search_half_deg, search_half_deg + 1))
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
            # Yedek plan: hiçbir koşu min_gap_width'i geçemedi — tek en
            # uzak noktaya nişan al. Bu, planlayıcının en zayıf kararıdır
            # (en uzak nokta yanda ya da bir kenarın arkasında olabilir);
            # o yüzden sessiz kalmaz, info sözlüğüyle telemetriye açılır.
            best_pos = max(range(len(idxs)), key=lambda p: extended[idxs[p]])
            info = {"n_gaps": 0, "fallback": True,
                    "chosen_center_deg": float(offsets[best_pos]),
                    "chosen_width_m": 0.0, "runs": []}
            return float(offsets[best_pos]), extended[idxs[best_pos]], info

        def physical_width(run):
            s, e = run
            angular_width_rad = math.radians(e - s)
            avg_range = sum(extended[idxs[p]] for p in range(s, e + 1)) / (e - s + 1)
            return angular_width_rad * avg_range

        def run_score(run):
            # IFGM faydası (Hossain vd. 2021, Denk. 13):
            #   U = m1 * gap_width + m2 * (pi - delta)
            # m1 = 1 (genişlik zaten metre), m2 = gap_align_weight,
            # delta = boşluk merkezi ile koridor ekseni arasındaki açı
            # (robot çerçevesinde). Eski "dümdüz ileri" önyargısından
            # farkı: hiza, koridorun GERÇEKTE olduğu yöne göre ölçülür.
            # Robot eksenden saptıysa, eksene geri götüren boşluk yüksek
            # puan alır; koridorun gerisine bakan geniş boşluk ise
            # hizadan ~sıfır puan alır.
            s, e = run
            center_rad = math.radians(offsets[(s + e) // 2])
            delta = abs(_angle_diff(center_rad, goal_dir_rad))
            return physical_width(run) + self.gap_align_weight * (math.pi - delta)

        def effective_score(run):
            score = run_score(run)
            if prev_center_deg is not None:
                center = offsets[(run[0] + run[1]) // 2]
                if abs(center - prev_center_deg) <= self.gap_keep_match_deg:
                    score += self.gap_keep_bonus
            return score

        best_run = max(runs, key=effective_score)
        s, e = best_run
        best_pos = max(range(s, e + 1), key=lambda p: extended[idxs[p]])

        run_stats = []
        for r in runs:
            rs, re = r
            run_stats.append({
                "center": offsets[(rs + re) // 2],
                "width": round(physical_width(r), 2),
                "depth": round(max(extended[idxs[p]] for p in range(rs, re + 1)), 2),
                "score": round(run_score(r), 2),
            })
        info = {"n_gaps": len(runs), "fallback": False,
                "chosen_center_deg": float(offsets[(s + e) // 2]),
                "chosen_width_m": physical_width(best_run), "runs": run_stats}
        return float(offsets[best_pos]), extended[idxs[best_pos]], info

    # ─── BÖLÜM: DWA yörünge puanlama (maliyet fonksiyonu) ────────────────────

    # _trajectory_cost: tek bir (v, w) hız adayını predict_time boyunca
    # kafada sürer, engellere olan payı ölçer ve toplam maliyeti döndürür.
    # Maliyet = engel + yön + hız + süreklilik + koridor + yanal sapma.
    # En düşük maliyetli aday kazanır — DWA'nın kalbi burasıdır.
    def _trajectory_cost(self, v: float, w: float, target_bearing_rad: float,
                          obstacles_cartesian: list, prev_w: float,
                          current_yaw: float = None, current_y: float = None,
                          breakdown: bool = False):
        x = y = theta = 0.0
        steps = max(1, int(self.predict_time / self.sim_dt))
        min_clearance = float("inf")

        for step_idx in range(1, steps + 1):
            x += v * math.cos(theta) * self.sim_dt
            y += v * math.sin(theta) * self.sim_dt
            theta += w * self.sim_dt

            if step_idx % 2 == 0 or step_idx == steps:
                for ox, oy in obstacles_cartesian:
                    clearance = math.hypot(ox - x, oy - y) - (self.robot_radius + 0.10)
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

        heading_cost    = self.eff_heading_weight * abs(_angle_diff(target_bearing_rad, theta))
        speed_cost      = self.eff_speed_weight * (self.max_vel - v)
        continuity_cost = self.eff_continuity_weight * abs(w - prev_w)

        if current_yaw is None:
            corridor_cost = 0.0
            lateral_cost  = 0.0
        else:
            # Geriye dönmek pahalı olmalı; uturn_hard_deg ötesi düpedüz
            # yasaktır — çarpışma gibi reddedilir (5e5, U dönüşünü gerçek
            # bir çarpma HARİÇ her şeyden kötü sıralar; köşeye sıkışan
            # robot dönmek yerine wedge-kurtarmanın düz geri gidişini
            # seçer). Duvarın altında ceza karesel büyür (eski not, 60
            # dereceden itibaren: 75 derecede ~0.4, 90'da ~1.8 —
            # varsayılan corridor_weight ile; koddaki eşik 50 derece).
            yaw_diff = abs(_angle_diff(self.desired_yaw, current_yaw + theta))
            if yaw_diff > self._uturn_hard_rad:
                corridor_cost = 5e5 + yaw_diff * 1e4
            elif yaw_diff > math.radians(50.0):
                corridor_cost = self.corridor_weight * ((yaw_diff - math.radians(50.0)) * 4.0) ** 2
            else:
                corridor_cost = 0.0

            if current_y is None:
                lateral_cost = 0.0
            else:
                # Tahmini yörünge bitiş noktası: robotun yerel
                # çerçevesinden dünya çerçevesine döndürülüp robotun
                # canlı dünya-Y'sine eklenir (doğuş noktasına / koridor
                # eksenine göre sapma). NEDEN: bu terim olmadan DWA
                # örnekleyicisini koridor orta hattına geri çeken HİÇBİR
                # sinyal yok — corridor_cost sadece yaw'ı cezalandırır;
                # dümdüz ileri bakan yörünge (yaw_diff=0) eksenden 4m
                # uzakta bile bedavadır. Log'lanan yörüngeler tam bunu
                # gösteriyor: robot bir kez merkezden kayınca asla geri
                # dönmüyor, çünkü maliyet fonksiyonunda dönmesini
                # söyleyen hiçbir şey yok. Ceza karesel (doğrusal değil):
                # küçük, sağlıklı AVOID salınımları ucuz kalır; zaten
                # kaymışken bitişi eksenden daha da uzaklaştıran
                # yörüngeler sertçe caydırılır.
                end_world_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
                lateral_cost = self.lateral_weight * end_world_y ** 2

        total = obstacle_cost + heading_cost + speed_cost + continuity_cost + corridor_cost + lateral_cost
        if breakdown:
            return total, {
                "total": total, "obstacle": obstacle_cost, "heading": heading_cost,
                "speed": speed_cost, "continuity": continuity_cost,
                "corridor": corridor_cost, "lateral": lateral_cost,
            }
        return total

    @staticmethod
    def _state_label(front_min: float, robot_radius: float, v: float, max_vel: float) -> str:
        # Telemetri etiketi: robot şu an hangi "ruh halinde"? Yalnız log
        # ve durum-geçiş mesajları için; kontrolü ETKİLEMEZ.
        # - TIGHT : burnun dibinde engel var (yarıçapın 2.5 katından yakın)
        # - AVOID : hız düşük (max'ın %60'ının altı) => kaçınma yapıyor
        # - CRUISE: yol açık, seyir hızında gidiyor
        if front_min < robot_radius * 2.5:
            return "TIGHT"
        if v < max_vel * 0.6:
            return "AVOID"
        return "CRUISE"

    def _progress_guard(self, v, w, now, tag=""):
        """Hareket bekçisi — kullanıcı emri (deneme 171): robot ASLA
        geri gitmez, ASLA U-dönüşü yapmaz, ASLA pinball gibi sekmez.
        Dışarı çıkan HER komut buradan geçer; yalnız est_heading
        kullanılır (lazer + iç durum; ground truth'a dokunulmaz).
        - v < 0 (geri): koşulsuz veto. (Eskiden "kurs-yukarı bakarken
          geri serbest" istisnası ve 88-92 derecelik tampon pencere
          vardı; ikisi de gözle görülür geri kaçışlar üretti, kalktı.)
        - burun kurstan >92° sapmışsa: ileri veto edilir, burnu ÖN
          taraftan eksene döndüren bir dönüş zorlanır (U-dönüşü engeli).
        - direksiyon işareti son flipten 0.5 s içinde yine terslenirse:
          w sıfıra söndürülür (pinball freni).
        Tüm vetolar sayılır ve loglanır (GUARD_* olayları)."""
        off = _angle_diff(self.est_heading, self.desired_yaw)
        # MUTLAK yalnız-ileri kuralı (kullanıcı: "ROBOT NEVER GOES BACK
        # ONLY GOES FORWARD"): hiçbir koşulda geri yok. Eski
        # "kurs-yukarı bakarken geri serbest" istisnası ve 88-92°
        # sınır penceresi görünür geri kaçışlar üretti; ikisi de gitti.
        # Geri komutu 0'lanır; robot zaten dönmüyorsa açık tarafa doğru
        # bir dönüş eklenir ki sıkışma yerinde saymaya dönüşmesin.
        if v < 0.0:
            self.guard_back_count += 1
            self._guard_log(now, f"BACK off={math.degrees(off):+.0f} {tag}")
            v = 0.0
            if abs(w) < 0.2:
                w = 0.8 if self._last_side_open >= 0.0 else -0.8
        # U-dönüşü bekçisi: burun kurs ekseninden 92°'den fazla sapmışsa
        # ileri hız 0'lanır ve burnu ÖN taraftan (kısa yönden) eksene
        # geri döndüren sabit 0.8 rad/s'lik bir dönüş komutlanır.
        if abs(off) > math.radians(92.0):
            self.guard_uturn_count += 1
            self._guard_log(now, f"UTURN off={math.degrees(off):+.0f} {tag}")
            return 0.0, -math.copysign(0.8, off)
        # Pinball bekçisi: direksiyon işareti (sol/sağ) son 0.5 s içinde
        # arka arkaya iki kez terslenirse ikinci flip söndürülür (w=0).
        # |w| < 0.15 "düz" sayılır ki gürültü flip sanılmasın.
        sgn = 1 if w > 0.15 else (-1 if w < -0.15 else 0)
        if sgn != 0:
            if (self._guard_last_w_sign != 0 and sgn != self._guard_last_w_sign
                    and now - self._guard_last_flip_t < 0.5):
                self.guard_pinball_count += 1
                self._guard_log(now, f"PINBALL w={w:+.2f} {tag}")
                w = 0.0
            else:
                if self._guard_last_w_sign != 0 and sgn != self._guard_last_w_sign:
                    self._guard_last_flip_t = now
                self._guard_last_w_sign = sgn
        return v, w

    # Bekçi olay logu: aynı olay saniyede en çok 1 satır (spam önleme).
    def _guard_log(self, now, msg):
        if self.last_guard_log_t is None or now - self.last_guard_log_t >= 1.0:
            self.last_guard_log_t = now
            self._log(f"EVENT GUARD_{msg}")

    def _publish_recover(self, front_min: float):
        """Sıkışma-kurtarma komutunu yayınlar: recover_linear + _recover_w
        çifti bekçiden (_progress_guard) geçirilip basılır. Bekçi geriyi
        yasakladığı için bu pratikte bir YERİNDE DÖNÜŞTÜR; amaç duvar
        temasını kırıp DWA'ya temiz bir başlangıç vermek."""
        v, w = self._progress_guard(self.recover_linear, self._recover_w,
                                    self._now(), tag="RECOVER")
        self.cmd_linear_x  = v
        self.cmd_angular_z = w
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        if self.last_label != "RECOVER":
            self.get_logger().info(f"[FgmDwaRobot] {self.last_label} → RECOVER  front_min={front_min:.2f}")
            self._log(f"TRANSITION {self.last_label} → RECOVER | front_min={front_min:.2f}")
            self.last_label = "RECOVER"

    def _try_thread(self, ext_scan, n, now, elapsed, front_min, min_r360, cb_t0):
        """Tek THREAD adımı: "iğneye iplik geçirme" mini-durumu.
        Kıvrımlı kapı kanallarında karşıya ışık görünmez (viraj görüş
        hattını keser) ve duvarlar kalıcı sıkışma gibi okunur; burada
        burna yakın EN DERİN ışına doğru emekleriz. Bu taramada komut
        yayınladıysa True döner (çağıran erken çıkar). Yalnız kapılar
        bölgesinde (>=2 kapı geçilmişse) çalışır ki erken sahadaki cep
        güvenliği bozulmasın; 8 s içinde ön açılmazsa pes eder (2 s
        kilit) — yanlış bir thread böylece ucuza atlatılır."""
        if self.door_cleared_count < 2 or now < self._thread_block_until:
            self._thread_start = None
            return False
        if self._thread_start is None:
            if min_r360 <= 0.13:  # giriş şartı: duvara sürtünürken başlama
                return False
        elif min_r360 <= 0.11:
            # Devam fazı biraz daha sıkı sürtünmeye katlanır (0.11):
            # deneme 149'da kusursuz nişanlı bir thread, anlık bir r360
            # düşüşüyle sessizce öldürüldü ve sweep koşuyu alıp gitti.
            # Kilit 1.2 s (eskiden 2.5): duvardan sıyrılacak tek kısa
            # manevraya yetecek kadar, sonra yeniden girilir — uzun
            # kilit, kanaldan tam sıkışma-kurtarmalı fırlatmayı garanti
            # ediyordu (deneme 157 buna ~3 s ve kanalın yarısını verdi).
            self._thread_start = None
            self._thread_block_until = now + 1.2
            self._log(f"THREAD abort | min_r360={min_r360:.2f}")
            return False
        # Arama konisi ±50° (eskiden ±35): kayıtlı ağız-giriş ışınları
        # +33…+35°'te, yani eski koninin tam kırpma kenarındaydı
        # (deneme 177: robot ağızda 10 s dikildi, THREAD hiç girmedi).
        t_half = int(50 * n / 360)
        best_o, best_r = 0, 0.0
        for o in range(-t_half, t_half + 1):
            r = ext_scan[o % n]
            if r > best_r:
                best_r, best_o = r, o
        ray_abs = self.est_heading + math.radians(best_o)
        if self._thread_start is None:
            # Giriş ışını çıtası yüzde-bekleme ile gevşer (ışık
            # eşikleriyle aynı tırmanma mantığı): sıkışmış bir ağız
            # duruşunda kanalın ilk bacağı viraja kadar ancak 0.6-0.8
            # okunabilir; deneme 180 duvarda 20 s debelendi, THREAD hiç
            # devreye girmedi.
            ray_min = 0.85 if self._face_dwell_now <= 4.0 else 0.65
            if (best_r <= ray_min
                    or abs(_angle_diff(ray_abs, self.desired_yaw))
                    > math.radians(90.0)):
                return False
            self._thread_start = now
            self._thread_bearing_ema = None
            self._log(f"THREAD engaged | ray={best_o:+d}deg "
                      f"depth={best_r:.2f} front_min={front_min:.2f}")
        else:
            # Devam fazı "yapışkan": başarı, ışın kaybı ya da zaman
            # aşımına dek sürer. NEDEN: deneme 143'te GİRİŞ koşulu her
            # taramada yeniden sınanıyordu; thread 1.4 s'de öldü ve
            # normal seyrüsefer robotu kanaldan geri çekti.
            if front_min > 1.0:
                self._thread_start = None
                self._log(f"THREAD success | front opened to {front_min:.2f}")
                return False
            if (best_r <= 0.7
                    or abs(_angle_diff(ray_abs, self.desired_yaw))
                    > math.radians(110.0)):
                self._thread_start = None
                self._thread_block_until = now + 2.5
                self._log(f"THREAD lost ray | depth={best_r:.2f}")
                return False
        if now - self._thread_start > 8.0:
            self._thread_block_until = now + 2.0
            self._thread_start = None
            self._log("THREAD gave up — falling back")
            return False
        # Işın yönüne EMA (üstel kayan ortalama) + ölü bant: ham argmax
        # her taramada ışından ışına zıplar ve P-direksiyon gövdeyi
        # kanal duvarlarına dokur (deneme 150: 5 s'de 0.44 m ilerleme,
        # koşu kaybı 1.10 m).
        if self._thread_bearing_ema is None:
            self._thread_bearing_ema = float(best_o)
        else:
            self._thread_bearing_ema = (0.6 * self._thread_bearing_ema
                                        + 0.4 * float(best_o))
        b_eff = self._thread_bearing_ema if abs(self._thread_bearing_ema) >= 4.0 else 0.0
        w_cmd = max(-0.9, min(0.9, 2.0 * math.radians(b_eff)))
        # Tempo burun boşluğuyla ölçeklenir: sürtünürken 0.14, önde hava
        # varsa 0.24'e kadar — sabit yavaş emekleme, 0.8-1.0 m'lik
        # kanalı süre penceresinde bitiremiyordu (deneme 148: hedefe
        # kilitlendi ama düdük çaldığında hâlâ emekliyordu).
        v_thread = 0.14 + 0.25 * max(0.0, min(front_min - 0.25, 0.4))
        v_thread, w_cmd = self._progress_guard(v_thread, w_cmd, now, tag="THREAD")
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v_thread, w_cmd
        self.cmd_linear_x, self.cmd_angular_z = v_thread, w_cmd
        self.cmd_pub.publish(cmd)
        self._emit_telemetry(now, elapsed, "THREAD", front_min,
                             bearing_raw=float(best_o), gap_depth=best_r,
                             min_r360=min_r360, cb_t0=cb_t0)
        return True

    def _corner_ahead(self, extended, n, bearing_rad, half_deg=35):
        """İçbükey köşe dedektörü — ARTIK KULLANILMIYOR (çağıran kalmadı,
        tarihsel kayıt olarak duruyor). Fikir: aday yönün ±half_deg
        yelpazesinde 1.2 m'den derin tek bir şişirilmiş ışın bile yoksa
        o yön bir çıkmaz L-köşesidir; oraya dümen kırmak robotu ancak
        dönerek çıkabileceği bir cebe sokar. True = köşe var (kaçın)."""
        c = int(round(math.degrees(bearing_rad)))
        for o in range(-half_deg, half_deg + 1):
            if extended[(c + o) % n] >= 1.2:
                return False
        return True

    def _grid_plan(self, clean, n, prefer_rel_deg=None, prefer_tol_deg=35.0):
        """Erişilebilirlik mini-grid'i (kullanıcı onaylı öneri): taramayı
        robot merkezli 80x80 hücrelik doluluk grid'ine (hücre 6 cm,
        yani ±2.4 m) rasterize et, yalnız SERBEST hücrelerden geçerek
        taşkın yap ve en çok kurs-aşağı ilerleme vadeden frontier'a
        (serbest/bilinmeyen sınırı) rota çiz. Topoloji, eşik savaşlarının
        yerini alır: bir cebin derin ışınları taşkın için çıkmaz
        sokaktır; gerçek bir kapı — eğik bir kıymık, hatta S-virajın
        öncesi bile — taşkının içinden aktığı bir koridordur. Yalnız
        lazer + iç durum kullanılır. Dönüş: (path_m, ilerleme_m) —
        path_m robot çerçevesinde metre cinsinden ara nokta listesi —
        ya da plan yoksa None."""
        CS, HALF, CAP = 0.06, 40, 3.4
        SIZE = 2 * HALF
        FREE, UNK, OBS = 1, 0, 2
        grid = [bytearray(SIZE) for _ in range(SIZE)]
        step_deg = max(1, n // 360)
        for i in range(0, n, step_deg):
            r = clean[i]
            ang = i * 2.0 * math.pi / n
            c, sn = math.cos(ang), math.sin(ang)
            rr = min(r, CAP)
            for k in range(1, int(rr / CS)):
                d = k * CS
                gx, gy = HALF + int(d * c / CS), HALF + int(d * sn / CS)
                if 0 <= gx < SIZE and 0 <= gy < SIZE and grid[gx][gy] == UNK:
                    grid[gx][gy] = FREE
            if r < CAP:
                gx, gy = HALF + int(r * c / CS), HALF + int(r * sn / CS)
                if 0 <= gx < SIZE and 0 <= gy < SIZE:
                    grid[gx][gy] = OBS
        # Engelleri robot yarıçapı + pay kadar şişir (3 hücre = 18 cm):
        # böylece robotu tek nokta sayarak plan yapabiliriz.
        blocked = set()
        R = 3
        offs = [(dx, dy) for dx in range(-R, R + 1) for dy in range(-R, R + 1)
                if dx * dx + dy * dy <= R * R]
        for gx in range(SIZE):
            row = grid[gx]
            for gy in range(SIZE):
                if row[gy] == OBS:
                    for dx, dy in offs:
                        x2, y2 = gx + dx, gy + dy
                        if 0 <= x2 < SIZE and 0 <= y2 < SIZE:
                            blocked.add((x2, y2))
        # Açıklık (clearance) alanı: engel hücrelerinden çok-kaynaklı
        # BFS, her serbest hücreye duvara uzaklığını (hücre cinsinden)
        # yazar. Rota sonra "düşük açıklık cezası" ile planlanır ki yol
        # koridorun orta eksenini (medial axis) izlesin — çıplak
        # en-kısa-yol BFS, şişirme sınırına yapışıp kapı kasalarını
        # sıyırıyordu (denemeler 236/237).
        from collections import deque
        import heapq
        INF = 10
        clearance = [[INF] * SIZE for _ in range(SIZE)]
        cq = deque()
        for gx in range(SIZE):
            row = grid[gx]
            for gy in range(SIZE):
                if row[gy] == OBS:
                    clearance[gx][gy] = 0
                    cq.append((gx, gy))
        while cq:
            cx, cy = cq.popleft()
            c1 = clearance[cx][cy] + 1
            if c1 >= INF:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < SIZE and 0 <= ny < SIZE
                        and clearance[nx][ny] > c1):
                    clearance[nx][ny] = c1
                    cq.append((nx, ny))
        # Dijkstra, adım maliyeti = 1 + ceza/açıklık: duvara yakın
        # geçmek pahalı olduğundan yollar kendiliğinden ortalanır.
        start = (HALF, HALF)
        parent = {start: None}
        dist = {start: 0.0}
        heap = [(0.0, start)]
        while heap:
            d0, (cx, cy) = heapq.heappop(heap)
            if d0 > dist.get((cx, cy), 1e18):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if (0 <= nx < SIZE and 0 <= ny < SIZE
                        and (nx, ny) not in blocked
                        and grid[nx][ny] == FREE):
                    cl = max(1, clearance[nx][ny])
                    nd = d0 + 1.0 + 6.0 / cl
                    if nd < dist.get((nx, ny), 1e18):
                        dist[(nx, ny)] = nd
                        parent[(nx, ny)] = (cx, cy)
                        heapq.heappush(heap, (nd, (nx, ny)))
        # En iyi frontier: BİLİNMEYEN'e değen, erişilebilir SERBEST
        # hücre; puanı kurs-aşağı ilerleme miktarı. Yön vetosu az
        # aşağıda (±75°).
        dc = _angle_diff(self.desired_yaw, self.est_heading)
        cdc, sdc = math.cos(dc), math.sin(dc)
        best, best_prog = None, 0.35  # en az 0.35 m ilerleme şartı
        for (cx, cy) in parent:
            if cx == HALF and cy == HALF:
                continue
            gr = grid[cx][cy]
            is_frontier = False
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < SIZE and 0 <= ny < SIZE and grid[nx][ny] == UNK:
                    is_frontier = True
                    break
            if not is_frontier:
                continue
            dx_m, dy_m = (cx - HALF) * CS, (cy - HALF) * CS
            # Açısal veto (dokümante edilmişti ama hiç kodlanmamıştı —
            # deneme 205, −135°/+180°'deki frontier'ların peşine düştü):
            # frontier'ın kendisi kurs-aşağının ±75° içinde olmalı.
            th = math.atan2(dy_m, dx_m)
            if abs(_angle_diff(th, dc)) > math.radians(75.0):
                continue
            # Frontier sabitleme (deneme 263): commit sırasında çağıran,
            # frontier seçimini taahhüt edilen kapının yakınına iğneler;
            # böylece takip, kapı ile koy arasında taramadan taramaya
            # zıplayamaz (deneme 262: hedef titremesi yüzünden 6.6 s'lik
            # duvar commit'i TIMEOUT yedi).
            if (prefer_rel_deg is not None
                    and abs(_angle_diff(th, math.radians(prefer_rel_deg)))
                    > math.radians(prefer_tol_deg)):
                continue
            prog = dx_m * cdc + dy_m * sdc
            if prog > best_prog:
                best_prog, best = prog, (cx, cy)
        if best is None:
            return None
        # Tam yol: parent zinciri frontier'dan başlangıca geri sarılır,
        # ters çevrilir ve robot çerçevesinde metreye dönüştürülür.
        node, chain = best, []
        while node is not None:
            chain.append(node)
            node = parent[node]
        chain.reverse()
        path_m = [((cx2 - HALF) * CS, (cy2 - HALF) * CS)
                  for (cx2, cy2) in chain]
        return path_m, best_prog

    def _grid_door_bearing(self, clean, n):
        """_grid_plan üstüne yön sarmalayıcısı: erişilebilir kurs-aşağı
        yolun ~0.6 m ilerideki ara noktasına (10. hücresine) bakan açıyı
        ve ilerleme miktarını döndürür; plan yoksa None."""
        gp = self._grid_plan(clean, n)
        if gp is None:
            return None
        path_m, prog = gp
        wp = path_m[min(len(path_m) - 1, 10)]
        return math.atan2(wp[1], wp[0]), prog

    def _publish_door_commit(self, front_min: float, target_bearing_rad: float):
        """Taahhüt edilen (COMMIT) kapıdan geçiş sürüşü. Hedef yön her
        taramada dar-koni boşluk aramasıyla YENİDEN hesaplanır (COMMIT
        girişinde dondurulmaz). NEDEN: dünya çerçevesinde donmuş bir
        hedef, geçiş ortasında S-manevra isteyen kademeli/şikan açıklığı
        izleyemez — robot giriş yönünde dümdüz sürüp ikinci kıvrımın
        kenarını sıyırırdı. Her taramada yeniden nişan almak, komutlanan
        yönün kayan boşluğu takip etmesini sağlar; ama FGM/DWA'nın tam
        engel-kaçınma maliyet fonksiyonu yine de ATLANIR (o fonksiyon
        ileri-taahhüt niyetiyle kavga edip spin döngüleri üretiyordu —
        önceki incelemenin bulgusu)."""
        w = max(-self.door_center_max_w,
                min(self.door_center_max_w, target_bearing_rad * self.door_center_gain))

        # Takılma (stall) kurtarması: commit bir süredir sürüyor ama
        # front_min hâlâ sıkışma tabanına yakın yapışıksa (fakat
        # wedge_front_dist ile tetiklenen sıkışma-kurtarmayı çağıracak
        # kadar düşük DEĞİLSE), robot bir kolon kenarına takılmıştır.
        # Salt ileri+dönüş, linear.x hiç eksiye inmediği için robotu
        # köşeye öğütür; kenardan fiziksel olarak kurtulana dek dönüşle
        # beklemek gerekir.
        elapsed_commit = (self._now() - self.door_commit_start_time
                           if self.door_commit_start_time is not None else 0.0)
        stalled = (elapsed_commit > self.door_stall_nudge_delay
                   and front_min < self.door_stall_front_dist)
        if stalled:
            nudge_sign = 1.0 if target_bearing_rad >= 0.0 else -1.0
            w = nudge_sign * self.door_stall_nudge_w
            # İnceleme düzeltmesi (dürüstlük): burası eskiden
            # recover_linear (-0.20) komutluyordu; hareket bekçisinin
            # mutlak geri-yasağı bunu her taramada sıfırlıyordu — yani
            # yukarıdaki "kısa geri kaçış" hiç çalışmadı. Kurala uygun
            # davranış (aslında hep olan şey) ışık tarafına yerinde
            # pivot dönüş; onu artık açıkça komutluyoruz.
            linear_x = 0.0
        else:
            # F1TENTH usulü "direksiyona göre hız": tam commit hızı ancak
            # geçide dik (kare) hizadayken; açılı girişler kendiliğinden
            # yavaşlar (komutlanan yönün cos²'si, 40° kilitte taban
            # 0.59). NEDEN: tam hızda eğik commit'ler 5.5 s'lik
            # kör-öğütme ölümleriydi.
            linear_x = (self.door_commit_speed
                        * max(0.55, math.cos(target_bearing_rad) ** 2))

        if stalled != self.door_stall_active:
            self.door_stall_active = stalled
            if stalled:
                self.get_logger().info(f"[FgmDwaRobot] Door stall — backing off with turn "
                              f"(front_min={front_min:.2f}, nudge_w={w:+.2f}).")
                self._log(f"DOOR STALL engaged | front_min={front_min:.2f} "
                          f"elapsed_commit={elapsed_commit:.2f} nudge_w={w:+.2f}")
            else:
                self.get_logger().info("[FgmDwaRobot] Door stall cleared — resuming forward commit.")
                self._log(f"DOOR STALL released | front_min={front_min:.2f}")

        w = max(-self.door_center_max_w, min(self.door_center_max_w, w))
        linear_x, w = self._progress_guard(linear_x, w,
                                           self._now(),
                                           tag="COMMIT")
        self.cmd_linear_x  = linear_x
        self.cmd_angular_z = w
        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    # ─── BÖLÜM: Ana tarama callback'i (her LiDAR karesinde bir kez) ─────────

    def scan_callback(self, msg: LaserScan):
        # Her şeyin kalbi. Akış sırası: taramayı temizle -> sinyalleri
        # türet (front_min, disparity, yön tahmini) -> kurtarma / THREAD /
        # kapı durum makinesi öncelik sırasıyla söz alır -> hiçbiri söz
        # almazsa FGM+DWA normal seyrüseferi sürer.
        if self._is_shutdown:
            return
        cb_t0 = time.perf_counter()
        now = self._now()

        # İlk tarama = kronometre başlar (60 saniyelik koşu buradan sayılır).
        if not self.scan_received:
            self.scan_received = True
            self.start_time    = now
            self.get_logger().info("[FgmDwaRobot] First scan received. 60-second run started.")
            self._log("FIRST SCAN — timer started")

        # Callback'ler arası 0.5 s'den uzun boşluk = tarama kaybı; sayılıp
        # loglanır (simülatör tıkanmalarını teşhis etmek için).
        self.scan_count += 1
        if self.last_cb_time is not None and now - self.last_cb_time > 0.5:
            self.scan_gap_count += 1
            self._log(f"SCAN GAP {now - self.last_cb_time:.2f}s between callbacks "
                      f"(scan #{self.scan_count})")
        # Entegrasyon adımı dt: iki callback arası süre, 0-0.3 s'ye
        # kırpılır — uzun bir tıkanma yön tahminini patlatmasın diye.
        dt_int = (max(0.0, min(0.3, now - self.last_cb_time))
                  if self.last_cb_time is not None else 0.0)
        self.last_cb_time = now

        # Süre doldu mu? 60 s bitince tek seferlik kapanış raporu ve dur.
        elapsed = now - self.start_time
        if elapsed >= self.run_duration:
            self._shutdown()
            return

        # Taramayı temizle: NaN/inf/sıfır ışınlar kullanılabilir hale gelir.
        n = len(msg.ranges)
        clean = self._clean_ranges(msg.ranges)

        # Burun konisi (±20°) içindeki en yakın engel = front_min. Çoğu
        # güvenlik kararının ana sinyali; koşu rekoru da kaydedilir.
        front_idxs = [int(o * n / 360) % n for o in range(-20, 21)]
        front_min  = min(clean[i] for i in front_idxs)
        if front_min < self.min_front_min_seen:
            self.min_front_min_seen = front_min
            self.min_front_ctx = (elapsed, self.x, self.y)
        # 360° minimum: front_min'in ±20° konisi YANDAN duvar sıyırmayı
        # göremez — geç kapı çarpışmaları tam da böyle ortaya çıkar.
        min_r360 = min(clean)
        if min_r360 < self.min_r360_seen:
            self.min_r360_seen = min_r360

        # Disparity extender (F1TENTH kenar şişirme) erken hesaplanır:
        # THREAD, aramasını ŞİŞİRİLMİŞ ışınlar üstünde yapmalı ki
        # bulduğu en derin ışın geçilebilir bir koridorun ORTASI olsun,
        # kapı kasasına yapışık bir kıymık değil. NEDEN: ham-ışın
        # thread'leri 0.11'lik gövde sürtmelerinin ve kanal içi
        # yalpalamanın kaynağıydı.
        radians_per_sample = 2.0 * math.pi / n
        extended = self._disparity_extend(clean, radians_per_sample)

        # Gerçek hareketin lazer kanıtı: iki tarama arasındaki ortalama
        # |mesafe değişimi|. Hem yön tahmini (hemen aşağıda) hem de
        # "yerinde donduk mu" dedektörü (daha aşağıda) bunu paylaşır.
        scan_change = None
        prev_scan = (self._prev_scan_full
                     if (self._prev_scan_full is not None
                         and len(self._prev_scan_full) == n) else None)
        if prev_scan is not None:
            scan_change = sum(abs(clean[i] - prev_scan[i])
                              for i in range(0, n, 4)) / (n / 4.0)
        self._prev_scan_full = clean

        # Yön (heading) tahmincisi: "komut kadar döndüm" öngörüsü +
        # koridor doğrultusu düzeltmesi — deneme 53'ün biçimi. Tarama
        # eşleme ve güven-ağırlıklı varyantların (denemeler 66/67/72)
        # HEPSİ daha kötü ölçüldü (pusula ort. hatası 82-85 vs ~30;
        # erken saha ölümleri 3'te 1'den 4'te 3'e çıktı), o yüzden bu
        # forma geri dönüldü. Yalnız sarma (wrap) düzeltmesi kaldı:
        # tahmin artık sürtünme sırasında sınırsız kaçamıyor.
        self.est_heading += self.cmd_angular_z * dt_int
        # Temas-dönmesi düzeltmesi: yavaş/temaslı fazlarda (emekleme,
        # geri, sürtünme — savrulmaların doğduğu, öteleme paralaksının
        # ihmal edilebilir olduğu anlar) taramadan taramaya GERÇEK
        # dönüşü ölç ve komutsuz kısmını tahmine ekle. Seyir hızında ve
        # hızlı komutlu spinlerde atlanır (korelasyon penceresi
        # kırpılırdı). Bu, deneme 67'nin global tarama eşleyicisi DEĞİL
        # (o, tahminciyi komple değiştirmişti ve kaybetti); burada
        # yalnız komutların hiç yapmadığı fiziksel dönme yamalanır.
        cmd_rot = self.cmd_angular_z * dt_int
        # COMMIT sırasında hız ne olursa olsun aktif: yarığın içinde yan
        # duvarlar harekete paraleldir, öteleme taramayı pek değiştirmez
        # ve dönme korelasyonu temiz kalır — üstelik kasa kenarında
        # commit hızıyla sürtünmek, tam da deneme 160'taki savrulmanın
        # doğduğu yerdi (0.55 m/s'de temas torku).
        contact_yaw_ok = (abs(self.cmd_linear_x) <= 0.25
                          or self.door_mode == "COMMIT")
        if (prev_scan is not None and contact_yaw_ok
                and abs(cmd_rot) < math.radians(15.0)):
            rot = self._scan_rotation(prev_scan, clean, n)
            if rot is not None:
                dyaw_scan, rot_q, _rot_cost = rot
                residual = dyaw_scan - cmd_rot
                if rot_q >= 0.15 and abs(residual) > math.radians(2.5):
                    self.est_heading += residual
                    if (self.last_contact_yaw_log_t is None
                            or now - self.last_contact_yaw_log_t >= 0.5):
                        self.last_contact_yaw_log_t = now
                        self._log(f"EVENT CONTACT_YAW | residual="
                                  f"{math.degrees(residual):+.1f}deg q={rot_q:.2f} "
                                  f"cmd_w={self.cmd_angular_z:+.2f}")
        self.est_heading = _angle_diff(self.est_heading, 0.0)
        meas = self._corridor_heading(clean, n, self.est_heading)
        if meas is None:
            # Bayat-tutunma koruması: art arda 15 ret (~1.5 s) sonrası
            # gevşetilmiş sıçrama limitiyle yeniden kilitlen — ama YALNIZ
            # açık sahnede. Sıkışık bir yüzde dik kapı çıkıntıları oyu
            # domine eder; orada gevşek yeniden-kilit, pusulanın tam da
            # U-dönüşü duvarına en çok ihtiyaç duyulan anda 90° yanlış
            # kalması demektir (deneme 1: yüzde iki fiziksel geri dönüş).
            self._heading_reject_streak += 1
            if self._heading_reject_streak >= 15 and front_min > 1.5:
                meas = self._corridor_heading(clean, n, self.est_heading,
                                              max_jump_rad=math.pi / 2)
        if meas is not None:
            measured, conf = meas
            self.est_heading += 0.15 * _angle_diff(measured, self.est_heading)
            self._heading_reject_streak = 0
        # Ölü hesap (dead-reckoning) izi: sadece tanılama/telemetri için.
        self.dr_heading += self.cmd_angular_z * dt_int
        self.dr_x += self.cmd_linear_x * math.cos(self.est_heading) * dt_int
        self.dr_y += self.cmd_linear_x * math.sin(self.est_heading) * dt_int

        # Yalnız-lazer "gerçekten sıkıştık" testi: hareket komutlanıyor
        # ama tarama donmuş görünüyorsa robot fiziken duruyor demektir.
        if scan_change is not None:
            commanding = abs(self.cmd_linear_x) > 0.10
            if commanding and scan_change < 0.015:
                self._static_streak += 1
            else:
                self._static_streak = 0
        # 8 tarama üst üste donma -> kapı durumunu bırak, kurtarmayı arm
        # et (tekrarlar 8 s penceresinde birikip süreyi katlar, en çok 4x).
        if self._static_streak >= 8 and self.recover_until is None:
            self._static_streak = 0
            self.door_mode = None
            self.door_commit_start_time = None
            self.door_cooldown_until = now + self.door_retrigger_cooldown
            self._recover_recent = [t for t in self._recover_recent if now - t < 8.0]
            self._recover_recent.append(now)
            self.recover_until = now + self.recover_seconds * min(4, len(self._recover_recent))
            self._recover_w = 0.0
            self.wedge_trigger_count += 1
            self.get_logger().info("[FgmDwaRobot] STATIC-STUCK — scans frozen while commanding motion; reversing.")
            self._log(f"RECOVER triggered | STATIC_STUCK front_min={front_min:.2f}")

        # Yan-sıkışma (SIDE_PIN) bekçisi — 10'luk doğrulama koşusunun
        # bulgusu: 163131/163604 koşularında robot gövdesinin YANI bir
        # saha sütununa dayandı (min_r360 0.12, front 0.99 açık!),
        # v=0.24 komutlanırken 40 s boyunca kımıldamadı. Wedge bekçisi
        # ön temas ister, STATIC_STUCK ise donmuş tarama ister — ama
        # w'nin ± titremesi taramayı her karede azıcık döndürüp onu
        # atlatıyordu. Çözüm dönme-telafili test: 2 s önceki taramayla
        # bugünkü tarama, EN İYİ dönme kaydırması denendikten sonra
        # hâlâ neredeyse aynıysa, robot dönme dışında hiç yer
        # değiştirmemiştir (0.24 m/s'de 2 s = 0.48 m → artık dev fark
        # olurdu). O zaman kurtarma dönüşünü başlat.
        if abs(self.cmd_linear_x) >= 0.10:
            if self._fwd_cmd_since is None:
                self._fwd_cmd_since = now
        else:
            self._fwd_cmd_since = None
        if self._pin_ref_scan is None or len(self._pin_ref_scan) != n:
            self._pin_ref_scan, self._pin_ref_t = list(clean), now
        elif now - self._pin_ref_t >= 2.0:
            pinned = False
            if (self._fwd_cmd_since is not None
                    and now - self._fwd_cmd_since >= 2.0
                    and min_r360 < 0.30
                    and self.recover_until is None):
                rot = self._scan_rotation(self._pin_ref_scan, clean, n)
                if rot is not None and rot[2] < 0.06:
                    pinned = True
            self._pin_ref_scan, self._pin_ref_t = list(clean), now
            if pinned:
                self.door_mode = None
                self.door_commit_start_time = None
                self.door_cooldown_until = now + self.door_retrigger_cooldown
                self._recover_recent = [t for t in self._recover_recent
                                        if now - t < 8.0]
                self._recover_recent.append(now)
                self.recover_until = now + max(
                    self.recover_seconds * min(4, len(self._recover_recent)),
                    3.0 if len(self._recover_recent) >= 2 else 0.0)
                self._recover_w = 0.0
                self.wedge_trigger_count += 1
                self.get_logger().info("[FgmDwaRobot] SIDE-PIN — no translation for 2s while commanding forward; rotating out.")
                self._log(f"RECOVER triggered | SIDE_PIN min_r360={min_r360:.2f} "
                          f"front_min={front_min:.2f}")

        # ── BÖLÜM: Sıkışma (wedge) kurtarması — aktifken tek söz sahibi ──
        if self.recover_until is not None:
            # Erken bırakma: bekçi geriyi yasakladığı için kurtarma bir
            # DÖNÜŞTÜR — burun açılınca (front > 0.6) dönmeye devam
            # etmek sadece saat yakar (deneme 171: iki kurtarmada ~7 s
            # gitti).
            # Tekrar-sıkışma tırmanışı (deneme 313): köşe cebinden
            # 0.6'da bırakmak yeniden sıkıştırır — denemeler 311/312 son
            # 10-12 saniyelerini RECOVER↔SWEEP itişmesinde yaktı; front
            # 0.17-0.24'te her 2-4 s'de yeniden sıkıştılar. 8 s içindeki
            # 2. ve sonraki kurtarmalarda burun GERÇEKTEN açılana dek
            # (front > 1.0) dönmeye devam et.
            release_thr = 0.6 if len(self._recover_recent) < 2 else 1.0
            if now < self.recover_until and front_min <= release_thr:
                self._publish_recover(front_min)
                self._emit_telemetry(now, elapsed, "RECOVER", front_min,
                                     min_r360=min_r360, cb_t0=cb_t0)
                return
            self.recover_until = None
            self.wedge_count = 0
            self.get_logger().info(f"[FgmDwaRobot] Recovery complete — front_min={front_min:.2f}. Resuming.")
            self._log(f"RECOVER complete | front_min={front_min:.2f}")

        # Aktif THREAD en yüksek öncelikle sürer (yapışkan mini-durum):
        # kanalın ortasındayken başka hiçbir modun sözü geçmez.
        if self._thread_start is not None and self.door_mode is None:
            if self._try_thread(extended, n, now, elapsed, front_min,
                                min_r360, cb_t0):
                return

        # Sıkışma sayacı: burun eşiğin altında kaldığı her tarama +1; bir
        # kez açılınca sıfırlanır (tek karelik gürültü tetikleyemez).
        if front_min < self.wedge_front_dist:
            self.wedge_count += 1
        else:
            self.wedge_count = 0

        if self.wedge_count >= self.wedge_consecutive:
            # İğne-iplik şansı: kıvrımlı kapı kanalında sıkışma alarmı
            # kanal DUVARLARINA öter ama derin bir ışın sıradaki bacağı
            # işaret eder — geri çıkmak, robotu geçilebilir bir pasajdan
            # fırlatmak olur (deneme 141: kapı 3'ün ağzına üç giriş,
            # front 0.18'de üç fırlatma, virajın ötesinde görünür ışık
            # sıfır).
            # door_mode kapısı (inceleme düzeltmesi): SWEEP/ALIGN/COMMIT
            # robota sahipken THREAD başlatmak _thread_start'ı öksüz
            # bırakır (devam yolu door_mode None ister) ve bu, 8 s'ye
            # kadar her kapı tetiğini veto eder.
            if self.door_mode is None and self._try_thread(
                    extended, n, now, elapsed, front_min,
                    min_r360, cb_t0):
                return
            # Tırmanma: 8 s içindeki her ek tetik kurtarma süresini
            # katlar (en çok 4x) — tek kısa kurtarmanın çözemediği
            # ceplere daha uzun süre tanınır. (Dönüş bileşeni YOK;
            # nedeni aşağıdaki "SADECE düz kurtarma" notunda.)
            self._recover_recent = [t for t in self._recover_recent if now - t < 8.0]
            self._recover_recent.append(now)
            n_recent = len(self._recover_recent)
            # Sigorta (circuit breaker): kapılar bölgesinde 8 s içinde
            # 3+ sıkışma-kurtarma = kurtarma dönüşünün çözemeyeceği bir
            # köşe cebi (deneme 206: bütün final TIGHT/RECOVER
            # döngüsünde geçti). Döngüde dönmek yerine kontrolü
            # duvar-yürüyücüye (SWEEP) devret.
            if (n_recent >= 3 and self.sweep_mode and self.door_mode is None
                    and self.door_cleared_count >= self.sweep_min_doors
                    and self._thread_start is None):
                self.door_mode = "SWEEP"
                self._sweep_start = now
                self._sweep_flips = 0
                self._sweep_stall_streak = 0
                self._sweep_last_flip = now
                self._sweep_dir = 1.0 if self._last_side_open >= 0 else -1.0
                self._recover_recent = []
                self.wedge_count = 0
                self.get_logger().info("[FgmDwaRobot] Recovery loop — forcing SWEEP.")
                self._log(f"TRANSITION door_mode None -> SWEEP | FORCED_BY_RECOVERY_LOOP "
                          f"dir={'L' if self._sweep_dir > 0 else 'R'}")
                # İnceleme düzeltmesi: burada recover_until KURMA —
                # kurtarma bloğu SWEEP dalından ÖNCE koşar; 3 s'lik bir
                # kurtarma robotu sahiplenirken sweep'in (daha yeni
                # başlamış) zaman aşımı ve flip-tolerans saatleri boşa
                # yanardı. Bu taramada bekçili tek bir yerinde dönüş
                # yayınla; SWEEP dalı gerçek devri sonraki taramada alır.
                self._recover_w = 0.0
                self.wedge_trigger_count += 1
                self._publish_recover(front_min)
                self._emit_telemetry(now, elapsed, "RECOVER", front_min,
                                     min_r360=min_r360, cb_t0=cb_t0)
                return
            # Tekrarlanan sıkışmalara, gerçekten açık bir yüze dönecek
            # kadar süre tanınır (bırakma onlar için front > 1.0 arar;
            # 0.8 rad/s'de 2 s zaten sadece ~90° eder).
            self.recover_until = now + max(
                self.recover_seconds * min(4, n_recent),
                3.0 if n_recent >= 2 else 0.0)
            # SADECE düz kurtarma: deneme 32'de gerideki dönüş bileşeni
            # robotu U-dönüşü duvarının erişimi dışına fiziksel olarak
            # döndürdü (kurtarma DWA'yı atlar) — ters-yön sayısı 1'den
            # 5'e fırladı. Süreyi tırmandır, dönüşü asla.
            self._recover_w = 0.0
            self.wedge_count   = 0
            self.wedge_trigger_count += 1
            self.get_logger().info(f"[FgmDwaRobot] Wedged — front_min={front_min:.2f}. Backing up.")
            self._log(f"RECOVER triggered | front_min={front_min:.2f}")
            self._publish_recover(front_min)
            self._emit_telemetry(now, elapsed, "RECOVER", front_min,
                                 min_r360=min_r360, cb_t0=cb_t0)
            return


        # ── BÖLÜM: FGM boşluk seçimi ve hedef yön ────────────────────────
        # Koridor ekseninin robot çerçevesindeki yönü, tarama-tabanlı yön
        # tahmininden gelir (yalnız-lazer kuralı: kontrolde odom/IMU yok).
        goal_dir_rad = _angle_diff(self.desired_yaw, self.est_heading)
        bearing_deg, gap_depth, gap_info = self._best_gap_bearing_deg(
            extended, n, prev_center_deg=self.fgm_prev_center_deg,
            goal_dir_rad=goal_dir_rad)

        # Planlayıcının en zayıf kararlarını birinci sınıf olay yap:
        # fallback (hiç boşluk yok) ve ani boşluk değişimi loglanır.
        if gap_info["fallback"]:
            self.fgm_fallback_count += 1
            if self.last_fallback_log_t is None or now - self.last_fallback_log_t >= 1.0:
                self.last_fallback_log_t = now
                self.get_logger().info(f"[FgmDwaRobot] FGM FALLBACK — no navigable gap, aiming at "
                              f"farthest point ({bearing_deg:.0f}°, {gap_depth:.2f}m).")
                self._log(f"EVENT FGM_FALLBACK | bearing={bearing_deg:.0f} depth={gap_depth:.2f} "
                          f"front_min={front_min:.2f} yaw={math.degrees(self.yaw):.1f}")
                self._dump_scan(elapsed, "FGM_FALLBACK", extended)
        else:
            if (self.fgm_prev_center_deg is not None
                    and abs(gap_info["chosen_center_deg"] - self.fgm_prev_center_deg) > 25.0):
                self.gap_switch_count += 1
                self._log(f"EVENT GAP_SWITCH #{self.gap_switch_count} | "
                          f"{self.fgm_prev_center_deg:.0f} -> {gap_info['chosen_center_deg']:.0f}deg "
                          f"n_gaps={gap_info['n_gaps']} runs={gap_info['runs']}")
        self.fgm_prev_center_deg = (None if gap_info["fallback"]
                                    else gap_info["chosen_center_deg"])
        # Alçak geçiren filtre (EMA) yalnız FGM hedef YÖNÜNE uygulanır
        # (DWA'nın komutladığı açısal hıza DEĞİL). bearing_deg, ileri
        # arama penceresi (GAP_SEARCH_HALF_DEG) içinde düz bir derece
        # ofsetidir; asla 0-360'lık tam yön değildir ve +/-100°'yi aşıp
        # sarmaz — yani iki yakın ham okumayı doğrusal EMA ile harmanlamak
        # burada her zaman doğrudur, açı-sarma özel muamelesi gerekmez
        # (+179 ile -179 pusula yönlerini harmanlamaya benzemez).
        self.smoothed_bearing_deg = (
            self.bearing_smoothing_alpha * bearing_deg
            + (1.0 - self.bearing_smoothing_alpha) * self.smoothed_bearing_deg
        )
        # Lazerle koridor ortalama: DWA yön hedefini daha açık tarafa
        # doğru eğ. Yana kaymış bir kapıdan geçince robot o duvara
        # yapışıyor (dört koşu, kapı 3 öncesi sol köşede gt_x=-1'de öldü;
        # kapı 3'ün kademeli açıklığı o köşeden görünmüyor). Ölü hesap
        # yanal tahmini ortalamak için fazla kaymış; ama sol/sağ
        # açıklıklar dengesizliği doğrudan ölçer.
        goal_deg = math.degrees(goal_dir_rad)
        half15 = int(15 * n / 360)
        l_dir = int(round((goal_deg + 90.0) * n / 360.0))
        r_dir = int(round((goal_deg - 90.0) * n / 360.0))
        left_clear = min(min(extended[(l_dir + o) % n]
                             for o in range(-half15, half15 + 1)), 3.0)
        right_clear = min(min(extended[(r_dir + o) % n]
                              for o in range(-half15, half15 + 1)), 3.0)
        center_bias_deg = max(-self.center_bias_max_deg,
                              min(self.center_bias_max_deg,
                                  self.center_bias_gain * (left_clear - right_clear)))
        self._last_side_open = left_clear - right_clear
        target_bearing_rad = math.radians(self.smoothed_bearing_deg + center_bias_deg)

        # ── BÖLÜM: Kapı-geçiş durum makinesi ─────────────────────────────
        # Sıkışma-kurtarmadan SONRA (öncelik onda kalır), DWA'dan ÖNCE
        # koşar. COMMIT, erken return ile DWA maliyet puanlamasını komple
        # atlar (o puanlama ileri-taahhüt niyetiyle kavga edip spin
        # döngüleri üretiyor) ama aynı dar-koni FGM boşluk aramasını her
        # taramada yeniden kullanır; böylece geçiş ortasında yana kayan
        # kademeli/şikan açıklık izlenebilir — COMMIT girişinde donmuş
        # tek bir yön bunu yapamazdı. ALIGN aşağı düşer (fall-through);
        # normal FGM/DWA tamamen değişmeden çalışır.

        # Duyusal kapı tetiği: "önü dar" = koridor ekseninin İLERİ
        # konisindeki en geniş gezilebilir boşluk tetik genişliğinin
        # altında (ya da hiç yok) VE ön zaten kısıtlı. Anahtar İLERİ
        # koni, seçilen boşluk DEĞİL: kapı yüzünde fayda-seçici haklı
        # olarak duvarın yanındaki geniş alanı tercih eder, yani seçilen
        # boşluğun genişliği asla "dar" okumaz (batch 20260710_201737:
        # sıfır ALIGN, sıfır kapı). Streak sayaçları tek-tarama
        # gürültüsünü iki yönde de süzer.
        # Kapı tetiği "kapıdan sızan ışık" ilkelidir: duvar yüzünde (ön
        # kısıtlı) DAR ama derin bir yay — ötesinde alan olan bir yarık.
        # Kör duvarda ışık yok (tetik yok, DWA uzaklaşır); açık arazide
        # ışık geniş (yine tetik yok).
        # Mesafe kapının YAPISINA (kasa kenarlarına) ölçülür, tam-karşı
        # ışına değil: yarıkla hizalanınca front_min kapının İÇİNDEN
        # geçen derin mesafeyi okur; front_min tabanlı her "kapıya yakın
        # mıyım" şartı tam hizalama başarılı olunca kilitlenirdi (deneme
        # 5: beş ALIGN'ın beşi de açıklığın içine bakarken front
        # 1.2-1.9 m'de zaman aşımına uğradı).
        frame_half = int(40 * n / 360)
        frame_min = min(clean[o % n] for o in range(-frame_half, frame_half + 1))

        # Yüzde-bekleme (face-dwell): temiz bir commit olmadan bir yüzün
        # dibinde ne kadar süredir park halindeyiz (yüz uzaklaşınca ya da
        # COMMIT'e girince sıfırlanır). BURADA hesaplanır çünkü aşağıdaki
        # derinlik eşiği buna bağlı.
        if frame_min < 0.95 and self.door_mode != "COMMIT":
            if self._face_dwell_start is None:
                self._face_dwell_start = now
        elif frame_min > 1.6 or self.door_mode == "COMMIT":
            # 1.6, 1.3 değil: köşe cebinde kasa okuması robot hâlâ
            # kapana kısılıyken 1.3'ün üstüne sıçrayabiliyor; bu, bekleme
            # sayacını sıfırlayıp 6 s'lik sweep şartını baştan
            # kurduruyordu (deneme 153: 18 s yeniden-devreye gecikmesi).
            self._face_dwell_start = None
        face_dwell = (now - self._face_dwell_start
                      if self._face_dwell_start is not None else 0.0)
        self._face_dwell_now = face_dwell

        # Işık hafızası (deneme 230): yüzde görülen son kapı-kalitesinde
        # ışığın MUTLAK yönünü hatırla — duvar-1 sürüklenmesi, kapı gözle
        # görüldükten SONRA oluyor; çünkü ondan sonra her sezgisel açık
        # köşeye oy veriyor ve o görüş unutuluyor.
        # (kayıt aşağıda, ışık hesaplandıktan sonra yapılır)

        # Uyarlanır geçiş-derinliği: bir pasaj, YÜZDEN BELİRGİN DERİN
        # olan ışıktır; mutlak bir sabitten derin olan değil. Oda tipi
        # kapıda yarığın ötesindeki alan kendisi de sınırlı (~2.0-2.1 m
        # görünür); sabit 2.2 m testi gerçek, geçilebilir bir boşluğu
        # "ışık yok" diye okudu ve DEPTH_LOST oradaki her commit'i iptal
        # etti (deneme 126/127 tarama dökümleri: 0.7 m'lik duvarların
        # çerçevelediği, tam karşıda 2.0-2.16 m'lik kanal; robot çekip
        # gitti). Taban 1.2 m, en az ~1 m geçiş odası bırakır; sabit
        # eşik tavan olarak kalır ki açık saha davranışı değişmesin.
        # Bekleme gevşemesi TÜM devreye-girme ışıklarına uygulanır (ana
        # ±75° ışık, sweep door_found, grab): sıkı pay varışta
        # koy-commit'lerine karşı korur; ama 4 s park edip hiçbir şey
        # bulamadıysak sığ S-kanallar (kapı 3, ~1.4-1.8 m görünür)
        # görünür OLMALI — deneme 140'ın sweep'i kapı-3 ağzının önünden
        # yürüyüp geçti çünkü gevşeme yalnız grab'de vardı.
        margin = (0.9 if face_dwell > 4.0 else self.door_depth_margin)
        eff_depth_thr = min(self.door_through_depth,
                            max(1.2, frame_min + margin))
        # GRAB (ağız-kapma) her zaman SIKI payı kullanır: 0.55 m/s ile
        # körlemesine commit eder; gevşetilmiş eşik altında sığ koy cebi
        # yine "kapılabilir" diye ışıldardı (deneme 155 = deneme 134'ün
        # hastalığı, bu kez grab yolundan). Arama ışıkları (sweep/align)
        # gevşek eşikte kalır; ışıksız/sığ yuvalar THREAD'in işidir.
        strict_depth_thr = min(self.door_through_depth,
                               max(1.2, frame_min + self.door_depth_margin))

        # ±75° arama: robot kapı 2'nin yüzüne vardığında açıklık
        # −57…−67° bandında durur (şikan kıvrımı) — ±50°'lik pencere onu
        # hiç görmedi ve kapı-modu, parkurun en zor darlığında tam da
        # devre dışı kaldı (koşular-arası analiz: o bantta 41 s
        # RECOVER/TIGHT birikti).
        light_center, light_width, light_slit_d = self._door_light(
            extended, n, 75.0, front_min, depth_thr=eff_depth_thr,
            behind_margin=0.7)
        if (light_center is not None and frame_min < 1.2
                and 0.05 <= light_width <= self.door_release_gap_width
                and abs(_angle_diff(self.est_heading + math.radians(light_center),
                                    self.desired_yaw)) <= math.radians(60.0)):
            self._light_mem_abs = self.est_heading + math.radians(light_center)
            self._light_mem_t = now
        # "Tek çıkış" koşulu: kapı-modu, ışığın geniş görüş alanındaki
        # TEK derin çıkış olduğu kapılar içindir. Saha ortasında iki
        # engel arasındaki sıkışıklık da dar ışık gösterir ama başka
        # derin yönler boldur — onları DWA salınımı, 0.12 m/s'lik
        # hizalama emeklemesinden çok daha iyi sürer (deneme 3: erken
        # sahada 22.6 s DOOR_ALIGN, koşu 2.7 m'de öldü).
        # ±135° yay: robotun GELDİĞİ arka-yan açıklığı da kapsayacak
        # kadar geniş. Saha ortasında o alan derin okur (tetik yok);
        # gerçek bir kapıda yarık dışında tüm manzara duvardır. Deneme
        # 4-9'un ±90° yayı arka çeyrekleri kaçırıyor ve bu ikisini
        # birbirinden ayıramıyordu.
        wide_half = int(135 * n / 360)
        deep_wide = sum(1 for o in range(-wide_half, wide_half + 1)
                        if extended[o % n] > self.door_through_depth)
        deep_frac = deep_wide / float(2 * wide_half + 1)
        # Sadece kurs-aşağı: yana ya da geriye götüren ışık, hizalanmaya
        # değer bir kapı değildir (deneme 6: +/-36-42°'lik yan
        # sıkışıklıklardan beş commit, robot yanal ping-pong yaptı, net
        # 3.5 m). Pusula gürültüsü (ort. ~40°) bu kesim için kabul
        # edilebilir (tarihte +/-50° idi; koddaki eşik şimdi 70°).
        light_down_course = (light_center is not None
                             and abs(_angle_diff(math.radians(light_center), goal_dir_rad))
                             < math.radians(70.0))
        # Yüz-kayma (face-slide) eğilimi (burada, ışık/kasa ölçüldükten
        # SONRA uygulanır — deneme 98 onlara hesaplanmadan referans
        # verdi ve callback çöktü): yüze yaslanmışken ışık görünür ama
        # merkez dışıysa, direksiyon hedefini ışığın tarafına doğru it.
        if (frame_min < 0.9 and light_center is not None
                and 25.0 < abs(light_center) <= 75.0):
            target_bearing_rad += math.radians(14.0 if light_center > 0 else -14.0)

        # Kapı-modu yalnız kabaca ÖNDEKİ ışıklar için (|yön| <= 40°):
        # merkezden uzak yüz ışıklarına önce kayma (slide) uygulanır —
        # iki başarılı kapı-3 geçişi de kapı-modu susarken düz kayma
        # modunda gerçekleşti; o titrek merkez-dışı ışıklarda devreye
        # girdiğinde kaymayı mahkûm hizalamalarla bölüyordu.
        narrow_ahead = (light_down_course
                        and abs(light_center) <= 40.0
                        and frame_min < 1.3
                        and self.door_min_light_width <= light_width < self.door_trigger_gap_width
                        and deep_frac < self.door_only_exit_frac)
        scene_open   = (frame_min > 2.0
                        or light_width > self.door_release_gap_width)
        self._door_narrow_streak = self._door_narrow_streak + 1 if narrow_ahead else 0
        self._door_open_streak   = self._door_open_streak + 1 if scene_open else 0

        # SWEEP tetiği: duvar var, önde geçilebilir IŞIK yok, sıkışık
        # (DWA) rejim — koşuların yüzlerde öldüğü durumun ta kendisi.
        # Streak ile süzülür.
        # SWEEP son çaredir: yalnız KAPI sistemi ışık görmüyorken VE FGM
        # gezgininin elinde de doğru düzgün bir şey yokken. Saha
        # sıkışıklıkları duvar+ışıksızlık testini geçer ama FGM'de hâlâ
        # iyi bir boşluk vardır — onlar normal salınımın işidir (deneme
        # 119: 17 s boşa saha süpürmesi).
        fgm_stuck = (gap_info["fallback"]
                     or gap_info["chosen_width_m"] < 0.6
                     or abs(_angle_diff(math.radians(gap_info["chosen_center_deg"]),
                                        goal_dir_rad)) > math.radians(60.0))
        no_door_here = (frame_min < 0.95 and front_min < 1.0
                        and fgm_stuck
                        and (light_center is None
                             or light_width < self.door_min_light_width
                             or abs(light_center) > 75.0))
        self._no_door_streak = self._no_door_streak + 1 if no_door_here else 0
        # Sweep salt yüzde-bekleme ile de devreye girebilir: titrek,
        # sınırda bir ışık no_door_here streak'ini sonsuza dek sıfırlar
        # (deneme 131: son duvarda 10 s AVOID pinball, DOOR_SWEEP=0.0 s)
        # — 6 s bekleme sonrası titremeye bakılmaksızın sweep serbest.
        # (face_dwell yukarıda, derinlik eşiği kısmında hesaplanıyor.)
        if (self.sweep_mode and self.door_mode is None
                and self._thread_start is None
                and now >= self.door_cooldown_until
                and (self._no_door_streak >= 10
                     or (face_dwell > 6.0 and frame_min < 0.95))
                and self.door_cleared_count >= self.sweep_min_doors
                and max(left_clear, right_clear) >= 0.7
                and (not self.hybrid_mode or not self._hybrid_fgm_active)):
            self.door_mode = "SWEEP"
            self._sweep_start = now
            self._sweep_flips = 0
            self._sweep_stall_streak = 0
            self._sweep_last_flip = now
            # Yön seçimi: yürüme alanına değil, kapı KANITINA doğru yürü.
            # 35-95° bandında yayılan derinlik, o tarafta bir açıklık
            # iması taşır; "ferah taraf" sezgiseli deneme 132'de 12 s
            # boyunca kapıdan uzağa yürüdü. Ferah taraf yalnız
            # beraberliği bozar; tıkalı taraf (açıklık < 0.7) asla
            # seçilmez.
            def _side_depth(sign):
                lo, hi = int(35 * n / 360), int(95 * n / 360)
                return max(extended[int(sign * o) % n] for o in range(lo, hi + 1))
            def _side_open_frac(sign):
                # Açıklık yalnız GENİŞ açık dönüşleri sayar; duvar
                # bandındaki (0.9-1.8) ışınlar aranabilir duvarı
                # işaretler ve derin yarık-içi sivrilikler artık cep
                # taklidi yapamaz (sınıflayıcı duvar-1'de tersine
                # dönmüştü: kapı tarafı, kapının kendi geçiş derinliği
                # YÜZÜNDEN "açık" okunuyordu).
                lo, hi = int(35 * n / 360), int(95 * n / 360)
                rays = [extended[int(sign * o) % n] for o in range(lo, hi + 1)]
                wall = sum(1 for r in rays if 0.9 <= r <= 1.8)
                openr = sum(1 for r in rays if r > 1.8)
                if wall >= 0.25 * len(rays):
                    return 0.0  # aranacak kadar duvar var: asla "cep" deme
                return openr / float(len(rays))
            l_depth, r_depth = _side_depth(1), _side_depth(-1)
            l_open, r_open = _side_open_frac(1), _side_open_frac(-1)
            # "Kapılar duvarlarda yaşar": cep ÇOK sayıda derin ışın
            # saçar; kapı ise orta menzilli duvardaki dar bir
            # sivriliktir. Bir taraf çoğunlukla açık (cep), öbürü
            # çoğunlukla duvarsa DUVAR boyunca süpür — 20+ duvar-1
            # finalinde cep tarafında bir kez bile kapı çıkmadı
            # (deneme 164: köşe üç kez seçildi).
            if l_open > 0.5 and r_open <= 0.3:
                self._sweep_dir = -1.0
            elif r_open > 0.5 and l_open <= 0.3:
                self._sweep_dir = 1.0
            elif abs(l_depth - r_depth) > 0.4:
                self._sweep_dir = 1.0 if l_depth > r_depth else -1.0
            else:
                self._sweep_dir = 1.0 if left_clear >= right_clear else -1.0
            # Işık hafızası sezgiselleri ezer: bu yüzde son 10 s içinde
            # kapı-kalitesinde ışık görüldüyse onun olduğu yöne DOĞRU
            # yürü — duvar-1 sağ-köşe ölümlerinin hepsinde kapı,
            # sürüklenme başlamadan önce görüş alanındaydı (deneme 230
            # teşhisi).
            if (self._light_mem_t is not None
                    and now - self._light_mem_t < 10.0):
                mem_rel = _angle_diff(self._light_mem_abs, self.est_heading)
                if abs(mem_rel) > math.radians(10.0):
                    self._sweep_dir = 1.0 if mem_rel > 0 else -1.0
            # Kapsama hafızası: aynı yüzde yeniden devreye girme ÖBÜR
            # yöne gider — deneme 132/133 ikisi de sola süpürdü, zaman
            # aşımına uğradı ve YİNE sola süpürdü. "Sol işe yaramadı,
            # sağa git" için harita gerekmez.
            if (self._last_sweep_exit is not None
                    and now - self._last_sweep_exit < 20.0):
                # 20 s (eskiden 6): bir yüzde meyvesiz kalan yön meyvesiz
                # kalır — aynı yüz hep aynı şekilde yeniden aranıyordu
                # (deneme 207 ve ~12 sağ-girinti seferi, o tarafta sıfır
                # geçiş).
                self._sweep_dir = -self._last_sweep_dir
            # Son emniyet: seçilen taraf fiziken tıkalıysa tersine çevir.
            if self._sweep_dir > 0 and left_clear < 0.7:
                self._sweep_dir = -1.0
            elif self._sweep_dir < 0 and right_clear < 0.7:
                self._sweep_dir = 1.0
            self.get_logger().info(f"[FgmDwaRobot] SWEEP engaged — no door visible, walking the wall "
                          f"({'left' if self._sweep_dir > 0 else 'right'}).")
            self._log(f"TRANSITION door_mode None -> SWEEP | dir={'L' if self._sweep_dir > 0 else 'R'} "
                      f"frame_min={frame_min:.2f}")

        # ALIGN tetiği: dar-ışık streak'i doldu, kapı-modu boş, cooldown
        # geçti ve burun kurstan <40° sapmış -> kapıya hizalanmaya başla.
        if (self.door_mode is None and self._thread_start is None
                and now >= self.door_cooldown_until
                and self._door_narrow_streak >= self.door_trigger_streak
                and abs(_angle_diff(self.est_heading, self.desired_yaw))
                < math.radians(40.0)):
            self.door_mode = "ALIGN"
            self.door_align_start_time = now
            # Taze süzme durumu (inceleme düzeltmesi): önceki ALIGN'dan
            # kalan bayat streak'ler, tek bir merkez-dışı titremenin
            # anında commit etmesine izin veriyordu (gate_light, artakalan
            # sayaçlarla geçer olmuştu).
            self._light_ok_streak = 0
            self._frame_ok_streak = 0
            self._last_light_sign = 1.0
            self.smoothed_align_light_deg = float(light_center)
            self.get_logger().info(f"[FgmDwaRobot] Door-mode ALIGN engaged "
                          f"(light at {light_center:.0f}°, width {light_width:.2f}m, "
                          f"front_min={front_min:.2f}).")
            self._log(f"TRANSITION door_mode None -> ALIGN | light_center={light_center:.1f} "
                      f"light_width={light_width:.2f} front_min={front_min:.2f}")
        # ALIGN bırakma: sahne açıldıysa (geniş ışık / uzak yüz) burada
        # kapı yok demektir — modu bırak, kısa bir cooldown ver.
        elif self.door_mode == "ALIGN" and self._door_open_streak >= self.door_release_streak:
            self.door_mode = None
            self.door_align_start_time = None
            self.door_cooldown_until = now + self.door_retrigger_cooldown
            self.get_logger().info(f"[FgmDwaRobot] Door-mode released — scene opened "
                          f"(light_width={light_width:.2f}m, front_min={front_min:.2f}).")
            self._log(f"TRANSITION door_mode ALIGN -> None | RELEASED "
                      f"light_width={light_width:.2f} front_min={front_min:.2f}")

        # Ağız-kapma (mouth-grab) kestirmesi: yüzde dururken burnun
        # dibinde güçlü, kurs-aşağı bir ışık varsa tam tetik→ALIGN→
        # kapılar boru hattını beklemek lükstür — o hat, sürekli
        # dar-boşluk streak'i artı üç kapı streak'i ister; deneme 135
        # t=45'te kapı ağzında durdu ve kalan kaotik 10 s'de hat hiç
        # devreye girmedi. Işık zaten görünür, yeterince geniş, burna
        # yakın ve kurs-aşağı götürüyorsa HEMEN commit et (2 taramalık
        # süzme); yanlış bir kapmayı DEPTH_LOST yine iptal eder.
        if (self.door_mode is None and self._thread_start is None
                and now >= self.door_cooldown_until
                and frame_min >= 0.35):
            mouth_c, mouth_w, mouth_slit_d = self._door_light(
                extended, n, DOOR_COMMIT_GAP_HALF_DEG, front_min,
                depth_thr=strict_depth_thr, behind_margin=0.7)
            mouth_ok = (mouth_c is not None
                        # "Ağızdayım" ölçüsü artık YARIĞIN mesafesi,
                        # frame_min değil — deneme 313'ün sahte kapması,
                        # girinti yarığı 1.8 m'deyken duvara 1.4 m kala
                        # saha dağınıklığı üzerinden eski [0.35,0.85)
                        # kasa penceresini geçmişti. Üst sınır da artık,
                        # yarık 0.9 m ötedeyken açık ağız alanında duran
                        # robotun kapmasını engellemiyor.
                        and mouth_slit_d is not None
                        and 0.30 <= mouth_slit_d < 0.95
                        # ±28° (kullanıcı: "daha çok commit et"):
                        # ±20/0.70'i zorlayan eğik-kapma ölümleri,
                        # cos²'lü commit hız planlamasından ÖNCEYDİ;
                        # artık açılı girişler kendiliğinden yavaşlıyor.
                        and abs(mouth_c) <= 28.0
                        and mouth_w >= self.door_min_light_width
                        # Kapı-tetik genişliğinden geniş ışık yarık
                        # değildir — onları normal seyrüsefer daha iyi
                        # sürer (deneme 137: 1.02 m'lik kapma 6.6 s
                        # yakıp TIMEOUT oldu).
                        and mouth_w <= self.door_trigger_gap_width
                        and abs(_angle_diff(
                            self.est_heading + math.radians(mouth_c),
                            self.desired_yaw)) <= math.radians(45.0))
            self._mouth_streak = self._mouth_streak + 1 if mouth_ok else 0
            # 2 tarama (eskiden 3): kapma, FGM'in kapıdan uzağa
            # boşluk-değiştirmesiyle yarışır — deneme 158 yarışı tek
            # taramayla kaybetti ve bedeli bütün final oldu.
            if self._mouth_streak >= 2:
                self._mouth_streak = 0
                self.door_mode = "COMMIT"
                self.door_commit_start_time = now
                self.door_commit_count += 1
                self._commit_no_depth_streak = 0
                self._commit_wide_streak = 0
                self._commit_entry_front = frame_min
                self._commit_side_graze = False
                self._commit_scan_count = 0
                self.smoothed_commit_bearing_deg = mouth_c
                self.get_logger().info(f"[FgmDwaRobot] Mouth-grab — committing directly "
                              f"(light {mouth_c:+.0f}°, {mouth_w:.2f}m).")
                self._log(f"TRANSITION door_mode None -> COMMIT | MOUTH_GRAB "
                          f"light={mouth_c:+.1f}deg width={mouth_w:.2f} "
                          f"slit_d={mouth_slit_d:.2f} frame_min={frame_min:.2f}")
        else:
            self._mouth_streak = 0

        # Proaktif THREAD: kanal-girişi imzası — burun neredeyse değiyor,
        # yüz yakın, derin bir ışın sıradaki bacağı işaretliyor —
        # emekleme, sıkışma emniyeti bizi fırlatamadan ÖNCE devreye
        # girer. Deneme 142 gösterdi ki giriş, şans eseri doğru noktada
        # sıkışmaya bağlanamaz. YALNIZ kullanılabilir ışık yokken:
        # THREAD ışıksız kıvrımlı kanallar içindir; ışıklı ağızda kapma
        # (0.55 m/s, kanıtlı) kazanmalı — deneme 144'ün thread'i
        # duvar-1'de kapmanın 3-taramalık streak'ini geçti ve finali
        # mahvetti.
        no_light_here = (light_center is None
                         or light_width < self.door_min_light_width
                         or abs(light_center) > 80.0)
        # Köşe-kapısı kilitlenme düzeltmesi (kullanıcı ekran görüntüsü,
        # deneme 215): koridor yan duvarına sıfır bir kapıda ışık
        # GÖRÜNÜR (thread kapmaya boyun eğer) ama kapmanın 0.35'lik
        # mesafe tabanı köşe sıkışmasından erişilmezdir — kimse harekete
        # geçmez. Kapma imkânsızsa (kasa < 0.35) THREAD ışığa rağmen
        # davranabilir; 4 s bekleme sonrası yuva şartı tek-taraflı
        # yuvaları da kabul eder (köşe kapılarının bir yanı duvar,
        # arkası koridordur).
        grab_impossible = frame_min < 0.35
        in_slot = (min(left_clear, right_clear) < 0.45
                   and max(left_clear, right_clear) < 1.2)
        in_slot_eff = (min(left_clear, right_clear) < 0.5
                       if self._face_dwell_now > 4.0 else in_slot)
        if (self.door_mode is None and self._mouth_streak == 0
                and (no_light_here or grab_impossible) and in_slot_eff
                and front_min < 0.45 and frame_min < 0.6):
            if self._try_thread(extended, n, now, elapsed, front_min,
                                min_r360, cb_t0):
                return

        # ── BÖLÜM: SWEEP sürüşü — duvar boyunca kapı arama yürüyüşü ─────
        if self.door_mode == "SWEEP":
            sweep_expired = now - self._sweep_start > self.sweep_timeout
            # Önde kapı bulundu -> ALIGN'a devret (kanıtlı dönüştürücü).
            # Işık DÜNYA çerçevesinde kurs-aşağı götürmeli; sırf burnun
            # yanında durması yetmez: ±75° eksen-dışı süpürürken robot
            # çerçevesindeki +46° ışık mutlakta ~+120° eder — yani
            # arkadaki giriş kapısıdır (deneme 130: giriş kapısından
            # GERİYE commit etti, 2.5 m hediye etti). Pencere mutlak
            # ±55°, tahmin hatasına cömert.
            light_abs_ok = (light_center is not None
                            and abs(_angle_diff(
                                self.est_heading + math.radians(light_center),
                                self.desired_yaw)) <= math.radians(55.0))
            # Robot çerçevesi penceresi ±80°: ±75° eksen-dışı yürürken
            # kapı robot çerçevesinde ∓75°'te kalır — eski ±50° pencere,
            # sweep'in kapıları ancak dönüş ortasında görebilmesi
            # demekti (deneme 133). Yön disiplinini yukarıdaki mutlak
            # veto sağlar.
            door_found = (light_center is not None
                          and abs(light_center) <= 80.0
                          and light_abs_ok
                          # ararken kıymıklar da sayılır: ALIGN hattı
                          # commit gücünde zaten yeniden eler
                          and light_width >= 0.12)
            face_gone = frame_min > 1.4
            if door_found or face_gone or sweep_expired:
                if door_found:
                    self.door_mode = "ALIGN"
                    self.door_align_start_time = now
                    self._light_ok_streak = 0
                    self._frame_ok_streak = 0
                    self._last_light_sign = 1.0
                    self.smoothed_align_light_deg = float(light_center)
                    self.get_logger().info(f"[FgmDwaRobot] SWEEP found a door "
                                  f"(light {light_center:+.0f}°, {light_width:.2f}m) — aligning.")
                    self._log(f"TRANSITION door_mode SWEEP -> ALIGN | light={light_center:.1f} "
                              f"width={light_width:.2f}")
                else:
                    self.door_mode = None
                    self.door_cooldown_until = now + (1.5 if sweep_expired else 1.5)
                    reason = "SWEEP_TIMEOUT" if sweep_expired else "FACE_GONE"
                    if sweep_expired:  # meyvesiz: kapsama hafızasına yaz
                        self._last_sweep_dir = self._sweep_dir
                        self._last_sweep_exit = now
                    else:
                        self._last_sweep_exit = None
                    self._log(f"TRANSITION door_mode SWEEP -> None | {reason}")
                # bu taramada normal akışa düş (aşağıdaki kod devam eder)
            else:
                # Süpürme yönünde koridor kenarı mı? Yön değiştir
                # (zigzag). Flip hijyeni (deneme 128/129: 2.6-4.3 s'de 3
                # flip yandı): taze bir flip, kenarlar yeniden flip
                # tetikleyemeden önce 1.2 s yol toleransı alır — yön
                # değiştirir değiştirmez, az önce terk edilen kenar hâlâ
                # yakın okur ve anında yeniden tetiklerdi. Eşik 0.4:
                # 2-3 m'lik odada ±75°'te 0.5, gerçek yürüme alanı hâlâ
                # varken ateşliyordu. İKİ yön de tıkalıysa bu gerçek bir
                # cep — ping-pong yapmak yerine çık.
                edge_off = int(round(self._sweep_dir * self.sweep_heading_deg * n / 360.0))
                e_half = max(1, int(15 * n / 360))
                edge_clear = min(clean[(edge_off + o) % n] for o in range(-e_half, e_half + 1))
                flip_grace = (now - self._sweep_last_flip < 1.2)
                # Lazerle takılma tespiti: hareket komutlanıyor ama dünya
                # yanımızdan akmıyor (deneme 136: tek noktada 10 s
                # çakılı, kapı 3'e 0.4 m kala duvara emekleme) -> bunu
                # kenar say (flip tetikle).
                if (scan_change is not None and scan_change < 0.03
                        and abs(self.cmd_linear_x) >= 0.10):
                    self._sweep_stall_streak += 1
                else:
                    self._sweep_stall_streak = 0
                sweep_stalled = self._sweep_stall_streak >= 12
                if sweep_stalled:
                    self._sweep_stall_streak = 0
                    self._log(f"SWEEP stall-flip | scan_change={scan_change:.3f}")
                # Flip eşiği = emekleme eşiği (0.45): eski 0.40/0.45
                # çifti, hiç flip'lemeden sonsuza dek emekleme diye bir
                # ölü bant bırakıyordu.
                if (edge_clear < 0.45 and not flip_grace) or sweep_stalled:
                    opp_off = int(round(-self._sweep_dir * self.sweep_heading_deg * n / 360.0))
                    opp_clear = min(clean[(opp_off + o) % n] for o in range(-e_half, e_half + 1))
                    self._sweep_flips += 1
                    if self._sweep_flips > self.sweep_max_flips or opp_clear < 0.4:
                        self.door_mode = None
                        self.door_cooldown_until = now + 1.5
                        reason = ("POCKET" if opp_clear < 0.4 else "FLIPS_EXHAUSTED")
                        self._last_sweep_dir = self._sweep_dir
                        self._last_sweep_exit = now
                        self._log(f"TRANSITION door_mode SWEEP -> None | {reason}")
                    else:
                        self._sweep_dir = -self._sweep_dir
                        self._sweep_last_flip = now
                        self.get_logger().info(f"[FgmDwaRobot] SWEEP edge — reversing "
                                      f"({'left' if self._sweep_dir > 0 else 'right'}).")
                        self._log(f"SWEEP flip #{self._sweep_flips} | edge_clear={edge_clear:.2f}")
                if self.door_mode == "SWEEP":
                    # 80/20 görev çevrimi: her periyodun sweep_main_frac
                    # kadarında ana tarafı tut, sonra kısaca öbür tarafa
                    # bak ki LiDAR görmediği yeri görsün — sürekli
                    # aşağı/yan flip-flop'u böylece bitti.
                    phase = ((now - self._sweep_start) % self.sweep_period) / self.sweep_period
                    cur_dir = self._sweep_dir if phase < self.sweep_main_frac else -self._sweep_dir
                    if cur_dir != self._sweep_dir:
                        # bakılacak taraf zaten duvarda mı? bakışı atla
                        p_off = int(round(cur_dir * self.sweep_heading_deg * n / 360.0))
                        p_clear = min(clean[(p_off + o) % n]
                                      for o in range(-e_half, e_half + 1))
                        if p_clear < 0.5:
                            cur_dir = self._sweep_dir
                    # Asla-geri-dönme garantisi: burun ±90° yan çizgisini
                    # aştıysa ÖN taraftan (kurs-aşağı) geri dön — en kısa
                    # yol direksiyonu oradan arkadan dolanırdı.
                    off_axis = _angle_diff(self.est_heading, self.desired_yaw)
                    if abs(off_axis) > math.radians(90.0):
                        w_cmd = -math.copysign(1.0, off_axis)
                        v_cmd = 0.12
                    else:
                        want_abs = self.desired_yaw + cur_dir * math.radians(self.sweep_heading_deg)
                        bearing_cmd = _angle_diff(want_abs, self.est_heading)
                        w_cmd = max(-1.0, min(1.0, 1.5 * bearing_cmd))
                        # Burun VEYA gidiş yönü darsa emekle — 75° eksen
                        # dışında yalnız burun şartı, sweep'in kendi
                        # gidiş kenarına sürmesine izin verdi (deneme
                        # 132).
                        travel_clear = min(front_min, edge_clear)
                        # 0.38/0.18 (eskiden 0.45/0.12): eğik yüz
                        # yürüyüşü travel_clear'ı neredeyse hep 0.38-0.45
                        # bandında tutar — deneme 156 efektif 0.11 m/s
                        # ile süpürdü ve kapı 3'ü düdüğe 0.1 m kala
                        # kaybetti.
                        # front 0.26 altı: SADECE dön — emekleyen sweep,
                        # gövdeyi 4. darlığın yüzüne mıhladı (deneme 195:
                        # son 5 s front 0.12'de kımıltısız, geri
                        # olmadan kurtarılamaz).
                        if front_min < 0.26:
                            v_cmd = 0.0
                        else:
                            v_cmd = self.sweep_speed if travel_clear > 0.38 else 0.18
                    v_cmd, w_cmd = self._progress_guard(v_cmd, w_cmd, now,
                                                        tag="SWEEP")
                    self.cmd_linear_x, self.cmd_angular_z = v_cmd, w_cmd
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = v_cmd, w_cmd
                    self.cmd_pub.publish(cmd)
                    if self.last_label != "DOOR_SWEEP":
                        self.get_logger().info(f"[FgmDwaRobot] {self.last_label} → DOOR_SWEEP (wall search).")
                        self.last_label = "DOOR_SWEEP"
                    self._emit_telemetry(now, elapsed, "DOOR_SWEEP", front_min,
                                         bearing_raw=light_center,
                                         gap_depth=light_width,
                                         min_r360=min_r360, cb_t0=cb_t0)
                    return

        # ── BÖLÜM: COMMIT sürüşü — kapıdan fiilen geçme evresi ──────────
        # ALIGN burnu kapıya hizaladı; COMMIT ise gaza basıp aralıktan
        # gerçekten geçme evresidir. Her taramada üç soru sorulur:
        # 1) Geçit hâlâ görünüyor mu? (derinlik kaybı -> DEPTH_LOST)
        # 2) Geçtik mi? (başarı kanıtları -> CLEARED)
        # 3) İş yanlış mı gidiyor? (eksen dışı / süre aşımı -> iptal)
        if self.door_mode == "COMMIT":
            # "İçinden görme" derinliği yok olursa erken iptal: geçit
            # kaybolmuş demektir (yanlış nişan ya da önü kapanmış) ve
            # ısrar etmek, süre aşımına dek burnu duvara sürtmektir.
            # COMMIT evresi eşiği gevşek paydır (0.9): koy/geçit ayrımı
            # asıl giriş kapılamasında önemlidir; bir kez commit edince
            # S-şikan, geçidin uzak bacağını yarı yolda perdeler ve sıkı
            # pay meşru geçişleri DEPTH_LOST ile öldürür (deneme 139'un
            # son commit'i tam düdükte böyle öldü).
            commit_thr = min(self.door_through_depth,
                             max(1.2, frame_min + 0.9))
            # ±40° commit konisindeki en derin uzatılmış ışın = geçidin
            # "içinden görünen" derinliği. Eşiğin altına inersek körüz.
            c_half = int(40 * n / 360)
            commit_depth = max(extended[o % n] for o in range(-c_half, c_half + 1))
            if commit_depth < commit_thr:
                self._commit_no_depth_streak += 1
            else:
                self._commit_no_depth_streak = 0
            # İki kademeli "kör" iptal: ışık titrerken alan hâlâ varsa
            # sabret (12-16 taramalık tolerans), ama hem körsen HEM de
            # çerçeve gerçekten kapanan bir cepse hızlı kaç (6 tarama).
            # Eski hızlı-kaçış kapısı frame < 0.9 idi; ama gerçek bir
            # kapı yüzünde çerçeve TÜM geçiş boyunca zaten < 0.9 kalır —
            # yani hızlı iptal, commit'in en önemli olduğu yerde salt
            # titreme yüzünden ateşliyordu (kullanıcı: "it thinked that
            # it will not go through").
            blind_cap = 16 if frame_min >= 0.65 else 12
            depth_lost = (self._commit_no_depth_streak >= blind_cap
                          or (self._commit_no_depth_streak >= 6
                              and frame_min < 0.65))
            # Başarı, ışığın genişçe açılması olarak da görünebilir
            # (yarığı geçip ferah bölgeye çıkmak) — bu sırada bir yan
            # duvar frame_min'i hâlâ düşük tutabilir. Commit'i frame_min'e
            # rehin etme.
            exit_light_c, exit_light_w, _exit_slit_d = self._door_light(
                extended, n, DOOR_COMMIT_GAP_HALF_DEG, front_min,
                depth_thr=commit_thr)
            if exit_light_w > self.door_release_gap_width:
                self._commit_wide_streak += 1
            else:
                self._commit_wide_streak = 0
            passage_open = self._commit_wide_streak >= 3
            # Geçiş kanıtı (deneme 312): gövdenin yan sektörlerini örnekle
            # (iki yanda 55-125°, ham tarama) — gerçek bir geçişte kapı
            # direkleri oradan < 0.55 m mesafeden akar. Yalnız yakın-giriş
            # commit'leri (giriş çerçevesi < 0.9: yüz/ağız commit'leri;
            # deneme 311'in sahte CLEARED'ı burada olmuştu) bu "yandan
            # sıyırma" kanıtını hak etmek zorunda; uzak tarla-yarığı
            # commit'leri eski çıkışı korur. Gerçekten apaçık bir duruş
            # (çerçeve > 1.3 VE bırakma genişliğinde ışık) da sayılır —
            # o bir odadır, geçilmemiş yarık değil.
            s_lo, s_hi = int(55 * n / 360), int(125 * n / 360)
            side_min_now = 10.0
            for o in range(s_lo, s_hi + 1):
                for r in (clean[o % n], clean[(-o) % n]):
                    if 0.05 < r < side_min_now:
                        side_min_now = r
            # 3 taramalık ısınma (review düzeltmesi): commit GİRİŞİNDE
            # zaten < 0.55 olan bir yan duvar (koridor duvarına yapışık
            # başlarken) hiçbir şeyi geçtiğimizin kanıtı değildir; gerçek
            # direkler geçiş boyunca yan sektörlerde çok tarama kalır,
            # yani ilk 3'ü atlamak hiçbir şey kaybettirmez.
            self._commit_scan_count += 1
            if side_min_now < 0.55 and self._commit_scan_count > 3:
                self._commit_side_graze = True
            wide_open = (frame_min > 1.3
                         and exit_light_w > self.door_release_gap_width)
            # (Review düzeltmesi: eski `entry_front >= 0.9` kaçamağı
            # kaldırıldı — ALIGN commit'leri hep 0.85 altında girer
            # (gate_front), yani muaf tutabildiği tek şey yüksek-çerçeveli
            # ağız kapmalarıydı: yeniden açtığı yol tam da o sahte-pozitif
            # yoluydu.)
            traversal_evidence = (self._commit_side_graze
                                  or self._commit_entry_front is None
                                  or wide_open)
            elapsed_commit = (now - self.door_commit_start_time
                              if self.door_commit_start_time is not None else 0.0)
            # Gerçek bir ışık hâlâ görünüyor ama henüz geçilmemişse
            # commit'e iki kat süre tanı — yüzde erken bırakmak, robotu
            # blob-FGM'e ve sağdaki girintiye yem eden şeydi.
            committed_too_long = elapsed_commit > (
                self.door_commit_timeout
                * (2.0 if (not traversal_evidence and exit_light_c is not None)
                   else 1.0))
            # (Deneme 7'ye kadar burada çerçeve-tabanlı bir "ilerleme yok"
            # iptali vardı, kaldırıldı: uzun bir geçidin içinde çerçeve
            # mesafesi ZATEN sabittir — yan duvarlar — bu yüzden meşru
            # geçişleri 1.2 s'de iptal ediyordu. Asıl koruma: 2.5 s süre
            # aşımı + derinlik-kaybı iptali + takılma dürtmesi.)
            no_progress = False
            # Eksen-dışı iptal (deneme 311): gövde yönü parkur-aşağı
            # yönden abort_off'tan fazla dönmüşse o commit zaten
            # başarısızdır — kapı düzlemi parkura kabaca dik olduğundan
            # hiçbir meşru geçiş oraya bakmaz. 92° U-dönüş bekçisini /
            # kör-seri DEPTH_LOST'u beklemek, deneme 310'da duvar-1'de
            # 6.6 s yaktı.
            off_now = _angle_diff(self.est_heading, self.desired_yaw)
            off_axis = abs(off_now) > math.radians(self.door_commit_abort_off_deg)
            # Commit konisinde önde hâlâ dar, kapı-gibi bir ışık varken
            # CLEARED ilan etme: şaşırtmalı kapılarda şikan-ortası bir oda
            # vardır; frame_min orada eşiği kısacık aşar ve erken CLEARED,
            # blob-FGM'i çıkışını bulamadığı bir cebe terk eder (deneme
            # 29: 1.5 s sonra çerçeve 1.15'te "cleared" dedi, kıymıklarda
            # çıkmaza girdi, yüzün önünde ters yöne döndü).
            next_jog_ahead = (self.door_min_light_width <= exit_light_w
                              < self.door_trigger_gap_width)
            success_exit = (((frame_min > self.door_exit_front_dist
                              and not next_jog_ahead) or passage_open)
                            and traversal_evidence)
            # Herhangi bir çıkış nedeni oluştuysa COMMIT'i kapat; nedeni
            # sınıflandırıp logla (CLEARED / OFF_AXIS / DEPTH_LOST /
            # TIMEOUT / NO_TRAVERSAL). Sayaçlar ve bekleme süreleri
            # nedene göre farklıdır.
            if (success_exit or committed_too_long or depth_lost
                    or no_progress or off_axis):
                commit_duration = (now - self.door_commit_start_time
                                   if self.door_commit_start_time is not None else 0.0)
                self.door_commit_total_time += commit_duration
                self.door_mode = None
                self.door_commit_start_time = None
                self.door_stall_active = False
                self.commit_prev_center_deg = None
                self._commit_no_depth_streak = 0
                self._commit_wide_streak = 0
                self._commit_entry_front = None
                if off_axis:
                    # Kısa bekleme (2.5 s'lik sahte-ışık cezası değil):
                    # ışık gerçekti, icraat yoldan çıktı. ~78°'den nişan
                    # menziline geri dönmek bekçi altında ~1 s sürer;
                    # 1.5 s, burun döner dönmez AYNI ışığın yeniden
                    # tetiklenmesine izin verir. THREAD devri yok — onun
                    # ±50° burun konisi yan dönükken yanlış bölgeye bakar.
                    self.door_timeout_count += 1
                    self.door_cooldown_until = now + 1.5
                    self.get_logger().info("Door commit aborted (off-axis).")
                    self._log(f"TRANSITION door_mode COMMIT -> None | OFF_AXIS "
                              f"off={math.degrees(off_now):+.0f}deg frame_min={frame_min:.2f} "
                              f"commit_duration={commit_duration:.2f}s")
                elif (depth_lost or no_progress) and frame_min <= self.door_exit_front_dist:
                    self.door_timeout_count += 1
                    # Başarısız commit'lere uzun bekleme: 0.5 s'lik olanı
                    # aynı sahte ışığı anında yeniden tetikliyordu
                    # (deneme 106: 17 kör fallback'lik kısır döngü).
                    self.door_cooldown_until = now + 2.5
                    reason = "DEPTH_LOST" if depth_lost else "NO_PROGRESS"
                    self.get_logger().info(f"Door commit aborted ({reason.lower()}).")
                    self._log(f"TRANSITION door_mode COMMIT -> None | {reason} frame_min={frame_min:.2f} "
                              f"commit_duration={commit_duration:.2f}s")
                    # Kör-iptal devri: ışık öldü ama kanal, THREAD'in
                    # kovalayabileceği bir dirsekle sürüyor olabilir
                    # (deneme 166: iptal sonrası FGM köşeye kaydı; oysa
                    # iptal noktasından bir thread köşeyi dönebilirdi).
                    if self._try_thread(extended, n, now, elapsed, front_min,
                                        min_r360, cb_t0):
                        return
                elif committed_too_long and frame_min <= self.door_exit_front_dist:
                    # Emniyet ağı: robot gerçekten takıldıysa front_min
                    # sonsuza dek basılı kalabilir. Süre aşımı kontrolü
                    # FGM/DWA'ya geri verir — en azından LiDAR'ın şu an
                    # gördüğüne tepki verebilirler.
                    self.door_timeout_count += 1
                    self.door_cooldown_until = now + 2.5
                    self.get_logger().info("Door commit timed out, forcing resume.")
                    self._log(f"TRANSITION door_mode COMMIT -> None | TIMEOUT front_min={front_min:.2f} "
                              f"commit_duration={commit_duration:.2f}s")
                elif traversal_evidence:
                    self.door_cleared_count += 1
                    self._face_dwell_start = None
                    self.get_logger().info("Door cleared, resuming normal navigation.")
                    self._log(f"TRANSITION door_mode COMMIT -> None | CLEARED frame_min={frame_min:.2f} "
                              f"commit_duration={commit_duration:.2f}s")
                else:
                    # (Uzatılmış) süre, yüzün dibinde direkleri hiç
                    # sıyırmadan bitti: bu geçilmiş bir kapı DEĞİL —
                    # cleared sayacı bölge mantığını kapıladığından
                    # şişirilmemeli. Kısa bekleme: aynı gerçek ışık taze
                    # bir saatle yeniden yakalanabilsin.
                    self.door_timeout_count += 1
                    self.door_cooldown_until = now + 1.5
                    self.get_logger().info("Door commit released (no traversal evidence).")
                    self._log(f"TRANSITION door_mode COMMIT -> None | NO_TRAVERSAL "
                              f"frame_min={frame_min:.2f} "
                              f"commit_duration={commit_duration:.2f}s")
                # aşağı düş: aynı taramada normal FGM/DWA'ya devam et
            else:
                # Izgara-güdümlü commit (kullanıcı isteği, deneme 236):
                # kapıdan geçen BFS yolunu pure-pursuit tarzı izle — her
                # taramada yeniden planlanır; ışığın ötesini göremediği
                # dirseklerden S çizerek geçer. Yol var olduğu sürece
                # kör-iptal serisi sıfırlanır (topoloji, görüş hattını
                # yener); ızgara çekimser kalırsa aşağıdaki ışık yeniden-
                # nişanı / en-derin-ışın yedeği devreye girer.
                grid_used = False
                if self.use_grid_door and self.use_grid_commit:
                    gp = self._grid_plan(
                        clean, n,
                        prefer_rel_deg=self.smoothed_commit_bearing_deg)
                    if gp is not None:
                        gpath, gprog = gp
                        la = gpath[min(len(gpath) - 1, 9)]
                        gb_deg = math.degrees(math.atan2(la[1], la[0]))
                        # yol, COMMIT EDİLEN KAPIDAN geçmeli: ancak commit
                        # yönüyle 35° içinde anlaşıyorsa yön ver (deneme
                        # 236: başka bir sınıra giden yol, körlük koruması
                        # kapalıyken commit'i yana sürükledi — 30 s wedge)
                        if abs(gb_deg - self.smoothed_commit_bearing_deg) <= 35.0:
                            self.smoothed_commit_bearing_deg = (
                                0.5 * gb_deg
                                + 0.5 * self.smoothed_commit_bearing_deg)
                            self._commit_no_depth_streak = 0
                            self.grid_commit_scans += 1
                            grid_used = True
                # Her taramada kapıdan görünen ışığın merkezine yeniden
                # nişan al (±40° commit konisindeki en geniş derin yay) —
                # geçit yönünün doğrudan ölçümü; FGM'in blob sorunundan
                # etkilenmez. EMA (üstel kayan ortalama) yumuşatması, tek
                # taramalık gürültüyü hem kontrolcüden hem takılma-
                # dürtmesinin işaret kararından uzak tutar.
                commit_light_c, commit_light_w, _commit_slit_d = self._door_light(
                    extended, n, DOOR_COMMIT_GAP_HALF_DEG, front_min,
                    depth_thr=commit_thr)
                if grid_used:
                    pass  # bu taramada yönün sahibi ızgara yolu
                elif commit_light_c is None:
                    # Commit konisinde ışık yok: onun yerine EN DERİN
                    # ışına nişan al (THREAD usulü). Eski "düze doğru
                    # sön" davranışı burnu dirsek kıvrımına, kör iptale
                    # kadar sürüyordu (denemeler 166-168, duvar-1'de
                    # 5.5-5.7 s ölümler); en derin ışın, ışık eşiğinin
                    # altında bile kanalın sıradaki bacağını izler. EMA
                    # yine yumuşatır, ±40° kilit yine sınırlar; gerçekte
                    # bir şey yoksa derinlik-kaybı yine iptal eder.
                    c_h = int(40 * n / 360)
                    deep_o, deep_r = 0, 0.0
                    for o in range(-c_h, c_h + 1):
                        r = extended[o % n]
                        if r > deep_r:
                            deep_r, deep_o = r, o
                    if deep_r > max(0.6, front_min + 0.15):
                        self.smoothed_commit_bearing_deg = (
                            0.3 * float(deep_o)
                            + 0.7 * self.smoothed_commit_bearing_deg)
                    else:
                        self.smoothed_commit_bearing_deg *= 0.85
                    self.commit_fallback_count += 1
                    if self.last_fallback_log_t is None or now - self.last_fallback_log_t >= 1.0:
                        self.last_fallback_log_t = now
                        self._log(f"EVENT COMMIT_FALLBACK | no light in commit cone "
                                  f"front_min={front_min:.2f}")
                        self._dump_scan(elapsed, "COMMIT_FALLBACK", extended)
                elif commit_light_c is not None:
                    # Burun dibinde direk sakınması (deneme 313): son
                    # ~0.7 m'de ışık yayının MERKEZİ yakın kapı direğine
                    # kayar (direk yayın bir yanını perdeler, EMA da bu
                    # kaymayı geriden izler) — denemeler 310/312 onu
                    # kovalarken ikisi de ön 0.15-0.17'de direğe sürttü.
                    # En derin uzatılmış ışın geçiş yönünün TA KENDİSİdir
                    # (THREAD'in öncülü; kanal ortasında kanıtlı). Çerçeve
                    # kapandıkça ona doğru karıştır: ağırlık çerçeve
                    # 0.7'de 0 → temasta 1.
                    aim_deg = float(commit_light_c)
                    if frame_min < 0.7:
                        c_h = int(40 * n / 360)
                        deep_o, deep_r = 0, 0.0
                        for o in range(-c_h, c_h + 1):
                            r = extended[o % n]
                            if r > deep_r:
                                deep_r, deep_o = r, o
                        if deep_r > max(1.0, frame_min + 0.5):
                            blend = (0.7 - frame_min) / 0.7
                            aim_deg = ((1.0 - blend) * aim_deg
                                       + blend * float(deep_o))
                    self.smoothed_commit_bearing_deg = (
                        self.bearing_smoothing_alpha * aim_deg
                        + (1.0 - self.bearing_smoothing_alpha) * self.smoothed_commit_bearing_deg
                    )
                max_lock_rad = math.radians(self.door_max_lock_bearing_deg)
                commit_bearing_rad = max(-max_lock_rad,
                                          min(max_lock_rad, math.radians(self.smoothed_commit_bearing_deg)))
                # Dünya-çerçevesi kelepçesi (deneme 311): yukarıdaki
                # kilit robot çerçevesindedir — gövde döndükten sonra,
                # robot çerçevesinde "yasal" görünen bir hedef yine de
                # parkur-yukarı bakabilir. est_heading + hedef, parkur-
                # aşağı yönden en çok max_world_off uzakta kalacak şekilde
                # kelepçele (FGM kolundaki dairesel U-dönüş kelepçesinin
                # eşi, ama daha sıkı: bir commit'in kapı düzleminin
                # ötesine bakması asla gerekmez).
                max_world = math.radians(self.door_commit_max_world_off_deg)
                world_tgt = off_now + commit_bearing_rad
                if abs(world_tgt) > max_world:
                    commit_bearing_rad = max(
                        -max_lock_rad,
                        min(max_lock_rad,
                            math.copysign(max_world, world_tgt) - off_now))
                self._publish_door_commit(frame_min, commit_bearing_rad)
                self._update_anomaly_trackers(
                    now, elapsed,
                    "DOOR_STALL" if self.door_stall_active else "DOOR_COMMIT",
                    front_min, extended)
                self._emit_telemetry(
                    now, elapsed,
                    "DOOR_STALL" if self.door_stall_active else "DOOR_COMMIT",
                    front_min, bearing_raw=commit_light_c,
                    bearing_smooth=self.smoothed_commit_bearing_deg,
                    gap_depth=commit_depth, min_r360=min_r360, cb_t0=cb_t0)
                return

        # ── BÖLÜM: ALIGN sürüşü — commit öncesi kapıya hizalanma ────────
        # ALIGN, kapı ışığı burnun tam önüne gelene dek düşük hızda yay
        # çizer. Üç "kapı" (gate) birden yeşil olunca COMMIT'e geçilir:
        # çerçeve yakın mı, ışık ortada ve yeterince geniş mi, dönüş hızı
        # sakin mi.
        if self.door_mode == "ALIGN":
            # 1 Hz kapı raporu: üç commit koşulundan hangisi engelliyor?
            # Bu olmadan ALIGN'da geçen süre loglardan açıklanamıyordu
            # (ayar koşularında 7 s sessiz ALIGN gözlendi).
            # Çerçeve kapısında 2 taramalık seri şartı: tek taramalık bir
            # frame_min zıplaması eşiğin altına/üstüne inip çıkarak sahte
            # anında-CLEARED commit'ler doğuruyordu.
            self._frame_ok_streak = (self._frame_ok_streak + 1
                                     if frame_min < self.door_commit_front_dist else 0)
            gate_front = self._frame_ok_streak >= 2
            # Işık yönünü EMA ile yumuşat (0.35 yeni / 0.65 eski): tek
            # taramalık zıplamalar hizalama yayını sarsmasın.
            if light_center is not None:
                self.smoothed_align_light_deg = (
                    0.35 * light_center + 0.65 * self.smoothed_align_light_deg)
            light_ok = (light_center is not None
                        and abs(self.smoothed_align_light_deg) < self.door_commit_max_bearing_deg
                        and light_width >= self.door_min_light_width
                        # dünya-çerçevesi vetosu: parkur-yukarı götüren
                        # bir ışığa asla commit etme (arkada-kalan-kapı
                        # vakası, deneme 130)
                        and abs(_angle_diff(
                            self.est_heading
                            + math.radians(self.smoothed_align_light_deg),
                            self.desired_yaw)) <= math.radians(60.0))
            # Işık kapısı 3 taramalık seri ister: tek karelik parıltıya
            # commit atlamamak için.
            self._light_ok_streak = self._light_ok_streak + 1 if light_ok else 0
            gate_light = self._light_ok_streak >= 3
            gate_w     = abs(self.cmd_angular_z) < self.door_commit_max_wz
            if self.last_align_gate_log_t is None or now - self.last_align_gate_log_t >= 1.0:
                self.last_align_gate_log_t = now
                light_txt = ("none" if light_center is None
                             else f"{light_center:+.1f}deg/{light_width:.2f}m")
                self._log(
                    f"ALIGN gate | frame_min={frame_min:.2f}<{self.door_commit_front_dist} [{'ok' if gate_front else 'BLOCK'}] "
                    f"light={light_txt}<{self.door_commit_max_bearing_deg}deg [{'ok' if gate_light else 'BLOCK'}] "
                    f"|w|={abs(self.cmd_angular_z):.2f}<{self.door_commit_max_wz} [{'ok' if gate_w else 'BLOCK'}]"
                )
            # Yalnızca kapıdan görünen ışık görünür VE neredeyse tam
            # öndeyken commit et — geçit yönünün doğrudan ölçümü budur.
            if gate_front and gate_light and gate_w:
                align_duration = (now - self.door_align_start_time
                                  if self.door_align_start_time is not None else 0.0)
                self.door_mode = "COMMIT"
                self.door_commit_start_time = now
                self.door_commit_count += 1
                self._commit_no_depth_streak = 0
                self._commit_wide_streak = 0
                self._commit_entry_front = frame_min
                self._commit_side_graze = False
                self._commit_scan_count = 0
                self.smoothed_commit_bearing_deg = light_center
                self.get_logger().info("Committing to door.")
                self._log(f"TRANSITION door_mode ALIGN -> COMMIT | front_min={front_min:.2f} "
                          f"align_duration={align_duration:.2f}s "
                          f"bearing={self.smoothed_commit_bearing_deg:.1f}deg")
                max_lock_rad = math.radians(self.door_max_lock_bearing_deg)
                clamped_bearing = max(-max_lock_rad,
                                      min(max_lock_rad, math.radians(light_center)))
                self._publish_door_commit(frame_min, clamped_bearing)
                self._emit_telemetry(now, elapsed, "DOOR_COMMIT", front_min,
                                     bearing_raw=bearing_deg,
                                     bearing_smooth=math.degrees(clamped_bearing),
                                     gap_depth=gap_depth, min_r360=min_r360,
                                     gap_info=gap_info, cb_t0=cb_t0)
                return

            # Aktif hizalama: blob-FGM/DWA'ya düşmek yerine düşük hızda
            # ışık merkezine doğru yay çiz (deneme 2'de blob-FGM/DWA,
            # yüzün önünde hiç hizalanamadan 24.5 s çırpınmıştı).
            align_expired = (self.door_align_start_time is not None
                             and now - self.door_align_start_time > self.door_align_timeout)
            # Hizalama yayı robotu asla U-dönüşüne doğru çevirmesin:
            # ALIGN, DWA'yı ve onun sert duvarını atladığı için kendi
            # eksen-dışı sınırını kendisi uygular.
            align_off_axis = (abs(_angle_diff(self.est_heading, self.desired_yaw))
                              > math.radians(60.0))
            if align_expired or align_off_axis:
                self.door_mode = None
                self.door_align_start_time = None
                self.door_cooldown_until = now + self.door_retrigger_cooldown
                reason = "ALIGN_TIMEOUT" if align_expired else "ALIGN_OFF_AXIS"
                self.get_logger().info(f"[FgmDwaRobot] ALIGN released ({reason.lower()}).")
                self._log(f"TRANSITION door_mode ALIGN -> None | {reason} "
                          f"front_min={front_min:.2f}")
                # bu taramada normal FGM/DWA'ya düş
            else:
                # Işık görünüyorsa: yumuşatılmış ışık açısına orantılı
                # (P) dönüş komutu; ±door_align_max_w'e kırpılır.
                if light_center is not None:
                    self._last_light_sign = 1.0 if light_center >= 0.0 else -1.0
                    w_cmd = max(-self.door_align_max_w,
                                min(self.door_align_max_w,
                                    math.radians(self.smoothed_align_light_deg)
                                    * self.door_align_gain))
                else:
                    # ışık koniden çıktı: son görüldüğü yana yay çizmeye
                    # devam et
                    w_cmd = self._last_light_sign * 0.5
                # Ön 0.4 m'den yakınsa emekleme hızına (0.05) düş.
                v_cmd = self.door_align_speed if front_min > 0.4 else 0.05
                # Review düzeltmesi: ALIGN, ilerleme bekçisini atlayan
                # tek yayın noktasıydı ("HER giden komutta uygulanır"
                # ilkesine aykırı) — pinball sönümleyici de ALIGN→NAV
                # sonrası bayat işaret durumuyla devam ediyordu.
                v_cmd, w_cmd = self._progress_guard(v_cmd, w_cmd, now,
                                                    tag="ALIGN")
                self.cmd_linear_x, self.cmd_angular_z = v_cmd, w_cmd
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = v_cmd, w_cmd
                self.cmd_pub.publish(cmd)
                if self.last_label != "DOOR_ALIGN":
                    self.get_logger().info(f"[FgmDwaRobot] {self.last_label} → DOOR_ALIGN "
                                  f"(arcing toward light).")
                    self.last_label = "DOOR_ALIGN"
                self._emit_telemetry(now, elapsed, "DOOR_ALIGN", front_min,
                                     bearing_raw=light_center,
                                     bearing_smooth=None, gap_depth=light_width,
                                     min_r360=min_r360, cb_t0=cb_t0)
                return

        # ── BÖLÜM: Normal seyir — dt hesabı ─────────────────────────────
        # Buraya düştüysek özel bir mod (wedge/kapı) sözü almadı: sıradan
        # FGM/DWA seyri başlıyor. Önce iki tarama arasında geçen süreyi
        # (dt) ölç; ivme sınırlarını gerçek zamana bağlamak için gerekir.
        # 0.02-0.2 s'ye kırpılır: saat teklese bile tek adımda delice
        # hız/dönüş sıçramasına izin verilmez.
        if self.last_scan_time is None:
            dt = 0.1
        else:
            dt = max(0.02, min(0.2, now - self.last_scan_time))
        self.last_scan_time = now

        # Kartezyen engel listesi (DWA'nın çarpışma denetimi için): yakın
        # alandaki HER noktayı tut, yalnız uzak alanı seyrelt. Kör 1/4
        # seyreltme, ince bir plakayı (yandan 2.6 cm = 1-2 ışın) çarpışma
        # denetiminden tümüyle düşürebiliyordu — denemeler 6/13/19'un
        # rover plakalarında ölürken birebir aynı konfigürasyonların
        # süzülüp geçmesinin ardındaki erken-saha rus ruleti buydu.
        obstacles_cartesian = []
        for i in range(n):
            r = clean[i]
            if r < 1.2 or (i % 4 == 0 and r < 3.5):
                angle = i * radians_per_sample
                obstacles_cartesian.append((r * math.cos(angle), r * math.sin(angle)))

        # ── BÖLÜM: Hız valisi (governor) — açıklığa göre hız tavanı ─────
        # Dinamik hız tavanı, salt tam öne değil GİDİLEN yöne bakar:
        # çapraz süzülürken ±20° ön koni, yandan geçilmekte olan engeli
        # içerir ve seçilen yol açık olsa bile hızı kısar. Bu yüzden
        # yumuşatılmış yön boyunca açıklık karışıma katılır (0.85
        # iskonto); gerçek çarpışmalara karşı DWA'nın engel terimi zaten
        # nöbette.
        # Yöne bakan vali + GENİŞLİK vetosu: hız artışı yalnız gidiş
        # yönünü içeren derin yay gerçekten genişse (>=35°) uygulanır —
        # gerçek koridor düzlükleri geçer, cep ağızları (~20°; iki kez
        # kapana kıstıran rover-yanı tuzak gibi) geçemez. Aksi hâlde tam
        # ön açıklık esas alınır.
        governor_input = front_min
        # Gidiş yönünü içeren derin (>2 m) yayın genişliğini say.
        b_off = int(round(self.smoothed_bearing_deg * n / 360.0))
        if extended[b_off % n] > 2.0:
            arc = 1
            for o in range(1, 60):
                if extended[(b_off + o) % n] > 2.0:
                    arc += 1
                else:
                    break
            for o in range(1, 60):
                if extended[(b_off - o) % n] > 2.0:
                    arc += 1
                else:
                    break
            if arc * 360.0 / n >= 35.0:
                b_half = max(1, int(10 * n / 360))
                bearing_clear = min(extended[(b_off + o) % n]
                                    for o in range(-b_half, b_half + 1))
                governor_input = max(front_min, min(bearing_clear, 2.5) * 0.85)
        # Doğrusal rampa: açıklık büyüdükçe hız tavanı, taban hız ile
        # max_vel arasında orantılı yükselir.
        if governor_input >= self.governor_full_dist:
            current_max_vel = self.max_vel
        elif governor_input <= self.governor_floor_dist:
            current_max_vel = self.v_min_forward
        else:
            ratio = ((governor_input - self.governor_floor_dist)
                     / (self.governor_full_dist - self.governor_floor_dist))
            current_max_vel = self.v_min_forward + ratio * (self.max_vel - self.v_min_forward)

        # Koşullu sprint: hizalı + önü açık + dönmüyor -> tavanı kaldır.
        # Sprint yalnız açık manzarada (deep_frac kapı-tetik bloğundan
        # gelir): oda yarığına doğru 0.9 m/s'lik bir atak, DWA hızı
        # süzemeden burnu yüze toslatır.
        sprinting = (self.sprint_vel > self.max_vel
                     and governor_input > self.sprint_front_dist
                     and deep_frac > 0.30
                     and abs(self.smoothed_bearing_deg) < self.sprint_bearing_deg
                     and abs(self.cmd_angular_z) < self.sprint_max_w)
        if sprinting:
            current_max_vel = self.sprint_vel

        # DWA'nın "dinamik penceresi": sıradaki komut, mevcut hızdan ivme
        # sınırları içinde dt sürede ulaşılabilen (v, w) aralığında olmak
        # zorunda. Adaylar bu pencereden örneklenir.
        v_lo = max(self.v_min_forward, self.cmd_linear_x - self.max_linear_accel * dt)
        v_hi = min(current_max_vel,    self.cmd_linear_x + self.max_linear_accel * dt)
        # Pencere ters döndüyse (tavan aniden düştü) tek geçerli hıza çök.
        if v_lo > v_hi:
            v_lo = v_hi = max(self.v_min_forward, min(current_max_vel, self.cmd_linear_x))

        w_lo = max(-self.max_angular, self.cmd_angular_z - self.max_angular_accel * dt)
        w_hi = min(self.max_angular,  self.cmd_angular_z + self.max_angular_accel * dt)

        v_candidates = _linspace(v_lo, v_hi, self.v_samples)
        w_candidates = _linspace(w_lo, w_hi, self.w_samples)

        # Dinamik ağırlık çizelgesi (Sci Rep 2025 improved-DWA fikri):
        # açık alanda k -> 1 (ağırlıklar birebir statik taban), engel
        # yakınında k -> 0 (hız ödülü söner, yön + süreklilik ikiye
        # katlanır) — böylece örnekleyici kapıların içinde açıklığı hız
        # uğruna satmayı bırakır. Statik ağırlıkların iki rejimi birden
        # tutturamayacağı kanıtlı (Ogren & Leonard 2005).
        if self.weight_decay_dist > 0.0:
            wk = 1.0 - math.exp(-front_min / self.weight_decay_dist)
        else:
            wk = 1.0
        self.eff_speed_weight      = self.speed_weight * wk
        self.eff_heading_weight    = self.heading_weight * (2.0 - wk)
        self.eff_continuity_weight = self.continuity_weight * (2.0 - wk)

        # Hibrit hakemlik: FGM-I direksiyonu yalnız geniş görüş açıkken
        # (histerezis, sınırda mod titremesini önler). Anahtar ön
        # açıklıktır (bu koridorda deep_frac hemen her yerde < 0.22'de
        # doyar — denemeler 88/89 toplamda 2-4 geçiş gördü): önü açıkken
        # FGM-I; dar alanda, çarpışma-denetimli sabrıyla kazanan DWA.

        # ── BÖLÜM: Ağırlıklı karar matrisi (arbiter, kullanıcı isteği) ──
        # Tüm direksiyon algoritmaları her taramada birlikte koşar.
        # Adaylar — FGM fayda boşluğu (her zaman), kapı-ışığı mıknatısı
        # (MAG) ve ızgara-BFS erişilebilirliği (GRID) (kapı rejimi,
        # çerçeve < 1.5) — şu ölçütlerle puanlanır: parkur-aşağı ilerleme
        # (0.45), yön boyunca açıklık (0.25), son seçimle süreklilik
        # (0.15) ve kaynak güveni (0.15), artı 0.06 "koltuk sahibi"
        # bonusu. Kazanan yönü belirler; kazananla 25° içinde anlaşan
        # adaylar puan-ağırlıklı karışıma girer ("algo 1 + algo 2"),
        # anlaşmayanlar girmez — iki kapının ortalaması bir duvardır.
        if self.use_arbiter:
            cands = [("FGM", target_bearing_rad,
                      min(1.0, gap_info["chosen_width_m"] / 1.2))]
            # Tarla topoloji bekçisi (kullanıcı katı geri-vites yasağını
            # korudu, yani kıstırılmalar ÖNLENMEK zorunda): engel
            # yakınında ızgara her yerde koşar — BFS'i yalnız boş-alandan-
            # ulaşılabilir yönler önerir, dolayısıyla cep ağızları (derin
            # ışın ama kapalı topoloji) asla cezbetmez. Tarlada aday
            # olarak katılır; MAG kapılar bölgesine özel kalır.
            if (frame_min < 1.2 and self.door_mode is None
                    and self.door_cleared_count < 2 and self.use_grid_door):
                gf = self._grid_door_bearing(clean, n)
                if gf is not None:
                    cands.append(("GRID", gf[0], min(1.0, gf[1] / 2.0)))
            if (frame_min < 1.5 and self.door_mode is None
                    and self.door_cleared_count >= 2):
                # yalnız kapılar bölgesi: tarlanın ayarlı FGM'i yardım
                # istemez; tarla sıkışıklıklarını hakemlemek deneme
                # 203'ü yavaşlatmıştı
                amag_c, amag_w, _amag_slit_d = self._door_light(
                    extended, n, DOOR_COMMIT_GAP_HALF_DEG, front_min,
                    depth_thr=eff_depth_thr)
                if (amag_c is not None
                        and 0.12 <= amag_w <= self.door_trigger_gap_width
                        and abs(_angle_diff(
                            self.est_heading + math.radians(amag_c),
                            self.desired_yaw)) <= math.radians(50.0)):
                    cands.append(("MAG", math.radians(amag_c),
                                  min(1.0, amag_w / 0.6)))
                if self.use_grid_door:
                    g = self._grid_door_bearing(clean, n)
                    if g is not None:
                        cands.append(("GRID", g[0], min(1.0, g[1] / 2.0)))
            # dc_b: parkur-aşağı yönün robot çerçevesindeki açısı.
            dc_b = _angle_diff(self.desired_yaw, self.est_heading)
            scored = []
            for aname, ab, aconf in cands:
                # sert filtre: hiçbir aday parkur-aşağıdan 85°'den öteye
                # bakamaz (herkes kötü puan aldığında geriye dönük bir
                # yön, açıklık+süreklilikle yine de kazanabiliyor)
                if aname != "FGM" and abs(_angle_diff(ab, dc_b)) > math.radians(85.0):
                    continue
                prog = max(0.0, math.cos(_angle_diff(ab, dc_b)))
                ray = extended[int(round(math.degrees(ab))) % n]
                clear = min(ray, 2.0) / 2.0
                cont = 1.0 - abs(_angle_diff(ab, self._arb_last_bearing)) / math.pi
                sc = 0.45 * prog + 0.25 * clear + 0.15 * cont + 0.15 * aconf
                if aname == self._arb_last_winner:
                    sc += 0.06
                scored.append((sc, aname, ab))
            if not scored:
                # herkes veto yedi: yasal yelpazedeki en derin ışına yönel
                best_o, best_d = 0, -1.0
                for odeg in range(-85, 86, 5):
                    bb = dc_b + math.radians(odeg)
                    ray = extended[int(round(math.degrees(bb))) % n]
                    if ray > best_d:
                        best_d, best_o = ray, odeg
                scored = [(0.1, "ESC", dc_b + math.radians(best_o))]
            scored.sort(reverse=True)
            win_sc, win_name, win_b = scored[0]
            # Kazananla 25° içinde anlaşan adayları puan ağırlıklı ortala;
            # anlaşmayanı katma (iki kapının ortalaması duvardır).
            num = win_sc * win_b
            den = win_sc
            for sc, aname, ab in scored[1:]:
                if abs(_angle_diff(ab, win_b)) <= math.radians(25.0):
                    num += sc * ab
                    den += sc
            target_bearing_rad = num / den
            self._arb_last_winner = win_name
            self._arb_last_bearing = target_bearing_rad
            if len(scored) > 1 and (self.last_arb_log_t is None
                                    or now - self.last_arb_log_t >= 1.0):
                self.last_arb_log_t = now
                txt = " ".join(f"{a}={math.degrees(b):+.0f}deg/{sc:.2f}"
                               for sc, a, b in scored)
                self._log(f"ARBITER | {txt} -> {win_name} "
                          f"final={math.degrees(target_bearing_rad):+.0f}deg")

        # ── BÖLÜM: Hibrit kol seçimi — bu taramada FGM-I mı, DWA mı? ────
        # Histerezisli anahtar: ön kapanınca (close eşiği) DWA'ya geç,
        # iyice açılınca (open eşiği) FGM-I'ya dön. Eşikler farklı ki
        # sınırda mod titremesi olmasın.
        if self.hybrid_mode:
            if self._hybrid_fgm_active:
                if front_min < self.hybrid_close_frac:
                    self._hybrid_fgm_active = False
                    self._log(f"HYBRID -> DWA | front_min={front_min:.2f}")
            else:
                if front_min > self.hybrid_open_frac:
                    self._hybrid_fgm_active = True
                    self._log(f"HYBRID -> FGM | front_min={front_min:.2f}")
            use_dwa_now = not self._hybrid_fgm_active
        else:
            use_dwa_now = self.use_dwa

        # ── BÖLÜM: DWA örnekleme döngüsü — en ucuz yörünge kazanır ──────
        # Penceredeki her (v, w) çifti için kısa bir yörünge simüle edilir
        # ve maliyeti toplanır (engel + yön + hız + süreklilik). En düşük
        # maliyetli çift komut olur; 1e6 ve üstü maliyet = çarpışan aday.
        if use_dwa_now:
            best_cost = None
            best_v, best_w = self.v_min_forward, 0.0
            n_colliding = 0
            current_yaw = self.est_heading
            current_y   = self.dr_y
            for v in v_candidates:
                for w in w_candidates:
                    cost = self._trajectory_cost(v, w, target_bearing_rad, obstacles_cartesian,
                                                  self.cmd_angular_z, current_yaw, current_y)
                    if cost >= 1e6:
                        n_colliding += 1
                    if best_cost is None or cost < best_cost:
                        best_cost = cost
                        best_v, best_w = v, w

            # Maliyet dökümü için yalnız kazananı yeniden değerlendir
            # (örnekleyicinin gördüğü prev_w ile, yani cmd_angular_z
            # üzerine yazılmadan önceki değerle).
            _, dwa_breakdown = self._trajectory_cost(
                best_v, best_w, target_bearing_rad, obstacles_cartesian,
                self.cmd_angular_z, current_yaw, current_y, breakdown=True)
            dwa_breakdown["n_candidates"] = len(v_candidates) * len(w_candidates)
            dwa_breakdown["n_colliding"]  = n_colliding
            dwa_breakdown["v_at_hi"]      = int(abs(best_v - v_hi) < 1e-9)
            dwa_breakdown["w_at_edge"]    = int(abs(best_w - w_lo) < 1e-9 or abs(best_w - w_hi) < 1e-9)
        else:
            # FGM-I doğrudan direksiyon: fayda-boşluğu yönüne P-kontrol;
            # hız validen gelir ve dönüş sertliğiyle ölçeklenir; ikisi de
            # pürüzsüzlük için ivme-sınırlı pencereye kırpılır. Yön önce,
            # öngörülen baş (est + yön) U-dönüş duvarının içinde kalacak
            # şekilde kelepçelenir — FGM-I, o duvarın normalde yaşadığı
            # DWA maliyet katmanını atlar; koşu 84 bu kelepçe yokken
            # odada geri döndü (tüm zamanların en hızlı ara dereceleri —
            # yarı yol 24.7 s — olmasına rağmen gt 8.3).
            n_colliding = 0
            dwa_breakdown = {}
            # Dairesel kelepçe: yasal yönler, MUTLAK başı parkur-aşağı
            # yönün ±uturn'ü içinde kalanlardır. Eski düz [lo, hi]
            # aralığı uç-nokta takasıyla, aralık sarmalandığında yasal
            # yayın TÜMLEYENİNİ seçiyordu — yani robot tam parkur-yukarı
            # bakarken düz gitmek "yasal", geri dönmek "yasak" oluyordu
            # (deneme 167: ±15° doğruluklu pusulayla 15 s parkur-yukarı
            # yürüyüş).
            bearing_dc = _angle_diff(self.desired_yaw, self.est_heading)
            tb_rel = _angle_diff(target_bearing_rad, bearing_dc)
            tb_rel = max(-self._uturn_hard_rad,
                         min(self._uturn_hard_rad, tb_rel))
            tb = bearing_dc + tb_rel
            tb_lo = bearing_dc - self._uturn_hard_rad
            tb_hi = bearing_dc + self._uturn_hard_rad
            # (kapı mıknatısı yukarıdaki ağırlıklı arbiter'a taşındı —
            # target_bearing_rad karışık kararı zaten içeriyor)
            w_des = self.fgm_p_gain * tb
            v_des = (self.v_min_forward
                     + (current_max_vel - self.v_min_forward)
                     * max(0.0, math.cos(tb)))
            best_v = max(v_lo, min(v_hi, v_des))
            best_w = max(w_lo, min(w_hi, w_des))

        # ── BÖLÜM: Son yayın — bekçi, komut, günlükler ──────────────────
        # Seçilen (v, w) önce ilerleme bekçisinden geçer (takılma/yanlış
        # yön düzeltmeleri), sonra /cmd_vel'e yayınlanır ve telemetri
        # yazılır.
        best_v, best_w = self._progress_guard(best_v, best_w, now, tag="NAV")
        self.cmd_linear_x  = best_v
        self.cmd_angular_z = best_w

        # Zikzak ölçümü: komutlanan dönüş hızının işaret değişimlerini
        # say (sıfıra yakın ölü bant hariç).
        w_sign = 1 if best_w > 0.1 else (-1 if best_w < -0.1 else 0)
        if w_sign != 0:
            if self.prev_w_sign != 0 and w_sign != self.prev_w_sign:
                self.w_flip_count += 1
            self.prev_w_sign = w_sign

        cmd = Twist()
        cmd.linear.x  = self.cmd_linear_x
        cmd.angular.z = self.cmd_angular_z
        self.cmd_pub.publish(cmd)

        self._update_anomaly_trackers(now, elapsed, self.last_label, front_min, extended)

        label = self._state_label(front_min, self.robot_radius, best_v, self.max_vel)

        if label != self.last_label:
            self.get_logger().info(
                f"[FgmDwaRobot] {self.last_label} → {label}  "
                f"front_min={front_min:.2f} gap_bearing={bearing_deg:.0f}°"
            )
            self._log(
                f"TRANSITION {self.last_label} → {label} | front_min={front_min:.2f} "
                f"gap_bearing={bearing_deg:.0f} bearing_smooth={self.smoothed_bearing_deg:.1f} "
                f"gap_depth={gap_depth:.2f} v={best_v:.2f} w={best_w:.2f}"
            )
            self.last_label = label

        # Zamana dayalı 5 s kalp atışı (eski int(elapsed)%5 koşulu, tarama
        # zamanlamasına göre çift ateşliyor ya da tümden kaçırıyordu).
        if self.last_sample_log_t is None or now - self.last_sample_log_t >= 5.0:
            self.last_sample_log_t = now
            self._log(
                f"t={elapsed:5.1f}s  {label:<7} v={best_v:.2f} w={best_w:.2f} "
                f"gap_bearing={bearing_deg:.0f} bearing_smooth={self.smoothed_bearing_deg:.1f} "
                f"gap_depth={gap_depth:.2f} front_min={front_min:.2f} "
                f"vmax_cap={current_max_vel:.2f} "
                f"colliding={n_colliding}/{dwa_breakdown.get('n_candidates', 0)} "
                f"wk={wk:.2f}"
            )
            self.get_logger().info(
                f"[FgmDwaRobot] t={elapsed:.0f}s {label} v={best_v:.2f} w={best_w:+.2f} "
                f"front_min={front_min:.2f} bearing={self.smoothed_bearing_deg:+.0f}°"
            )

        if self.have_odom and (self.last_xy_log_t is None or now - self.last_xy_log_t >= 1.0):
            self.last_xy_log_t = now
            self._log(f"XY t={elapsed:5.1f} x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f} state={label}")

        self._emit_telemetry(now, elapsed, label, front_min,
                             bearing_raw=bearing_deg,
                             bearing_smooth=self.smoothed_bearing_deg,
                             gap_depth=gap_depth, current_max_vel=current_max_vel,
                             dwa=dwa_breakdown, min_r360=min_r360,
                             gap_info=gap_info, cb_t0=cb_t0)

    # ─── BÖLÜM: Kapanış — koşu karnesi ve temiz durdurma ─────────────────────

    def _shutdown(self):
        # 60 s dolunca çağrılır: robotu durdurur, koşunun karnesini yazar
        # (SUMMARY satırları), abonelikleri bırakır ve Gazebo'daki 'sun'
        # ışığını silmeyi dener.
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

            # Genişletilmiş gözlemlenebilirlik özeti (ek satırlar; üstteki
            # SUMMARY satırı, mevcut ayrıştırıcılar ve karşılaştırmalar
            # bozulmasın diye biçimini aynen korur).
            avg_speed = self.path_length / max(1e-6, self.run_duration)
            efficiency = net_disp / max(1e-6, self.path_length)
            state_breakdown = " ".join(
                f"{k}={v:.1f}s" for k, v in sorted(self.state_time.items(),
                                                    key=lambda kv: -kv[1]))
            closest = ("t={:.1f}s at ({:.2f},{:.2f})".format(*self.min_front_ctx)
                       if self.min_front_ctx else "n/a")
            cb_avg = self.cb_ms_sum / max(1, self.cb_count)
            self._log(f"SUMMARY2 avg_speed={avg_speed:.2f}m/s path_efficiency={efficiency:.2f} "
                      f"closest_approach[{closest}] states[{state_breakdown}]")
            self._log(f"SUMMARY3 doors: commits={self.door_commit_count} "
                      f"cleared={self.door_cleared_count} timeouts={self.door_timeout_count} "
                      f"commit_time={self.door_commit_total_time:.2f}s "
                      f"grid_commit_scans={self.grid_commit_scans}")
            self._log(f"SUMMARY4 scans={self.scan_count} scan_gaps={self.scan_gap_count} "
                      f"cb_avg={cb_avg:.1f}ms cb_max={self.cb_ms_max:.1f}ms")
            self._log(f"SUMMARY5 deficiencies: w_flips={self.w_flip_count} "
                      f"gap_switches={self.gap_switch_count} "
                      f"fgm_fallbacks={self.fgm_fallback_count} "
                      f"commit_fallbacks={self.commit_fallback_count} "
                      f"stuck={self.stuck_event_count} wrong_way={self.wrongway_event_count} "
                      f"regress={self.regress_event_count} pinball={self.pinball_event_count} "
                      f"guard_uturn={self.guard_uturn_count} "
                      f"guard_back={self.guard_back_count} "
                      f"guard_pinball={self.guard_pinball_count} "
                      f"corner_avoid={self.corner_avoid_count} "
                      f"backward_motion={self.backward_motion_count} "
                      f"upcourse_motion={self.upcourse_motion_count} "
                      f"min_r360_seen={self.min_r360_seen:.2f}")
            if self.have_gt:
                gt_net = math.hypot(self.gt_x - self.gt_start_xy[0],
                                    self.gt_y - self.gt_start_xy[1])
                slip = self.path_length - self.gt_path_length
                self._log(f"SUMMARY_GT gt_net_displacement={gt_net:.2f}m "
                          f"gt_path={self.gt_path_length:.2f}m "
                          f"encoder_minus_gt_path={slip:+.2f}m "
                          f"(positive = wheel slip, i.e. wall grinding)")
                self.get_logger().info(f"[FgmDwaRobot] GROUND TRUTH net_disp={gt_net:.2f}m "
                              f"path={self.gt_path_length:.2f}m slip={slip:+.2f}m")
            self.get_logger().info(
                f"[FgmDwaRobot] deficiencies: stuck={self.stuck_event_count} "
                f"wrong_way={self.wrongway_event_count} regress={self.regress_event_count} "
                f"min_r360={self.min_r360_seen:.2f}m | zigzag: w_flips={self.w_flip_count} "
                f"gap_switches={self.gap_switch_count} "
                f"fallbacks={self.fgm_fallback_count}+{self.commit_fallback_count}"
            )
            self.get_logger().info(
                f"[FgmDwaRobot] avg_speed={avg_speed:.2f}m/s efficiency={efficiency:.2f} "
                f"states[{state_breakdown}]"
            )
            self.get_logger().info(
                f"[FgmDwaRobot] doors: commits={self.door_commit_count} cleared={self.door_cleared_count} "
                f"timeouts={self.door_timeout_count} | scans={self.scan_count} "
                f"gaps={self.scan_gap_count} cb_avg={cb_avg:.1f}ms cb_max={self.cb_ms_max:.1f}ms"
            )

        # /scan, /odom, /ground_truth_odom aboneliklerini kapat.
        self.destroy_subscription(self.scan_sub)
        self.destroy_subscription(self.odom_sub)
        self.destroy_subscription(self.gt_sub)
        self.get_logger().info("Unsubscribed from /scan.")
        self.cmd_pub.publish(Twist())

        # Bitişte Gazebo'daki 'sun' ışığını silmeyi dene; başarısızlık
        # ölümcül değil, yalnızca uyarı olarak loglanır.
        try:
            self._delete_sun_light()
        except Exception as exc:
            self.get_logger().warn(f"[FgmDwaRobot] Could not delete light: {exc}")
            self._log(f"WARNING delete_light: {exc}")

        self._log("RUN END")
        self._logf.close()
        self._telef.close()
        if self._scanf is not None:
            self._scanf.close()
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


# ─── BÖLÜM: Giriş noktası ───
# ros2 run / ros2 launch bu dosyayı çalıştırınca düğümü kurar ve döngüyü başlatır.
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
