"""
expert_policy.py — Birleşik potansiyel-alan tabanlı expert controller.

Eski prepare_barn_data.py mantığındaki 3 ayrık mod (WAYPOINT / AVOID_SOFT /
AVOID_HARD) yerine, HER adımda hedefe-çekim + engelden-itilme vektörlerini
tek bir açıda birleştirir. Mod geçişi yok, ani sıçrama yok — robot dar
geçitlerde waypoint'i asla tamamen gözden kaybetmiyor.

Yalnızca numpy kullanır — rospy gerektirmez. data toplama (prepare_barn_data),
standalone doğrulama ve DAgger yeniden-etiketleme tarafından ortak kullanılır.
"""

import math
import numpy as np

import feature_utils as fu

# Ağırlıklar: itilme sadece yakın mesafede baskın olsun diye repel_gain
# attract_gain'den büyük ama repülsif katkı 1/d^2 ile hızla sıfıra düşüyor.
ATTRACT_GAIN = 1.0
REPEL_GAIN   = 0.35
REPEL_D0     = 0.5   # bu mesafenin ötesindeki sektörler itme üretmez


def _sector_bearings(n_sectors):
    """Her sektörün merkez açısını (radyan, robot çerçevesinde) döndür."""
    sector_width = 2.0 * math.pi / n_sectors
    idx = np.arange(n_sectors)
    angles = idx * sector_width
    # [-pi, pi] aralığına sar
    angles = np.where(angles > math.pi, angles - 2.0 * math.pi, angles)
    return angles


_SECTOR_BEARINGS = {}


def _get_sector_bearings(n_sectors):
    if n_sectors not in _SECTOR_BEARINGS:
        _SECTOR_BEARINGS[n_sectors] = _sector_bearings(n_sectors)
    return _SECTOR_BEARINGS[n_sectors]


def repulsive_bearing_and_strength(sectors, range_min, range_max, n_sectors):
    """
    Normalize sektör mesafelerinden (0..1) birleşik itme vektörünü hesapla.
    Döndürür: (bearing_robot_frame, strength) — strength 0 ise itme yok demektir.
    """
    bearings = _get_sector_bearings(n_sectors)
    real_dist = sectors * (range_max - range_min) + range_min

    close = real_dist < REPEL_D0
    if not np.any(close):
        return 0.0, 0.0

    # Ters-kare tipi itki: yakın sektör daha güçlü iter, yönü sektörün TERSİ.
    # Ortalama alınır (toplam değil) — aksi halde dar koridorlarda çok sayıda
    # yakın sektör olması (duvarlar boyunca), her biri az itse bile, itme
    # büyüklüğünü yapay olarak şişirip robotu tek yöne kilitliyordu.
    mag = (1.0 / np.maximum(real_dist[close], 1e-3) - 1.0 / REPEL_D0)
    mag = np.clip(mag, 0.0, None)

    push_x = -np.mean(mag * np.cos(bearings[close]))
    push_y = -np.mean(mag * np.sin(bearings[close]))

    strength = math.hypot(push_x, push_y)
    if strength < 1e-6:
        return 0.0, 0.0
    return math.atan2(push_y, push_x), strength


def combined_heading(robot_x, robot_y, robot_yaw, target_x, target_y,
                      sectors, range_min, range_max, n_sectors):
    """
    Hedefe-çekim + engelden-itilme vektörlerini birleştirip DÜNYA çerçevesinde
    tek bir hedef yön (radyan) döndür.
    """
    # Çekim: hedefe düz bearing, dünya çerçevesinde
    dx = target_x - robot_x
    dy = target_y - robot_y
    attract_bearing = math.atan2(dy, dx)
    attract_x = ATTRACT_GAIN * math.cos(attract_bearing)
    attract_y = ATTRACT_GAIN * math.sin(attract_bearing)

    # İtilme: robot çerçevesinde hesaplanır, dünya çerçevesine çevrilir
    repel_bearing_robot, repel_strength = repulsive_bearing_and_strength(
        sectors, range_min, range_max, n_sectors
    )
    repel_bearing_world = repel_bearing_robot + robot_yaw
    repel_x = REPEL_GAIN * repel_strength * math.cos(repel_bearing_world)
    repel_y = REPEL_GAIN * repel_strength * math.sin(repel_bearing_world)

    total_x = attract_x + repel_x
    total_y = attract_y + repel_y

    if abs(total_x) < 1e-9 and abs(total_y) < 1e-9:
        return attract_bearing
    return math.atan2(total_y, total_x)


def expert_action(robot_x, robot_y, robot_yaw, target_x, target_y,
                   ranges, angle_min, angle_increment, range_min, range_max,
                   max_vel, max_angular, n_sectors=fu.N_SECTORS):
    """
    Tek adımlık expert karar fonksiyonu.
    Girdi: robot poz + hedef waypoint + ham LiDAR taraması.
    Çıktı: (action_idx, sectors) — sectors çağıran tarafından feature vektörü
    kurmak için de kullanılabilir (yeniden hesaplamayı önler).
    """
    sectors = fu.scan_to_sectors(ranges, angle_min, angle_increment,
                                  range_min, range_max, n_sectors)

    target_heading_world = combined_heading(
        robot_x, robot_y, robot_yaw, target_x, target_y,
        sectors, range_min, range_max, n_sectors
    )

    ang_err = target_heading_world - robot_yaw
    while ang_err > math.pi:
        ang_err -= 2.0 * math.pi
    while ang_err < -math.pi:
        ang_err += 2.0 * math.pi

    w = max(-max_angular, min(max_angular, 1.8 * ang_err))
    v = max_vel * max(0.15, 1.0 - min(1.0, abs(ang_err) / (math.pi / 4.0)))

    action_idx = fu.cmd_to_action_index(v, w, max_vel, max_angular)
    return action_idx, sectors
