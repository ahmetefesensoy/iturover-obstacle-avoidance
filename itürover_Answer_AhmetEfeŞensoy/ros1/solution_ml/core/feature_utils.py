"""
feature_utils.py — Lazer scan → sektör özellik vektörü + aksiyon tablosu.

Tüm ML modülleri (collect_data, ml_robot, train_bc, dqn_finetune, test_model)
bu dosyayı import eder.  Yalnızca numpy kullanır — rospy gerektirmez.
"""

import math
import numpy as np


N_SECTORS = 36

# (isim, ileri_frac, açısal_frac) — gerçek hızlar action_table() ile hesaplanır
ACTIONS = [
    ("HARD_LEFT",  0.35,  1.0),
    ("LEFT",       0.70,  0.45),
    ("FORWARD",    1.0,   0.0),
    ("RIGHT",      0.70, -0.45),
    ("HARD_RIGHT", 0.35, -1.0),
]


def clean_ranges(ranges, range_min, range_max):
    """Geçersiz lazer mesafelerini (inf, NaN, sınır dışı) range_max ile doldur."""
    arr = np.asarray(ranges, dtype=np.float32)
    bad = (~np.isfinite(arr)) | (arr < range_min) | (arr > range_max)
    arr[bad] = float(range_max)
    return arr


def scan_to_sectors(ranges, angle_min, angle_increment,
                    range_min, range_max, n_sectors=N_SECTORS):
    """
    Ham lazer mesafe dizisini n_sectors eşit açısal dilime böl.
    Her dilim kendi içindeki en küçük mesafeyi saklar.
    Çıktı: [0, 1] aralığına normalize edilmiş float32 array (n_sectors,).
    """
    cleaned = clean_ranges(ranges, range_min, range_max)
    n = len(cleaned)

    sector_width = 2.0 * math.pi / n_sectors
    sector_offset = -sector_width / 2.0

    angles = np.empty(n, dtype=np.float64)
    for i in range(n):
        a = angle_min + i * angle_increment
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        angles[i] = a

    sectors = np.full(n_sectors, float(range_max), dtype=np.float32)
    for i in range(n):
        raw_idx = int(math.floor((angles[i] - sector_offset) / sector_width))
        k = raw_idx % n_sectors
        if cleaned[i] < sectors[k]:
            sectors[k] = cleaned[i]

    span = float(range_max - range_min)
    sectors = (sectors - range_min) / span
    return np.clip(sectors, 0.0, 1.0).astype(np.float32)


def action_table(max_vel, max_angular, table=ACTIONS):
    """(isim, v, w) listesi döndür — ACTIONS'taki fraksiyonları gerçek hızlara çevir."""
    return [(name, fwd_frac * max_vel, ang_frac * max_angular)
            for name, fwd_frac, ang_frac in table]


def cmd_to_action_index(v, w, max_vel, max_angular, table=ACTIONS):
    """
    Sürekli (v, w) komutunu en yakın ayrık aksiyona eşle.
    Mesafe metriği: (Δv/max_vel)² + (Δw/max_angular)² — ölçekten bağımsız.
    """
    best_idx, best_dist = 0, float("inf")
    for i, (_, tv, tw) in enumerate(action_table(max_vel, max_angular, table)):
        dv = (v - tv) / max(max_vel, 1e-6)
        dw = (w - tw) / max(max_angular, 1e-6)
        dist = dv * dv + dw * dw
        if dist < best_dist:
            best_dist = dist
            best_idx = i
    return best_idx


def build_feature_vector(sectors, prev_action_idx, n_actions=None):
    """
    Sektör mesafeleri + önceki aksiyon one-hot'unu birleştir.
    Giriş: sectors (n_sectors,), prev_action_idx int
    Çıktı: float32 array (n_sectors + n_actions,)
    """
    if n_actions is None:
        n_actions = len(ACTIONS)
    one_hot = np.zeros(n_actions, dtype=np.float32)
    one_hot[prev_action_idx] = 1.0
    return np.concatenate([sectors, one_hot])


def augment(sectors, rng, noise_std=0.02, dropout_p=0.05, max_shift=2):
    """
    Eğitim sırasında domain randomization: gürültü, dropout, dairesel kaydırma.
    Yalnızca sektör dilimine uygulanır — one-hot kısmına asla dokunma.
    """
    s = sectors.copy()
    n = len(s)

    if noise_std > 0:
        s = s + rng.randn(n).astype(np.float32) * noise_std

    if dropout_p > 0:
        mask = rng.rand(n) < dropout_p
        s[mask] = 1.0

    if max_shift > 0:
        shift = int(rng.randint(-max_shift, max_shift + 1))
        s = np.roll(s, shift)

    return np.clip(s, 0.0, 1.0).astype(np.float32)
