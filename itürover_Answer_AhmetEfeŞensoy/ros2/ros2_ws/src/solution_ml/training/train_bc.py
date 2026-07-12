#!/usr/bin/env python3
"""
train_bc.py — Davranışsal Klonlama (BC) eğitim scripti.

ROS gerektirmez, veri toplandıktan sonra doğrudan çalıştır:
  python3 train_bc.py --datasets ~/rover_ml_data/*.npz --augment --epochs 200

Bölme: ardışık kare sızmasını önlemek için tüm dosya/çalışma bazlı.
Artırma: yalnızca sektör dilimine uygulanır, önceki-aksiyon one-hot'a dokunulmaz.
"""

import argparse
import os
import sys
import math
import numpy as np


from reactive_robot_common import feature_utils as fu
from reactive_robot_common.nn import MLP


# ── Vektörize batch augmentasyon ─────────────────────────────────────────────
# fu.augment() satır-satır Python döngüsüyle 400k+ örnekte epoch başına
# onlarca saniye ekliyordu (18.7ms/batch ölçüldü, 200 epoch'ta ~100 dakika).
# Aynı gürültü/dropout/circular-shift mantığını tüm batch üzerinde tek seferde
# uygulayan bu sürüm augment API'sini (feature_utils.augment) değiştirmez,
# yalnızca train_bc.py'nin kendi eğitim döngüsünde kullanılır.
def augment_batch(sectors_batch, rng, noise_std=0.02, dropout_p=0.05, max_shift=2):
    """sectors_batch (B, n_sectors) üzerinde satır-bazlı gürültü/dropout/shift uygula."""
    B, n = sectors_batch.shape
    s = sectors_batch.copy()

    if noise_std > 0:
        s = s + rng.randn(B, n).astype(np.float32) * noise_std

    if dropout_p > 0:
        mask = rng.rand(B, n) < dropout_p
        s[mask] = 1.0

    if max_shift > 0:
        shifts = rng.randint(-max_shift, max_shift + 1, size=B)
        col_idx = (np.arange(n)[None, :] - shifts[:, None]) % n
        s = np.take_along_axis(s, col_idx, axis=1)

    return np.clip(s, 0.0, 1.0).astype(np.float32)


# ── Veri yükleme ─────────────────────────────────────────────────────────────

def load_datasets(paths):
    """Bir veya daha fazla .npz yükle. X (N, girdi), y (N,), run_ids (N,) döndürür."""
    X_list, y_list, ids_list = [], [], []
    for run_id, path in enumerate(paths):
        data = np.load(path, allow_pickle=False)
        X_list.append(data["X"].astype(np.float32))
        y_list.append(data["y"].astype(np.int64))
        ids_list.append(np.full(len(data["y"]), run_id, dtype=np.int64))
        tag = str(data["world_tag"]) if "world_tag" in data else "?"
        print(f"  [{run_id}] {path}  satır={len(data['y'])}  dünya={tag}")
    X = np.concatenate(X_list, axis=0)
    y = np.concatenate(y_list, axis=0)
    run_ids = np.concatenate(ids_list, axis=0)
    print(f"Toplam: {len(y)} örnek  girdi_boyutu={X.shape[1]}")
    return X, y, run_ids


# ── Doğrulama bölmesi ─────────────────────────────────────────────────────────

def split_holdout(X, y, run_ids, val_frac=0.2, seed=0):
    """Tüm çalışma bazlı bölme — ardışık kare sızmasını önler."""
    rng = np.random.RandomState(seed)
    unique_runs = np.unique(run_ids)

    if len(unique_runs) == 1:
        # Tek çalışma: son val_frac% bloğunu doğrulama olarak ayır
        n = len(y)
        split = max(1, int(n * (1.0 - val_frac)))
        return X[:split], y[:split], X[split:], y[split:]

    # Çoklu çalışma: val_frac'e ulaşana kadar tüm çalışmaları rastgele val'e ata
    rng.shuffle(unique_runs)
    n_val_target = int(len(y) * val_frac)
    val_runs = set()
    n_val = 0
    for rid in unique_runs:
        run_size = int((run_ids == rid).sum())
        if n_val < n_val_target:
            val_runs.add(rid)
            n_val += run_size

    val_mask = np.isin(run_ids, list(val_runs))
    return X[~val_mask], y[~val_mask], X[val_mask], y[val_mask]


# ── Eğitim döngüsü ─────────────────────────────────────────────────────────────

def train(X_train, y_train, X_val, y_val,
          sizes, epochs, batch_size, lr, do_augment=False, seed=0,
          class_weights=None, init_net=None):
    """Mini-batch çapraz entropi + Adam ile MLP eğit. Eğitilmiş ağı döndürür."""
    rng = np.random.RandomState(seed)
    if init_net is not None:
        net = init_net
    else:
        net = MLP(sizes, seed=seed)

    n_sectors = sizes[0] - len(fu.ACTIONS)
    n_train = len(y_train)
    n_batches = max(1, math.ceil(n_train / batch_size))

    # Sınıf ağırlıkları tensörü
    if class_weights is not None:
        w_arr = np.array([class_weights.get(i, 1.0) for i in range(sizes[-1])],
                         dtype=np.float32)
    else:
        w_arr = None

    print(f"\nMLP boyutları: {sizes}  parametre: "
          + str(sum(v.size for v in net.params.values())))
    print(f"Eğitim: {n_train} örnek  Doğrulama: {len(y_val)} örnek")
    print(f"Epoch: {epochs}  batch: {batch_size}  lr: {lr}  artırma: {do_augment}\n")
    print(f"{'Epoch':>6}  {'EğKayıp':>10}  {'DoğKayıp':>10}  {'DoğAcc':>8}")
    print("-" * 42)

    for epoch in range(1, epochs + 1):
        idx = np.arange(n_train)
        rng.shuffle(idx)
        X_shuf = X_train[idx]
        y_shuf = y_train[idx]

        epoch_loss = 0.0
        for b in range(n_batches):
            start = b * batch_size
            end   = min(start + batch_size, n_train)
            Xb = X_shuf[start:end].copy()
            yb = y_shuf[start:end]

            if do_augment:
                Xb[:, :n_sectors] = augment_batch(
                    Xb[:, :n_sectors], rng,
                    noise_std=0.02, dropout_p=0.05, max_shift=2
                )

            logits, cache = net.forward(Xb)
            if w_arr is not None:
                log_probs = logits - np.log(np.exp(logits).sum(axis=1, keepdims=True) + 1e-8)
                sample_weights = w_arr[yb]
                loss = -(sample_weights * log_probs[np.arange(len(yb)), yb]).mean()
            else:
                loss = MLP.cross_entropy_loss(logits, yb)
            epoch_loss += loss * (end - start)

            grads = net.backward(logits, cache, yb)
            net.adam_step(grads, lr=lr)

        epoch_loss /= n_train

        val_logits, _ = net.forward(X_val)
        val_loss = MLP.cross_entropy_loss(val_logits, y_val)
        val_acc  = float((val_logits.argmax(axis=1) == y_val).mean())

        if epoch % max(1, epochs // 20) == 0 or epoch == epochs:
            print(f"{epoch:>6}  {epoch_loss:>10.4f}  {val_loss:>10.4f}  "
                  f"{val_acc:>8.1%}")

    return net


# ── Aksiyon dağılımı ─────────────────────────────────────────────────────────

def print_action_dist(y, label=""):
    counts = [(fu.ACTIONS[i][0], int((y == i).sum())) for i in range(len(fu.ACTIONS))]
    total = len(y)
    parts = [f"{name}={n} ({100*n/total:.0f}%)" for name, n in counts]
    print(f"Aksiyon dağılımı {label}: " + "  ".join(parts))


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Train BC policy from collected .npz dataset(s)."
    )
    parser.add_argument("--datasets", nargs="+", required=True,
                        help="Path(s) to .npz dataset file(s)")
    parser.add_argument("--out",        default="weights/bc_weights.npz",
                        help="Output path for trained weights")
    parser.add_argument("--epochs",     type=int,   default=200)
    parser.add_argument("--batch-size", type=int,   default=256)
    parser.add_argument("--lr",         type=float, default=1e-3)
    parser.add_argument("--augment",    action="store_true",
                        help="Apply domain-randomisation augmentation during training")
    parser.add_argument("--val-frac",   type=float, default=0.2)
    parser.add_argument("--seed",       type=int,   default=0)
    parser.add_argument("--hidden",     nargs="+",  type=int, default=[128, 64],
                        help="Hidden layer sizes (default: 128 64)")
    parser.add_argument("--class-weights", action="store_true",
                        help="Use class weights to balance action distribution")
    parser.add_argument("--init-weights", default=None,
                        help="Path to existing .npz weights for fine-tuning")
    args = parser.parse_args()

    print(f"\n{'='*50}")
    print("Loading datasets...")
    X, y, run_ids = load_datasets(args.datasets)
    print_action_dist(y, "full")

    print("\nSplitting train/val by whole run (avoiding temporal leakage)...")
    X_tr, y_tr, X_val, y_val = split_holdout(
        X, y, run_ids, val_frac=args.val_frac, seed=args.seed
    )
    print_action_dist(y_tr,  "train")
    print_action_dist(y_val, "val  ")

    input_dim = X.shape[1]
    n_classes  = len(fu.ACTIONS)
    sizes = tuple([input_dim] + args.hidden + [n_classes])

    # İstenirse sınıf ağırlıklarını hesapla
    class_weight_dict = None
    if args.class_weights:
        n_total = len(y_tr)
        class_weight_dict = {}
        for i in range(n_classes):
            count = int((y_tr == i).sum())
            if count > 0:
                class_weight_dict[i] = n_total / (n_classes * count)
        print(f"\nClass weights: { {k: f'{v:.2f}' for k, v in class_weight_dict.items()} }")

    init_net = None
    if args.init_weights:
        print(f"\nFine-tuning from: {args.init_weights}")
        init_net = MLP(sizes)
        init_net.load(args.init_weights)

    net = train(X_tr, y_tr, X_val, y_val,
                sizes=sizes,
                epochs=args.epochs,
                batch_size=args.batch_size,
                lr=args.lr,
                do_augment=args.augment,
                seed=args.seed,
                class_weights=class_weight_dict,
                init_net=init_net)

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # yol .npz ile bitmiyorsa np.savez otomatik olarak ekler
    save_path = args.out if args.out.endswith(".npz") else args.out + ".npz"
    net.save(save_path)
    print(f"\nWeights saved → {save_path}")


if __name__ == "__main__":
    main()
