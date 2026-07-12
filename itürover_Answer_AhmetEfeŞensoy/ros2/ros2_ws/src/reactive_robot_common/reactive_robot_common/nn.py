"""
nn.py — Sıfırdan yazılmış NumPy MLP.

PyTorch/TF yok, sadece numpy. ReLU gizli katmanlar, ham logit çıkışı.
Eğitim: log-sum-exp kararlı çapraz entropi + Adam. Kayıt: .npz.
"""

import numpy as np


class MLP:
    """İleri beslemeli MLP: forward, backward, Adam güncellemesi, kayıt/yükleme."""

    def __init__(self, sizes, seed=0):
        """sizes örn. (41, 64, 64, 5). He başlatma: W ~ N(0, sqrt(2/fan_in)), b=0."""
        assert len(sizes) >= 2
        self.sizes = tuple(sizes)
        rng = np.random.RandomState(seed)

        self.params = {}
        for i in range(len(sizes) - 1):
            fan_in = sizes[i]
            std = np.sqrt(2.0 / fan_in)
            self.params[f"W{i+1}"] = (rng.randn(sizes[i], sizes[i+1]) * std).astype(np.float32)
            self.params[f"b{i+1}"] = np.zeros(sizes[i+1], dtype=np.float32)

        # Adam momentleri ilk adam_step çağrısında başlatılır
        self._m = None
        self._v = None
        self._t = 0

    # ── İleri geçiş ──────────────────────────────────────────────────────────

    def forward(self, X):
        """X: (B, girdi_boyutu) → (logits, cache). Gizli katmanlar ReLU."""
        n_layers = len(self.sizes) - 1
        cache = {"X": X}
        A = X
        for i in range(1, n_layers):
            Z = A @ self.params[f"W{i}"] + self.params[f"b{i}"]
            cache[f"Z{i}"] = Z
            A = np.maximum(0.0, Z)   # ReLU
            cache[f"A{i}"] = A
        # çıkış katmanı: aktivasyon yok, ham logit
        Z_out = A @ self.params[f"W{n_layers}"] + self.params[f"b{n_layers}"]
        cache[f"Z{n_layers}"] = Z_out
        cache["n_layers"] = n_layers
        return Z_out, cache

    # ── Kayıp ────────────────────────────────────────────────────────────────

    @staticmethod
    def cross_entropy_loss(logits, y_true):
        """logits (B, sınıf) + y_true (B,) → ortalama çapraz entropi (skaler)."""
        B = logits.shape[0]
        # sayısal kararlılık: satır maksimumunu çıkar
        shifted = logits - logits.max(axis=1, keepdims=True)
        log_sum_exp = np.log(np.exp(shifted).sum(axis=1))      # (B,)
        log_probs = shifted[np.arange(B), y_true] - log_sum_exp
        return float(-log_probs.mean())

    # ── Geri yayılım ─────────────────────────────────────────────────────────

    def backward(self, logits, cache, y_true):
        """Softmax + çapraz entropi + ReLU için elle geri yayılım. Grad dict döndürür."""
        n_layers = cache["n_layers"]
        B = logits.shape[0]

        shifted = logits - logits.max(axis=1, keepdims=True)
        exp_z   = np.exp(shifted)
        probs   = exp_z / exp_z.sum(axis=1, keepdims=True)

        # CE gradyanı: probs - one_hot(y_true)
        dZ = probs.copy()
        dZ[np.arange(B), y_true] -= 1.0
        dZ /= B

        grads = {}
        for i in range(n_layers, 0, -1):
            A_prev = cache.get(f"A{i-1}", cache["X"])
            grads[f"W{i}"] = A_prev.T @ dZ
            grads[f"b{i}"] = dZ.sum(axis=0)

            if i > 1:
                dA = dZ @ self.params[f"W{i}"].T
                dZ = dA * (cache[f"Z{i-1}"] > 0).astype(np.float32)  # ReLU kapısı

        return grads

    # ── Adam güncellemesi ─────────────────────────────────────────────────────

    def adam_step(self, grads, lr=1e-3, beta1=0.9, beta2=0.999, eps=1e-8):
        """self.params üzerinde yerinde Adam güncellemesi."""
        if self._m is None:
            self._m = {k: np.zeros_like(v) for k, v in self.params.items()}
            self._v = {k: np.zeros_like(v) for k, v in self.params.items()}

        self._t += 1
        t = self._t
        for k in self.params:
            g = grads[k].astype(np.float32)
            self._m[k] = beta1 * self._m[k] + (1.0 - beta1) * g
            self._v[k] = beta2 * self._v[k] + (1.0 - beta2) * g * g
            m_hat = self._m[k] / (1.0 - beta1 ** t)
            v_hat = self._v[k] / (1.0 - beta2 ** t)
            self.params[k] -= lr * m_hat / (np.sqrt(v_hat) + eps)

    # ── Tahmin ───────────────────────────────────────────────────────────────

    def predict(self, X):
        """Argmax sınıf indisi döndürür, şekil (B,)."""
        logits, _ = self.forward(X)
        return logits.argmax(axis=1)

    def predict_one(self, x):
        """Tek örnek: x (girdi_boyutu,) → int aksiyon indisi."""
        return int(self.predict(x[np.newaxis, :])[0])

    # ── Kayıt / Yükleme ──────────────────────────────────────────────────────

    def save(self, path):
        """Ağırlıkları ve mimariyi .npz olarak kaydet."""
        data = dict(self.params)
        data["_sizes"] = np.array(self.sizes, dtype=np.int64)
        np.savez(path, **data)

    @classmethod
    def load(cls, path):
        """Bir .npz dosyasından ağırlıkları yükle, hazır MLP örneği döndür."""
        data = np.load(path, allow_pickle=False)
        sizes = tuple(int(x) for x in data["_sizes"])
        net = cls(sizes, seed=0)
        for k in net.params:
            net.params[k] = data[k].astype(np.float32)
        return net
