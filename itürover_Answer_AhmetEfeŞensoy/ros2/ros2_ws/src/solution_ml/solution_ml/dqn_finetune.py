#!/usr/bin/env python3
"""dqn_finetune.py — BC ile başlatılan MLP politikasının DQN ile ince ayarı (ROS 2).

Episode tabanlı: robotu Gazebo'da spawn noktasına ışınlar (SetModelState),
epsilon-greedy ile ayrık aksiyon seçer, replay buffer'dan mini-batch ile
online ağı günceller, periyodik olarak target ağı senkronlar ve ağırlıkları
diske kaydeder.

ROS 2 notları:
  * rospy → rclpy.node.Node
  * get_param("~x") → declare_parameter + get_parameter (self._param)
  * SetModelState / DeleteLight servisleri: create_client + call_async + spin
  * rospy.sleep(...) (episode reset'te kısa oturma bekleyişi) → time.sleep(...)
    (bu bir eğitim node'u; kısa duvar-saati beklemeleri kabul edilebilir)
  * rospy.on_shutdown → main()'in finally bloğunda _on_shutdown çağrılır
  * feature_utils / nn artık reactive_robot_common paketinden import edilir
"""

import collections
import math
import os
import time
import random
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteLight, SetEntityState
from gazebo_msgs.msg import EntityState

from reactive_robot_common import feature_utils as fu
from reactive_robot_common.nn import MLP

# Ağırlıkların/logların varsayılan konumu: kullanıcı ev dizini (kaynak ağacına yazmamak için).
_DATA_DIR = os.path.expanduser("~/rover_ml_data/dqn")


class DqnFinetune(Node):

    def __init__(self):
        super().__init__("dqn_finetune")

        # ── Parametreler ──────────────────────────────────────────────────────
        self.init_weights     = self._param("init_weights", "")
        # ROS 1'de <param if="out_weights != ''"> ile boşsa hiç set edilmiyordu.
        # ROS 2'de parametre her zaman geçirilir; boş string'i "varsayılanı kullan"
        # olarak yorumluyoruz.
        _default_out_weights = os.path.join(_DATA_DIR, "weights", "dqn_weights.npz")
        _out_weights = self._param("out_weights", _default_out_weights)
        self.out_weights = _out_weights if _out_weights else _default_out_weights
        self.n_sectors        = int(self._param("n_sectors",     fu.N_SECTORS))
        self.max_vel          = float(self._param("max_vel",     0.26))
        self.max_angular      = float(self._param("max_angular", 1.82))

        # Robot spawn pozu (dünyaya özgü — başka haritalar için launch argümanı olarak geçin)
        self.spawn_x    = float(self._param("spawn_x",    0.0))
        self.spawn_y    = float(self._param("spawn_y",    9.5))
        self.spawn_z    = float(self._param("spawn_z",    0.0))
        self.spawn_yaw  = float(self._param("spawn_yaw", -1.57))
        self.model_name = self._param("model_name", "turtlebot3_burger_rgbd")

        # Episode ayarları
        self.episode_max_steps = int(self._param("episode_max_steps", 600))

        # Çarpışma tespiti (wedge-recovery eşiğini yansıtır)
        self.collision_dist        = float(self._param("collision_dist",       0.15))
        self.collision_consecutive = int(self._param("collision_consecutive",   3))
        self.clearance_target      = float(self._param("clearance_target",     0.5))

        # RL hiperparametreleri
        self.replay_capacity   = int(self._param("replay_capacity",  10000))
        self.batch_size        = int(self._param("batch_size",        64))
        self.gamma             = float(self._param("gamma",           0.99))
        self.lr                = float(self._param("lr",              5e-4))
        self.epsilon_start     = float(self._param("epsilon_start",   0.5))
        self.epsilon_end       = float(self._param("epsilon_end",     0.05))
        self.epsilon_decay_eps = int(self._param("epsilon_decay_eps", 200))
        self.update_freq       = int(self._param("update_freq",       4))
        self.target_sync_steps = int(self._param("target_sync_steps", 200))
        self.save_every_eps    = int(self._param("save_every_eps",    50))

        # Wedge backstop (ml_robot / fgm_dwa ile aynı)
        self.wedge_front_dist  = float(self._param("wedge_front_dist",  0.20))
        self.wedge_consecutive = int(self._param("wedge_consecutive",   3))
        self.recover_seconds   = float(self._param("recover_seconds",   0.7))
        self.recover_linear    = float(self._param("recover_linear",   -0.13))

        # ── Ağları yükle / oluştur ───────────────────────────────────────────
        if self.init_weights and os.path.isfile(self.init_weights):
            self.online_net = MLP.load(self.init_weights)
            self.get_logger().info(f"[DqnFinetune] Loaded BC init weights from {self.init_weights}")
        else:
            # init weights yoksa: varsayılan mimariyle rastgele başlat
            input_dim = self.n_sectors + len(fu.ACTIONS)
            self.online_net = MLP((input_dim, 64, 64, len(fu.ACTIONS)), seed=0)
            self.get_logger().warn("[DqnFinetune] No init_weights — starting from random init")

        # Target network: online net'in derin kopyası
        self.target_net = MLP(self.online_net.sizes, seed=1)
        for k in self.online_net.params:
            self.target_net.params[k] = self.online_net.params[k].copy()

        self._action_table = fu.action_table(self.max_vel, self.max_angular)

        # ── Replay buffer ────────────────────────────────────────────────
        # Her geçiş (transition): (feat, action_idx, reward, next_feat, done)
        self.replay = collections.deque(maxlen=self.replay_capacity)

        # ── Episode / adım sayaçları ────────────────────────────────────────
        self.global_step     = 0
        self.episode         = 0
        self.step_in_episode = 0
        self.episode_reward  = 0.0
        self.collision_count = 0
        self.wedge_count     = 0
        self.recover_until   = None

        self.prev_action_idx  = 2   # varsayılan FORWARD
        self.prev_feat        = None
        self.prev_action      = None

        # SetEntityState servis istemcisi (episode reset için).
        self._set_state_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        # ── Loglama ──────────────────────────────────────────────────────
        log_dir = os.path.join(_DATA_DIR, "logs")
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._logf = open(os.path.join(log_dir, f"dqn_{stamp}.log"), "w", buffering=1)
        self._log("DQN FINETUNE START | "
                  f"init={self.init_weights} episodes_decay={self.epsilon_decay_eps} "
                  f"batch={self.batch_size} gamma={self.gamma} lr={self.lr}")

        # ── ROS ───────────────────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.get_logger().info("[DqnFinetune] Ready — waiting for /scan …")

    # ── ROS 2 yardımcıları ─────────────────────────────────────────────────────

    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Loglama ──────────────────────────────────────────────────────────────

    def _log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._logf.write(f"[{ts}] {msg}\n")

    # ── Epsilon zamanlaması ──────────────────────────────────────────────────

    def _epsilon(self):
        frac = min(1.0, self.episode / max(1, self.epsilon_decay_eps))
        return self.epsilon_start + frac * (self.epsilon_end - self.epsilon_start)

    # ── Episode sıfırlama ────────────────────────────────────────────────────

    def _reset_episode(self):
        """Robotu spawn noktasına ışınla, hızı sıfırla, episode sayaçlarını sıfırla."""
        # Önce hızı sıfırla (kalıntı momentumu önler)
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

        try:
            if not self._set_state_cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("/gazebo/set_entity_state service unavailable")

            state = EntityState()
            state.name = self.model_name
            state.reference_frame = "world"

            state.pose.position.x = self.spawn_x
            state.pose.position.y = self.spawn_y
            state.pose.position.z = self.spawn_z

            # spawn_yaw'ı quaternion'a çevir (roll=0, pitch=0)
            half_yaw = self.spawn_yaw / 2.0
            state.pose.orientation.z = math.sin(half_yaw)
            state.pose.orientation.w = math.cos(half_yaw)

            # Kalıntı hızı önlemek için twist'i açıkça sıfırla
            # (Gazebo diff-drive eklentisi ışınlamada hızı her zaman sıfırlamaz)
            state.twist.linear.x  = 0.0
            state.twist.linear.y  = 0.0
            state.twist.linear.z  = 0.0
            state.twist.angular.x = 0.0
            state.twist.angular.y = 0.0
            state.twist.angular.z = 0.0

            req = SetEntityState.Request()
            req.state = state
            future = self._set_state_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        except Exception as exc:
            self.get_logger().warn(f"[DqnFinetune] set_entity_state failed: {exc}")
            self._log(f"WARNING set_entity_state: {exc}")

        time.sleep(0.2)
        self.cmd_pub.publish(Twist())   # ışınlama oturduktan sonra tekrar sıfırla

        self.step_in_episode = 0
        self.episode_reward  = 0.0
        self.collision_count = 0
        self.wedge_count     = 0
        self.recover_until   = None
        self.prev_feat       = None
        self.prev_action     = None
        self.prev_action_idx = 2   # varsayılan FORWARD
        self.episode        += 1
        self.get_logger().info(f"[DqnFinetune] === Episode {self.episode} start  "
                               f"epsilon={self._epsilon():.3f} ===")
        self._log(f"EPISODE {self.episode} START epsilon={self._epsilon():.3f}")

    # ── DQN güncellemesi ─────────────────────────────────────────────────────

    def _update(self):
        if len(self.replay) < self.batch_size:
            return

        batch = random.sample(self.replay, self.batch_size)
        feats, actions, rewards, next_feats, dones = zip(*batch)

        Xb      = np.array(feats,      dtype=np.float32)
        Xb_next = np.array(next_feats, dtype=np.float32)
        yb_act  = np.array(actions,    dtype=np.int64)
        rb      = np.array(rewards,    dtype=np.float32)
        db      = np.array(dones,      dtype=np.float32)

        # Mevcut durumlar için online Q-değerleri
        q_logits, cache = self.online_net.forward(Xb)

        # Hedef: r + gamma * max_a Q_target(s', a) * (1 - done)
        q_next, _ = self.target_net.forward(Xb_next)
        q_target_val = rb + self.gamma * q_next.max(axis=1) * (1.0 - db)

        # Pseudo-label oluştur: kaybı yalnızca seçilen aksiyon üzerinden
        # hesaplamak için one-hot ağırlıklı hedefler. "Yamalı" (patched) bir
        # logit matrisinin argmax'ı olan sentetik bir y oluşturuyoruz; böylece
        # cross-entropy backward yalnızca seçilen aksiyon üzerinde gradyan
        # üretir — o aksiyon için Q üzerinde MSE ile eşdeğer ama mevcut
        # backward() API'si içinde uygulanmış olur.
        q_patched = q_logits.copy()
        B = len(yb_act)
        for i in range(B):
            q_patched[i, yb_act[i]] = q_target_val[i]

        # Gradyanı hesapla: q_patched'i MSE için hedef logit olarak ele al
        # (y_true ile backward() CE içindir; burada doğrudan gradyan kullanıyoruz)
        dlogits = (q_logits - q_patched) / B     # MSE gradyanı
        grads = {}
        n_layers = cache["n_layers"]
        dZ = dlogits.astype(np.float32)
        for i in range(n_layers, 0, -1):
            A_prev = cache.get(f"A{i-1}", cache["X"])
            grads[f"W{i}"] = A_prev.T @ dZ
            grads[f"b{i}"] = dZ.sum(axis=0)
            if i > 1:
                dA = dZ @ self.online_net.params[f"W{i}"].T
                dZ = dA * (cache[f"Z{i-1}"] > 0).astype(np.float32)

        self.online_net.adam_step(grads, lr=self.lr)

    # ── Scan callback ─────────────────────────────────────────────────────────

    def _scan_callback(self, msg):
        now = self._now()
        n   = len(msg.ranges)
        cleaned = fu.clean_ranges(msg.ranges, msg.range_min, msg.range_max)

        # Ön mesafe (ml_robot ile aynı ±20 indeks yaklaşımı)
        front_idxs = [int(o * n / 360) % n for o in range(-20, 21)]
        front_min  = float(min(cleaned[i] for i in front_idxs))

        # ── Wedge backstop (ml_robot ile aynı) ───────────────────────────
        if self.recover_until is not None:
            if now < self.recover_until:
                cmd = Twist()
                cmd.linear.x = self.recover_linear
                self.cmd_pub.publish(cmd)
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
            cmd = Twist()
            cmd.linear.x = self.recover_linear
            self.cmd_pub.publish(cmd)
            return

        # ── Özellik vektörünü oluştur ────────────────────────────────────────
        sectors = fu.scan_to_sectors(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.n_sectors
        )
        feat = fu.build_feature_vector(
            sectors, self.prev_action_idx, n_actions=len(fu.ACTIONS)
        )

        # ── Önceki geçişten ödülü hesapla ───────────────────────────────────
        if self.prev_feat is not None:
            done   = False
            reward = 0.01   # hayatta kalma ödülü

            # Clearance bonusu
            clearance = front_min - self.clearance_target
            reward += 0.005 * max(0.0, clearance)

            # Çarpışma cezası
            if front_min < self.collision_dist:
                self.collision_count += 1
            else:
                self.collision_count = 0

            if self.collision_count >= 3:
                reward = -10.0
                done   = True
                self.get_logger().warn(f"[DqnFinetune] Collision! front_min={front_min:.2f}")
                self._log(f"COLLISION | ep={self.episode} step={self.step_in_episode}")

            self.episode_reward += reward
            self.replay.append(
                (self.prev_feat, self.prev_action, reward, feat, float(done))
            )

            if done or self.step_in_episode >= self.episode_max_steps:
                self.get_logger().info(
                    f"[DqnFinetune] Episode {self.episode} end  "
                    f"steps={self.step_in_episode} reward={self.episode_reward:.2f}"
                )
                self._log(f"EPISODE {self.episode} END steps={self.step_in_episode} "
                          f"reward={self.episode_reward:.2f}")

                if self.episode % self.save_every_eps == 0:
                    self._save_weights()

                self._reset_episode()
                return

        # ── Epsilon-greedy aksiyon seçimi ────────────────────────────────────
        if random.random() < self._epsilon():
            action_idx = random.randint(0, len(fu.ACTIONS) - 1)
        else:
            action_idx = self.online_net.predict_one(feat)

        name, v, w = self._action_table[action_idx]

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        # ── DQN güncellemesi ─────────────────────────────────────────────
        self.global_step += 1
        if self.global_step % self.update_freq == 0:
            self._update()

        if self.global_step % self.target_sync_steps == 0:
            for k in self.online_net.params:
                self.target_net.params[k] = self.online_net.params[k].copy()
            self.get_logger().info(f"[DqnFinetune] Target net synced at step {self.global_step}")

        self.prev_feat       = feat
        self.prev_action     = action_idx
        self.prev_action_idx = action_idx
        self.step_in_episode += 1

    # ── Kalıcılık (persistence) ──────────────────────────────────────────────

    def _save_weights(self):
        os.makedirs(os.path.dirname(self.out_weights), exist_ok=True)
        self.online_net.save(self.out_weights)
        self.get_logger().info(f"[DqnFinetune] Weights saved → {self.out_weights}")
        self._log(f"WEIGHTS SAVED | ep={self.episode} step={self.global_step}")

    # ── Kapanış (ROS 1 on_shutdown karşılığı; main finally'den çağrılır) ──────

    def on_shutdown(self):
        self.cmd_pub.publish(Twist())
        self._save_weights()

        try:
            self._delete_sun_light()
        except Exception as exc:
            self._log(f"WARNING delete_light: {exc}")

        self._log(f"DQN FINETUNE END | total_episodes={self.episode} "
                  f"total_steps={self.global_step}")
        self._logf.close()

    def _delete_sun_light(self):
        client = self.create_client(DeleteLight, "/gazebo/delete_light")
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/gazebo/delete_light service unavailable")
        req = DeleteLight.Request()
        req.light_name = "sun"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self._log("Deleted 'sun' light")
        else:
            raise RuntimeError("delete_light call returned no result")


def main(args=None):
    rclpy.init(args=args)
    node = DqnFinetune()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 1 on_shutdown hook'unun karşılığı: ağırlıkları kaydet, ışığı sil.
        try:
            node.on_shutdown()
        except Exception as exc:
            node.get_logger().error(f"[DqnFinetune] shutdown failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
