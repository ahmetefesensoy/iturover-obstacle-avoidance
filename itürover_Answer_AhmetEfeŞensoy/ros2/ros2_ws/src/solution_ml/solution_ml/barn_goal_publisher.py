#!/usr/bin/env python3
"""
barn_goal_publisher.py — BARN dünyaları için waypoint tabanlı navigasyon (ROS 2 / Nav2).

path_N.npy dosyasından ara waypoint'ler örnekler, böylece yerel planlayıcı tüm
dolambaçlı labirenti tek seferde değil kısa segmentler (~0.75m) halinde planlar.

Robot pozisyonu MAP frame'inde TF üzerinden takip edilir (ham odom değil;
odom robotun nerede spawn olduğuna bakmaksızın (0,0)'dan başlar).

Son waypoint'e ulaşıldığında: /tmp/barn_goal_reached dosyasını yazar ve
/goal_reached True yayınlar -> shutdown_timer kapanışı tetikler.

ROS 2 / Nav2 notları:
  * actionlib.SimpleActionClient('move_base', MoveBaseAction)
      → rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
  * MoveBaseGoal().target_pose → NavigateToPose.Goal().pose (PoseStamped)
  * client.send_goal(goal) → client.send_goal_async(goal)
  * tf2_ros.Buffer()/TransformListener(buf) → Buffer() + TransformListener(buf, self)
  * rospy.Timer(Duration(3), cb) → self.create_timer(3.0, cb)  (event arg'ı yok)
  * latched publisher → QoS TRANSIENT_LOCAL durability
  * Bool(True) → Bool(data=True)
"""

import os
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

ORIGIN_X   = -4.5
ORIGIN_Y   =  0.0
RESOLUTION =  0.15
SUCCESS_MARKER = "/tmp/barn_goal_reached"
WAYPOINT_STEP  = 5    # waypoint'ler arasındaki path hücre sayısı (~0.75m aralık)
WAYPOINT_TOL   = 0.40 # waypoint'e ulaşıldı sayılacak mesafe (m)
STUCK_TIMEOUT  = 20.0 # hareketsiz geçen süre (sn) -> mevcut waypoint'i yeniden gönder


def grid_to_world(row, col):
    x = ORIGIN_X + col * RESOLUTION + RESOLUTION / 2.0
    y = ORIGIN_Y + row * RESOLUTION + RESOLUTION / 2.0
    return x, y


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('barn_goal_publisher')

        self.declare_parameter('barn_dir', '/root/BARN_dataset/BARN_dataset')
        self.declare_parameter('world_idx', 0)
        barn_dir  = self.get_parameter('barn_dir').value
        world_idx = int(self.get_parameter('world_idx').value)
        self._done = False

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Waypoint listesini oluştur — path[0] (spawn hücresi) atlanır
        path_file = os.path.join(barn_dir, 'path_files', f'path_{world_idx}.npy')
        if os.path.exists(path_file):
            path = np.load(path_file)
            indices = list(range(WAYPOINT_STEP, len(path), WAYPOINT_STEP))
            if not indices or indices[-1] != len(path) - 1:
                indices.append(len(path) - 1)
            self.waypoints = [grid_to_world(path[i][0], path[i][1]) for i in indices]
            self.get_logger().info(f'[goal_pub] world_{world_idx}: {len(self.waypoints)} waypoints')
        else:
            self.waypoints = [(-0.08, 2.77)]
            self.get_logger().warn('[goal_pub] path file not found, using single default goal')

        self.wp_idx = 0
        self.robot_x = None
        self.robot_y = None
        self.last_move_time = self._now()
        self.prev_x = None
        self.prev_y = None

        # Latched /goal_reached yayıncısı (TRANSIENT_LOCAL = ROS 1 latch=True).
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.goal_pub = self.create_publisher(Bool, '/goal_reached', latched_qos)

        # Nav2 NavigateToPose action istemcisi (ROS 1 move_base karşılığı).
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('[goal_pub] Waiting for navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=40.0):
            self.get_logger().error('[goal_pub] navigate_to_pose not available')
            return

        # TF map->base_footprint kullanılabilir olana kadar bekle
        self.get_logger().info('[goal_pub] Waiting for map->base_footprint TF...')
        for _ in range(40):
            try:
                self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(),
                    timeout=Duration(seconds=1.0))
                self.get_logger().info('[goal_pub] TF ready')
                break
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.5)

        self._sleep(2.0)  # costmap'in oturması için ek bekleme süresi
        self._update_robot_pose()
        if self.robot_x is not None:
            self.get_logger().info(f'[goal_pub] robot in map: ({self.robot_x:.3f},{self.robot_y:.3f})')

        self._send_waypoint()
        # İlerleme kontrolü için periyodik zamanlayıcı (ROS 1 rospy.Timer karşılığı).
        self.create_timer(3.0, self._check_progress)

    # ── ROS 2 yardımcıları ─────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sleep(self, sec: float):
        """Kısa, spin yapan bekleme (TF/costmap otursun diye)."""
        end = self._now() + sec
        while self._now() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── Robot pozunu TF üzerinden MAP frame'inde al ────────────────────────────

    def _update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            x = t.transform.translation.x
            y = t.transform.translation.y
            if self.prev_x is not None:
                if math.hypot(x - self.prev_x, y - self.prev_y) > 0.04:
                    self.last_move_time = self._now()
            else:
                self.last_move_time = self._now()
            self.prev_x, self.prev_y = x, y
            self.robot_x, self.robot_y = x, y
        except Exception:
            pass  # TF hatasında önceki değerleri koru

    def _send_waypoint(self):
        if self.wp_idx >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.wp_idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0
        self.client.send_goal_async(goal)
        self.get_logger().info(f'[goal_pub] wp {self.wp_idx+1}/{len(self.waypoints)}: ({gx:.2f},{gy:.2f})')

    def _check_progress(self):
        if self._done or self.wp_idx >= len(self.waypoints):
            return

        self._update_robot_pose()
        if self.robot_x is None:
            return

        gx, gy = self.waypoints[self.wp_idx]
        dist = math.hypot(self.robot_x - gx, self.robot_y - gy)

        if dist < WAYPOINT_TOL:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.get_logger().info('[goal_pub] *** KURS TAMAMLANDI! ***')
                self._done = True
                with open(SUCCESS_MARKER, 'w') as f:
                    f.write('SUCCESS\n')
                self.goal_pub.publish(Bool(data=True))
            else:
                self.last_move_time = self._now()
                self._send_waypoint()
            return

        # Takılma kontrolü: uzun süre ilerleme yoksa mevcut waypoint'i yeniden gönder
        stuck_secs = self._now() - self.last_move_time
        if stuck_secs > STUCK_TIMEOUT:
            self.get_logger().warn(f'[goal_pub] {stuck_secs:.0f}s takilda - wp yeniden gonderiliyor')
            self._send_waypoint()
            self.last_move_time = self._now()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
