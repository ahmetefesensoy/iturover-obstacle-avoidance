#!/usr/bin/env python3
"""
barn_goal_publisher.py — BARN dünyaları için waypoint tabanlı navigasyon.

path_N.npy dosyasından ara waypoint'ler örnekler, böylece TEB tüm dolambaçlı
labirenti tek seferde değil kısa segmentler (~0.75m) halinde planlar.

Robot pozisyonu MAP frame'inde TF üzerinden takip edilir (ham odom değil;
odom robotun nerede spawn olduğuna bakmaksızın (0,0)'dan başlar).

Son waypoint'e ulaşıldığında: /tmp/barn_goal_reached dosyasını yazar ve
/goal_reached True yayınlar -> shutdown_timer roslaunch kapanışını tetikler.
"""
import rospy, actionlib, os, math
import numpy as np
import tf2_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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


class WaypointNavigator:
    def __init__(self):
        rospy.init_node('barn_goal_publisher')

        barn_dir  = rospy.get_param('~barn_dir', '/root/BARN_dataset/BARN_dataset')
        world_idx = int(rospy.get_param('~world_idx', 0))
        self._done = False

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Waypoint listesini oluştur — path[0] (spawn hücresi) atlanır, 1. adımdan başlanır
        path_file = os.path.join(barn_dir, 'path_files', f'path_{world_idx}.npy')
        if os.path.exists(path_file):
            path = np.load(path_file)
            indices = list(range(WAYPOINT_STEP, len(path), WAYPOINT_STEP))
            if not indices or indices[-1] != len(path) - 1:
                indices.append(len(path) - 1)
            self.waypoints = [grid_to_world(path[i][0], path[i][1]) for i in indices]
            rospy.loginfo(f'[goal_pub] world_{world_idx}: {len(self.waypoints)} waypoints')
        else:
            self.waypoints = [(-0.08, 2.77)]
            rospy.logwarn('[goal_pub] path file not found, using single default goal')

        self.wp_idx = 0
        self.robot_x = None
        self.robot_y = None
        self.last_move_time = rospy.Time(0)
        self.prev_x = None
        self.prev_y = None

        self.goal_pub = rospy.Publisher('/goal_reached', Bool, queue_size=1, latch=True)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('[goal_pub] Waiting for move_base...')
        if not self.client.wait_for_server(timeout=rospy.Duration(40.0)):
            rospy.logerr('[goal_pub] move_base not available')
            return

        # TF map->base_footprint kullanılabilir olana kadar bekle
        rospy.loginfo('[goal_pub] Waiting for map->base_footprint TF...')
        for _ in range(40):
            try:
                self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rospy.Time(0), rospy.Duration(1.0))
                rospy.loginfo('[goal_pub] TF ready')
                break
            except Exception:
                rospy.sleep(0.5)

        rospy.sleep(2.0)  # costmap'in oturması için ek bekleme süresi
        self._update_robot_pose()
        if self.robot_x is not None:
            rospy.loginfo(f'[goal_pub] robot in map: ({self.robot_x:.3f},{self.robot_y:.3f})')

        self._send_waypoint()
        rospy.Timer(rospy.Duration(3.0), self._check_progress)
        rospy.spin()

    # Robot pozunu TF üzerinden MAP frame'inde al
    def _update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                rospy.Time(0), rospy.Duration(0.5))
            x = t.transform.translation.x
            y = t.transform.translation.y
            if self.prev_x is not None:
                if math.hypot(x - self.prev_x, y - self.prev_y) > 0.04:
                    self.last_move_time = rospy.Time.now()
            else:
                self.last_move_time = rospy.Time.now()
            self.prev_x, self.prev_y = x, y
            self.robot_x, self.robot_y = x, y
        except Exception:
            pass  # TF hatasında önceki değerleri koru

    def _send_waypoint(self):
        if self.wp_idx >= len(self.waypoints):
            return
        gx, gy = self.waypoints[self.wp_idx]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        rospy.loginfo(f'[goal_pub] wp {self.wp_idx+1}/{len(self.waypoints)}: ({gx:.2f},{gy:.2f})')

    def _check_progress(self, event):
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
                rospy.loginfo('[goal_pub] *** KURS TAMAMLANDI! ***')
                self._done = True
                with open(SUCCESS_MARKER, 'w') as f:
                    f.write('SUCCESS\n')
                self.goal_pub.publish(Bool(True))
            else:
                self.last_move_time = rospy.Time.now()
                self._send_waypoint()
            return

        # Takılma kontrolü
        if self.last_move_time.to_sec() > 0:
            stuck_secs = (rospy.Time.now() - self.last_move_time).to_sec()
            if stuck_secs > STUCK_TIMEOUT:
                rospy.logwarn(f'[goal_pub] {stuck_secs:.0f}s takilda - wp yeniden gonderiliyor')
                self.client.cancel_all_goals()
                rospy.sleep(0.5)
                self._send_waypoint()
                self.last_move_time = rospy.Time.now()


if __name__ == '__main__':
    WaypointNavigator()
