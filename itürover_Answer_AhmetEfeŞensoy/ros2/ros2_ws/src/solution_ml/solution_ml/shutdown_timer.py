#!/usr/bin/env python3
"""shutdown_timer.py — Güvenlik zaman aşımı VE goal-reached dinleyicisi (ROS 2).

/goal_reached True alınırsa erken çıkar (robot parkuru tamamladı demektir),
yoksa `duration` saniye sonunda kendini kapatır.

ROS 1'de `required=true` node kapanınca roslaunch her şeyi öldürüyordu. ROS 2'de
node kendini kapattığında rclpy.shutdown() çağrılır; diğer node'ların
on_exit/OnProcessExit ile birlikte kapanması launch dosyasında ayarlanır.

ROS 2 notları:
  * modül seviyesindeki rospy.sleep(duration) → create_timer(duration, cb)
    (spin ile eşzamanlı uyunamayacağı için tek-atışlık zamanlayıcı kullanılır)
  * rospy.signal_shutdown → rclpy.shutdown()
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ShutdownTimer(Node):
    def __init__(self):
        super().__init__('shutdown_timer')
        self.declare_parameter('duration', 120.0)
        self.duration = float(self.get_parameter('duration').value)
        self._fired = False

        self.create_subscription(Bool, '/goal_reached', self._goal_cb, 1)
        # Tek-atışlık güvenlik zamanlayıcısı (rospy.sleep yerine).
        self._timer = self.create_timer(self.duration, self._on_timeout)
        self.get_logger().info(f'[shutdown_timer] {self.duration:.0f}s safety limit started')

    def _goal_cb(self, msg: Bool):
        if msg.data and not self._fired:
            self._fired = True
            self.get_logger().info('[shutdown_timer] Goal reached — triggering shutdown')
            rclpy.shutdown()

    def _on_timeout(self):
        if self._fired:
            return
        self._fired = True
        self.get_logger().info('[shutdown_timer] Time limit reached (goal not reached)')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ShutdownTimer()
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
