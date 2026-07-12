#!/usr/bin/env python3
"""odom_to_tf.py — /odom'u odom→base_footprint TF olarak yeniden yayınlar (ROS 2).

turtlebot3_burger_rgbd.gazebo.xacro'da publish_odom_tf=false olduğu için gereklidir.

ROS 2 notları:
  * rospy → rclpy.node.Node
  * tf2_ros.TransformBroadcaster() → TransformBroadcaster(node)
  * modül seviyesindeki script yapısı main() + rclpy.spin() ile değiştirildi
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation      = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
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
