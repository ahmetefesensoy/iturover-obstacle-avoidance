#!/usr/bin/env python3
"""odom_to_tf.py — /odom'u odom→base_footprint TF olarak yeniden yayınlar.
turtlebot3_burger_rgbd.gazebo.xacro'da publishOdomTF=false olduğu için gereklidir."""
import rospy, tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

rospy.init_node('odom_to_tf')
br = tf2_ros.TransformBroadcaster()

def cb(msg):
    t = TransformStamped()
    t.header.stamp    = msg.header.stamp
    t.header.frame_id = 'odom'
    t.child_frame_id  = 'base_footprint'
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation      = msg.pose.pose.orientation
    br.sendTransform(t)

rospy.Subscriber('/odom', Odometry, cb)
rospy.spin()
