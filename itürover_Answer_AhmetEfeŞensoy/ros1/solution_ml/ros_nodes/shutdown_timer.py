#!/usr/bin/env python3
"""shutdown_timer.py — Güvenlik zaman aşımı VE goal-reached dinleyicisi.
launch içinde required=true → bu node kapandığında roslaunch her şeyi öldürür.
/goal_reached True alınırsa erken çıkar (robot parkuru tamamladı demektir).
"""
import rospy
from std_msgs.msg import Bool

rospy.init_node('shutdown_timer')
duration = float(rospy.get_param('~duration', 120.0))

def _goal_cb(msg):
    if msg.data:
        rospy.loginfo('[shutdown_timer] Goal reached — triggering shutdown')
        rospy.signal_shutdown('goal_reached')

rospy.Subscriber('/goal_reached', Bool, _goal_cb)
rospy.loginfo(f'[shutdown_timer] {duration:.0f}s safety limit started')
rospy.sleep(duration)
rospy.loginfo('[shutdown_timer] Time limit reached (goal not reached)')
