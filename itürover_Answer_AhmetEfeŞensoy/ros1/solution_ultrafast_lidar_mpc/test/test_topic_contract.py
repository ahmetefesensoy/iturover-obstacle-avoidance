#!/usr/bin/env python3
import time
import unittest

import rosgraph
import rospy
import rostest


class ProductionTopicContract(unittest.TestCase):
    def test_runtime_graph_is_lidar_only(self):
        master = rosgraph.Master(rospy.get_name())
        deadline = time.time() + 10.0
        subscriptions = set()
        publications = set()
        while time.time() < deadline:
            publishers, subscribers, _ = master.getSystemState()
            subscriptions = {
                topic for topic, nodes in subscribers
                if "/ultrafast_lidar_mpc" in nodes
            }
            publications = {
                topic for topic, nodes in publishers
                if "/ultrafast_lidar_mpc" in nodes
            }
            if "/scan" in subscriptions and "/cmd_vel" in publications:
                break
            time.sleep(0.05)

        self.assertEqual(subscriptions, {"/scan"})
        self.assertIn("/cmd_vel", publications)
        self.assertIn("/ultrafast_lidar_mpc/telemetry", publications)


if __name__ == "__main__":
    rospy.init_node("production_topic_contract_test")
    rostest.rosrun("ultrafast_lidar_mpc", "production_topic_contract",
                   ProductionTopicContract)
