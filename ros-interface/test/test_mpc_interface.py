#!/usr/bin/env python

import sys
import unittest

import rospy
from linear__feedback_controller_msgs import Control, Sensor

PKG = "ros-interface"
NAME = "MPC_Interface_test"


class TestMPCInterface(unittest.TestCase):
    # def __init__(self, *args):
    #     super(TestMPCInterface, self).__init__(*args)

    #     self.sensorData = Sensor

    # test 1 == 1
    def test_one_equals_one(self):
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")

    # def test_message_reception(self):
    #     rospy.init_node(NAME, anonymous=True)
    #     pub = rospy.Publisher("/sensor_state", Sensor)
    #     pub.publish(self.sensorData)
    #     msg = rospy.wait_for_message("command", Control)
    #     assert (
    #         msg.InitialState == self.sensorData
    #     ), "Received initial state should be equal to the one sent"
    #     self.success = True


if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestMPCInterface)
