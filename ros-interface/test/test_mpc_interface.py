#!/usr/bin/env python

# import sys
import unittest
import rospy
import numpy as np
from std_msgs.msg import Header
from linear_feedback_controller_msgs.msg import Sensor, Control
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from time import sleep

PKG = "ros-interface"
NAME = "MPC_Interface_test"


class TestMPCInterface(unittest.TestCase):
    # def __init__(self, *args):
    #     super(TestMPCInterface, self).__init__(*args)

    # test 1 == 1
    def test_one_equals_one(self):
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")

    def test_message_reception(self):
        rospy.init_node(NAME, anonymous=True)

        self.sensorData = self._define_sensor_msg()

        pub = rospy.Publisher("sensor_state", Sensor)
        pub.publish(self.sensorData)

        msg = rospy.wait_for_message("command", Control, timeout=10)
        self.assertEquals(
            msg.InitialState,
            self.sensorData,
            "Received initial state should be equal to the one sent",
        )

    def _define_sensor_msg(self):
        controlled_joints = rospy.get_param("controlled_joints")

        sensorData = Sensor(
            base_pose=Pose(
                position=Point(x=0, y=0, z=0),
                orientation=Quaternion(x=0, y=0, z=0, w=1),
            ),
            base_twist=Twist(
                linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)
            ),
            joint_state=JointState(
                name=controlled_joints,
                position=np.zeros(len(controlled_joints)),
                velocity=np.zeros(len(controlled_joints)),
            ),
        )

        return sensorData


if __name__ == "__main__":
    # import rostest
    # rostest.rosrun(PKG, NAME, TestMPCInterface)

    rospy.init_node(NAME, anonymous=True)

    controlled_joints = rospy.get_param("controlled_joints")

    sensorData = Sensor(
        header=Header(stamp=rospy.get_rostime()),
        base_pose=Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        ),
        base_twist=Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)),
        joint_state=JointState(
            name=controlled_joints,
            position=np.ones(len(controlled_joints)),
            velocity=np.zeros(len(controlled_joints)),
        ),
    )

    pub = rospy.Publisher("sensor_state", Sensor, queue_size=10)
    for _ in range(10):
        print("Publishing empty sensor state")
        pub.publish(sensorData)
        sleep(1)
