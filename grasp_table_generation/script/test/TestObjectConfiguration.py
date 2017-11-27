#!/usr/bin/env python

PKG = 'grasp_table_generation'
Name = 'test_object_configuration'

import sys
import unittest
import rostest

import threading
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy


class TestObjectConfiguration(unittest.TestCase):
    def setUp(self):
        self.initTFListener()
        self.initTestPoint()

    def initTFListener(self):
        with threading.Lock():
            self.listener = tf.TransformListener(True, rospy.Duration(40.0))


    def initTestPoint(self):
        self.testPoint = PoseStamped()
        self.testPoint.header.frame_id = "/object3DCenter"
        self.testPoint.header.seq = 0
        self.testPoint.header.stamp = rospy.Time(0)
        self.testPoint.pose.position.x = 1
        self.testPoint.pose.position.y = 1
        self.testPoint.pose.position.z = 1
        self.testPoint.pose.orientation.w = 1
        self.testPoint.pose.orientation.x = 0
        self.testPoint.pose.orientation.y = 0
        self.testPoint.pose.orientation.z = 0


    def test_TransformationXY(self):
        self.listener.waitForTransform('/object2Dxy', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointXY = self.listener.transformPose('/object2Dxy', self.testPoint)
        trans = [pointXY.pose.position.x, pointXY.pose.position.y, pointXY.pose.position.z]
        #rot = [pointXY.pose.orientation.w, pointXY.pose.orientation.x, pointXY.pose.orientation.y, pointXY.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 3, 1])

    def test_TransformationXZ(self):
        self.listener.waitForTransform('/object2Dxz', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointXZ = self.listener.transformPose('/object2Dxz', self.testPoint)
        trans = [pointXZ.pose.position.x, pointXZ.pose.position.y, pointXZ.pose.position.z]
        #rot = [pointXZ.pose.orientation.w, pointXZ.pose.orientation.x, pointXZ.pose.orientation.y, pointXZ.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 4, -1])

    def test_TransformationYX(self):
        self.listener.waitForTransform('/object2Dyx', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointYX = self.listener.transformPose('/object2Dyx', self.testPoint)
        trans = [pointYX.pose.position.x, pointYX.pose.position.y, pointYX.pose.position.z]
        #rot = [pointYX.pose.orientation.w, pointYX.pose.orientation.x, pointYX.pose.orientation.y, pointYX.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 2, -1])

    def test_TransformationYZ(self):
        self.listener.waitForTransform('/object2Dyz', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointYZ = self.listener.transformPose('/object2Dyz', self.testPoint)
        trans = [pointYZ.pose.position.x, pointYZ.pose.position.y, pointYZ.pose.position.z]
        #rot = [pointYZ.pose.orientation.w, pointYZ.pose.orientation.x, pointYZ.pose.orientation.y, pointYZ.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 4, 1])

    def test_TransformationZX(self):
        self.listener.waitForTransform('/object2Dzx', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointZX = self.listener.transformPose('/object2Dzx', self.testPoint)
        trans = [pointZX.pose.position.x, pointZX.pose.position.y, pointZX.pose.position.z]
        #rot = [pointZX.pose.orientation.w, pointZX.pose.orientation.x, pointZX.pose.orientation.y, pointZX.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 2, 1])

    def test_TransformationZY(self):
        self.listener.waitForTransform('/object2Dzy', "/object3DCenter", rospy.Time(0), rospy.Duration(10))
        pointZY = self.listener.transformPose('/object2Dzy', self.testPoint)
        trans = [pointZY.pose.position.x, pointZY.pose.position.y, pointZY.pose.position.z]
        #rot = [pointZY.pose.orientation.w, pointZY.pose.orientation.x, pointZY.pose.orientation.y, pointZY.pose.orientation.z]
        pointTrans = [round(trans[i], 2) for i in range(0, len(trans))]
        self.assertEqual(pointTrans, [1, 3, -1])


if __name__ == "__main__":
    rospy.init_node('test_object_configuration')
    rostest.rosrun(PKG, Name, TestObjectConfiguration)
