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
        #rospy.init_node('test_object_configuration')
        self.initTFListener()
        self.initTestPoint()

    def initTFListener(self):
        with threading.Lock():
            self.listener = tf.TransformListener(True, rospy.Duration(40.0))


    def initTestPoint(self):
        self.testPoint = PoseStamped()
        self.testPoint.header.frame_id = "object3DCenter"
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
        pointXY = self.listener.transformPose('objeccct2DXY', self.testPoint)
        pointTrans = [pointXY.pose.position.x, pointXY.pose.position.y, pointXY.pose.position.z]
        pointRot = [pointXY.pose.orientation.w, pointXY.pose.orientation.x, pointXY.pose.orientation.y, pointXY.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 3, 1])

    def test_TransformationXZ(self):
        pointXZ = self.listener.transformPose('objeccct2DXZ', self.testPoint)
        pointTrans = [pointXZ.pose.position.x, pointXZ.pose.position.y, pointXZ.pose.position.z]
        pointRot = [pointXZ.pose.orientation.w, pointXZ.pose.orientation.x, pointXZ.pose.orientation.y, pointXZ.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 4, -1])

    def test_TransformationYX(self):
        pointYX = self.listener.transformPose('objeccct2DYX', self.testPoint)
        pointTrans = [pointYX.pose.position.x, pointYX.pose.position.y, pointYX.pose.position.z]
        pointRot = [pointYX.pose.orientation.w, pointYX.pose.orientation.x, pointYX.pose.orientation.y, pointYX.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 2, -1])

    def test_TransformationYZ(self):
        pointYZ = self.listener.transformPose('objeccct2DYZ', self.testPoint)
        pointTrans = [pointYZ.pose.position.x, pointYZ.pose.position.y, pointYZ.pose.position.z]
        pointRot = [pointYZ.pose.orientation.w, pointYZ.pose.orientation.x, pointYZ.pose.orientation.y, pointYZ.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 4, 1])

    def test_TransformationZX(self):
        pointZX = self.listener.transformPose('objeccct2DZXX', self.testPoint)
        pointTrans = [pointZX.pose.position.x, pointZX.pose.position.y, pointZX.pose.position.z]
        pointRot = [pointZX.pose.orientation.w, pointZX.pose.orientation.x, pointZX.pose.orientation.y, pointZX.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 2, 1])

    def test_TransformationZY(self):
        pointZY = self.listener.transformPose('objeccct2DZY', self.testPoint)
        pointTrans = [pointZY.pose.position.x, pointZY.pose.position.y, pointZY.pose.position.z]
        pointRot = [pointZY.pose.orientation.w, pointZY.pose.orientation.x, pointZY.pose.orientation.y, pointZY.pose.orientation.z]
        self.assertEqual(pointTrans, [1, 3, -1])


if __name__ == "__main__":
    rostest.rosrun(PKG, Name, TestObjectConfiguration)
