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
        rospy.init_node('test_object_configuration')
        with threading.Lock():
            self.listener=tf.TransformListener(True, rospy.Duration(40.0))
        self.initTestPoint()
    
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
        self.listener.waitForTransform()
        pass

    def test_TransformationXZ(self):
        pass

    def test_TransformationYX(self):
        pass

    def test_TransformationYZ(self):
        pass

    def test_TransformationZX(self):
        pass

    def test_TransformationZY(self):
        pass

