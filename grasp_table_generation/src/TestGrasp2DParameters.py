#!/usr/bin/env python

PKG = 'grasp_table_generation'
Name = 'test_grasp_2D_parameter'

import sys
import unittest
import rostest

import math
import threading
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
from grasp_2D_parameters import grasp_2D_parameters

class TestGrasp2DParameters(unittest.TestCase):
    def setUp(self):
        self.grasp2DParameters = grasp_2D_parameters(112, 30)
        self.line1 = [[-1, -1], [2, 2]]
        self.line2 = [[1, 1], [4, 4]]
        self.pointOnLine = [3, 3]
        self.pointNotOnLine = [3, 6]
        self.PointOnLineSegment = [0, 1]

    def test_getDistanceFromPointToLine(self):
        dist = self.grasp2DParameters.getDistanceFromPointToLine(self.PointOnLineSegment, self.line1)
        distReal = math.sqrt(2)/2
        dist = round(dist, 2)
        distReal = round(distReal, 2)
        self.assertEqual(dist, distReal)

    def test_isPointOnLine(self):
        flag1 = self.grasp2DParameters.isPointOnLine(self.PointOnLineSegment, self.line1)
        self.assertTrue(flag1)
        flag2 = self.grasp2DParameters.isPointOnLine(self.pointNotOnLine, self.line1)
        self.assertFalse(flag2)

    def test_isPointOnLineSegment(self):
        flag1 = self.grasp2DParameters.isPointOnLineSegment(self.PointOnLineSegment, self.line1)
        self.assertTrue(flag1)
        flag2 = self.grasp2DParameters.isPointOnLineSegment(self.pointOnLine, self.line1)
        self.assertFalse(flag2)
        flag3 = self.grasp2DParameters.isPointOnLineSegment(self.pointNotOnLine, self.line1)
        self.assertFalse(flag3)

    def test_isLinesCoincide(self):
        flag1 = self.grasp2DParameters.isLinesCoincide(self.line2, self.line1)
        self.assertTrue(flag1)

    def test_getCoincideLineLength(self):
        dist = self.grasp2DParameters.getCoincideLineLength(self.line2, self.line1)
        dist = round(dist, 2)
        distReal = round(math.sqrt(2), 2)
        self.assertEqual(dist, distReal)

    def test_getDistanceBetweenPoints(self):
        dist = self.grasp2DParameters.getDistanceBetweenPoints([1, 1], [2, 2])
        dist = round(dist, 2)
        distReal = round(math.sqrt(2), 2)
        self.assertEqual(dist, distReal)

    def test_getPointWithLineSegment(self):
        pointStart = [1, 1]
        length = math.sqrt(2)
        angle = math.pi*3/4
        point = self.grasp2DParameters.getPointWithLineSegment(pointStart, length, angle)
        pointEnd = [round(point[i], 2) for i in range(0, len(point), 1)]
        self.assertEqual(pointEnd, [0, 2])



if __name__ == "__main__":
    rospy.init_node('test_grasp_2D_parameter')
    rostest.rosrun(PKG, Name, TestGrasp2DParameters)
