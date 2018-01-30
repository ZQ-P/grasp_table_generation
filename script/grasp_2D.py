#!/usr/bin/env python

import rospy
import math
from gripper_shape import gripper_shape
from ObjectConfiguration import ObjectConfiguration


class grasp_2D(gripper_shape):
    def __init__(self, objectX, objectY):
        """initialize the parameters, arguments"""
        gripper_shape.__init__(self)
        self.initGrasp2DParameters()
        self.initObject2D(objectX, objectY)

    def initGrasp2DParameters(self):
        """initialize and declare parameters of gripper"""
        self.gripperAngleMove = {'start': 0, 'end': 0}
        self.gripperOriginalPoint = {'position': [0, 0], 'orientation': math.pi/2}
        self.existence = False

    def initObject2D(self, objectX, objectY):
        """initialize parameters of object"""
        self.objectShape = {'x': objectX, 'y': objectY}

    def getGraspParameters(self):
        self.calculateGraspParameters()
        #print self.gripperOriginalPoint
        #print self.gripperAngleMove
        return [self.gripperOriginalPoint, self.gripperAngleMove]

    def calculateGraspParameters(self):
        L = self.objectShape['y']
        L3 = self.gripperLengthMove['between']
        L4 = self.gripperLengthMove['start']
        L5 = self.gripperLengthFest['start']
        L6 = self.gripperLengthFest['between']
        theta4 = self.gripperAngleFest['start']
        theta5 = self.gripperAngleFest['end']
        theta3 = -theta5 - theta4
        x = L6*math.cos(math.pi/2 + theta5) + L5*math.sin(theta3) - L/2
        flag = L/2+L4*math.sin(theta3)-x
        if math.fabs(flag) > L3:
            self.existence = False
            rospy.logwarn("this side is too big to grasp.")
            return
        self.existence = True
        theta1 = math.pi/2
        theta2 = math.acos(flag/L3)
        y1 = -(L4*math.cos(theta3) + L3*math.sin(theta2) + 10)
        y2 = -(L6*math.sin(math.pi/2+theta5) + L5*math.cos(theta3) + 10)
        y = (y2, y1)[y1<y2]
        xOffset = self.gripperOffset * math.cos(theta3)
        yOffset = self.gripperOffset * math.sin(theta3)
        print x, y
        self.gripperAngleMove['start'] = theta2
        self.gripperAngleMove['end'] = theta1
        self.gripperOriginalPoint['position'] = [x-xOffset, y-yOffset]
        self.gripperOriginalPoint['orientation'] = theta3

if __name__ == "__main__":
    param = grasp_2D(90, 60)
    param.getGraspParameters()
