#!/usr/bin/python

#import rospy

class gripper_shape():
    def __init__(self):
        self.initGripperLengthFest()
        self.initGripperAngleFest()
        self.initGripperLengthMove()

    def initGripperLengthFest(self):
        gripperLengthFest1 = "~gripperLength1"
        gripperLengthFest2 = "~gripperLength2"
        gripperLengthFest3 = "~gripperLength3"
        self.gripperLengthFest = {'startLength': gripperLengthFest1, 'betweenLenth': gripperLengthFest2, 'endLength': gripperLengthFest3}

    def initGripperAngleFest(self):
        gripperAngleFestL12 = "~gripperAngleL12"
        gripperAngleFestL23 = "~gripperAngleL23"
        self.gripperAngleFest = {'startAngle': gripperAngleFestL12, 'endAngle':  gripperAngleFestL23}

    def initGripperLengthMove(self):
        gripperLengthMove1 = "~gripperFingerLength1"
        gripperLengthMove2 = "~gripperFingerLength2"
        gripperLengthMove3 = "~gripperFingerLength3"
        self.gripperLengthMove = {'startLength': gripperLengthMove1, 'betweenLength': gripperLengthMove2, 'endLength': gripperLengthMove3}

    def initGripperWithMove(self):
        self.gripperWidthMove = 1.5
    