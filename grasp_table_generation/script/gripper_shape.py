#!/usr/bin/env python

import rospy

class gripper_shape():
    def __init__(self):
        self.initGripperLengthFest()
        self.initGripperAngleFest()
        self.initGripperLengthMove()

    def initGripperLengthFest(self):
        gripperLengthFest1 = rospy.get_param('/grasp_table_generation/gripperLengthFest1')
        gripperLengthFest2 = rospy.get_param('/grasp_table_generation/gripperLengthFest2')
        gripperLengthFest3 = rospy.get_param('/grasp_table_generation/gripperLengthFest3')
        self.gripperLengthFest = {'start': gripperLengthFest1, 'between': gripperLengthFest2, 'end': gripperLengthFest3}

    def initGripperAngleFest(self):
        gripperAngleFestL12 = rospy.get_param('/grasp_table_generation/gripperAngleFestL12')
        gripperAngleFestL23 = rospy.get_param('/grasp_table_generation/gripperAngleFestL23')
        self.gripperAngleFest = {'start': gripperAngleFestL12, 'end':  gripperAngleFestL23}

    def initGripperLengthMove(self):
        gripperLengthMove1 = rospy.get_param('/grasp_table_generation/gripperLengthMove1')
        gripperLengthMove2 = rospy.get_param('/grasp_table_generation/gripperLengthMove2')
        gripperLengthMove3 = rospy.get_param('/grasp_table_generation/gripperLengthMove3')
        self.gripperLengthMove = {'start': gripperLengthMove1, 'between': gripperLengthMove2, 'end': gripperLengthMove3}

    def initGripperWithMove(self):
        self.gripperWidthMove = rospy.get_param('/grasp_table_generation/gripperWidthMove')
    