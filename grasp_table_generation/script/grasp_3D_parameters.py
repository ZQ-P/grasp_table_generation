#!/usr/bin/env python

import csv
import rospy
import tf
import math
from ObjectConfiguration import ObjectConfiguration
from grasp_2D_parameters import grasp_2D_parameters
from geometry_msgs.msg import PoseStamped

class grasp_3D_parameters():
    def __init__(self):
        self.initListener()
        self.initObject()
        self.initParameters()


    def initListener(self):
        self.listener = tf.TransformListener()

    def initObject(self):
        objectLength = rospy.get_param('/grasp_table_generation/objectLength')
        objectWidth = rospy.get_param('/grasp_table_generation/objectWidth')
        objectHeight = rospy.get_param('/grasp_table_generation/objectHeight')
        self.objectShape3D = {'x': objectLength, 'y': objectWidth, 'z': objectHeight}
        self.object2DFrames = ['/object2Dxy', '/object2Dxz', '/object2Dyx', '/object2Dyz', '/object2Dzx', '/object2Dzy']
        self.objectCenterFrameName = '/object3DCenter'
        self.objectOriginalCoordnationFrameName = '/objectOriginalCoordination'

    def initParameters(self):
        self.grasp3DPose = []
        self.grasp3DFingerAngle = []


    def generate3DParameter(self):
        for index, frame in enumerate(self.object2DFrames):
            grasp2DParameter = self.generate2DParameter(frame)
            grasp3DPoseTo2D = self.generate3DPosefrom2DParameter(index, frame, grasp2DParameter)
            grasp3DPose = self.create3DPoseToCenter(frame, grasp3DPoseTo2D)
            self.addToGraspList(grasp3DPose, grasp2DParameter)
            self.createSymmetryPose(frame, grasp2DParameter, grasp3DPose)
            grasp2DParameter[0]['orientation'] += math.pi
            grasp3DPoseTo2D = self.generate3DPosefrom2DParameter(index, frame, grasp2DParameter)
            grasp3DPose = self.create3DPoseToCenter(frame, grasp3DPoseTo2D)
            self.createSymmetryPose(frame, grasp2DParameter, grasp3DPose)
        return [self.grasp3DPose, self.grasp3DFingerAngle]

    def generate2DParameter(self, frame):
        object2DLengthIndex = frame[9]
        object2DWidthIndex = frame[10]
        grasp2D = grasp_2D_parameters(self.objectShape3D[object2DLengthIndex], self.objectShape3D[object2DWidthIndex])
        grasp2DParameter = grasp2D.determineParametersFromParallelGrasp()
        return grasp2DParameter

    def generate3DPosefrom2DParameter(self, index, frame, grasp2DParameter):
        grasp3DPoseTo2D = PoseStamped()
        grasp3DPoseTo2D.header.seq = index
        grasp3DPoseTo2D.header.stamp = rospy.Time.now()
        grasp3DPoseTo2D.header.frame_id = frame
        grasp3DPoseTo2D.pose.position.x = grasp2DParameter[0]['position'][0]
        grasp3DPoseTo2D.pose.position.y = grasp2DParameter[0]['position'][1]
        grasp3DPoseTo2D.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, grasp2DParameter[0]['orientation'])
        grasp3DPoseTo2D.pose.orientation.x = quaternion[0]
        grasp3DPoseTo2D.pose.orientation.y = quaternion[1]
        grasp3DPoseTo2D.pose.orientation.z = quaternion[2]
        grasp3DPoseTo2D.pose.orientation.w = quaternion[3]
        return grasp3DPoseTo2D

    def create3DPoseToCenter(self, frame, grasp3DPoseTo2D):
        grasp3DPose = PoseStamped()
        try:
            self.listener.waitForTransform(self.objectCenterFrameName, frame, rospy.Time.now(), rospy.Duration(10.0))
            grasp3DPose = self.listener.transformPose(self.objectCenterFrameName, grasp3DPoseTo2D)
        except:
            rospy.logerr("generate3DPose: cannot tranform the position")
            exit()
        #print "-------------------------------------------------"
        return grasp3DPose

    def addToGraspList(self, grasp3DPose, grasp2DParameter):
        self.grasp3DPose.append(grasp3DPose)
        self.grasp3DFingerAngle.append(grasp2DParameter[1])

    def createSymmetryPose(self, frame, grasp2DParameter, grasp3DPose):
        if frame[10] is 'x':
            grasp3DPose.pose.position.x = -grasp3DPose.pose.position.x
        elif frame[10] is 'y':
            grasp3DPose.pose.position.y = -grasp3DPose.pose.position.y
        elif frame[10] is 'z':
            grasp3DPose.pose.position.z = -grasp3DPose.pose.position.z
        self.addToGraspList(grasp3DPose, grasp2DParameter)


    def writeGraspListInCSV(self):
        filename = 'sdhx_filename.csv'
        with open(filename, 'wb') as csvfile:
            print csvfile
            output = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_ALL)
            print output.writerow(['id', 'object', 'gripper_finger_1_joint', 'gripper_finger_2_joint', 'direction', 'qw', 'qx', 'qy', 'qz', 'pos-x', 'pos-y', 'pos-z', 'eps_l1', 'vol_l1'])
            print "write the file successfully"

        
if __name__ == "__main__":
    rospy.init_node("grasp_3D_parameters")
    grasp3DParameters = grasp_3D_parameters()
    try:
        #grasp3DParameters.generate3DParameter()
        pass
    except:
        rospy.logerr("grasp_3D_parameters: cannot generate 3D Parameters!")
        exit()
    rospy.loginfo("grasp_3D_parameters: generate the grasp list successfully!")
    try:
        grasp3DParameters.writeGraspListInCSV()
    except:
        rospy.logerr("grasp_3D_parameters: cannot open the file!")
        exit()
    rospy.loginfo("grasp_3D_parameters: save the paramters to the csv file successfully!")
    rospy.spin()
