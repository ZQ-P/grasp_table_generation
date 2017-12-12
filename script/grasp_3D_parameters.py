#!/usr/bin/env python

import csv
import rospy
import tf
import math
import copy
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
        self.objectName = "tissue_soft_star"

    def initParameters(self):
        self.grasp3DPose = []
        self.grasp3DFingerAngle = []
        self.existence = False


    def generate3DParameter(self):
        for index, frame in enumerate(self.object2DFrames):
            grasp2DParameter = self.generate2DParameter(frame)
            if not self.existence:
                continue
            grasp3DPoseTo2D = self.generate3DPosefrom2DParameter(index, frame, grasp2DParameter)
            grasp3DPose = self.create3DPoseToCenter(frame, grasp3DPoseTo2D)
            self.transformGraspOrientation(grasp3DPose, 'y', math.pi/2)
            self.transformGraspOrientation(grasp3DPose, 'z', math.pi)
            self.transformGraspPoseToBottom(grasp3DPose)
            self.addToGraspList(grasp3DPose, grasp2DParameter)
            self.createSymmetryPose(frame, grasp2DParameter, grasp3DPose)
        return [self.grasp3DPose, self.grasp3DFingerAngle]

    def generate2DParameter(self, frame):
        object2DLengthIndex = frame[9]
        object2DWidthIndex = frame[10]
        grasp2D = grasp_2D_parameters(self.objectShape3D[object2DLengthIndex], self.objectShape3D[object2DWidthIndex])
        grasp2DParameter = grasp2D.determineParametersFromParallelGrasp()
        self.existence = grasp2D.existence
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

    def transformGraspOrientation(self, grasp3DPose, axis, rotatingAngle):
        if axis is 'x':
            axes = 'ryzx'
        elif axis is 'y':
            axes = 'rxzy'
        elif axis is 'z':
            axes = 'rxyz'
        quaternion = [grasp3DPose.pose.orientation.x, grasp3DPose.pose.orientation.y, grasp3DPose.pose.orientation.z, grasp3DPose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion, axes)
        eulerTemp = [euler[0], euler[1], euler[2] + rotatingAngle]
        quaternion = tf.transformations.quaternion_from_euler(eulerTemp[0], eulerTemp[1], eulerTemp[2], axes)
        grasp3DPose.pose.orientation.x = quaternion[0]
        grasp3DPose.pose.orientation.y = quaternion[1]
        grasp3DPose.pose.orientation.z = quaternion[2]
        grasp3DPose.pose.orientation.w = quaternion[3]
        """quaternion = [grasp3DPose.pose.orientation.x, grasp3DPose.pose.orientation.y, grasp3DPose.pose.orientation.z, grasp3DPose.pose.orientation.w]
        euler1 = tf.transformations.euler_from_quaternion(quaternion, axes='rxyz')
        eulerTemp = [euler1[0], euler1[1], euler1[2]+math.pi]
        quaternion = tf.transformations.quaternion_from_euler(eulerTemp[0], eulerTemp[1], eulerTemp[2], axes='rxyz')
        grasp3DPose.pose.orientation.x = quaternion[0]
        grasp3DPose.pose.orientation.y = quaternion[1]
        grasp3DPose.pose.orientation.z = quaternion[2]
        grasp3DPose.pose.orientation.w = quaternion[3]"""

    def inverseGraspOrientation(self, grasp3DPose, axis):
        if axis is 'x':
            axes = 'rzyx'
        elif axis is 'y':
            axes = 'rxzy'
        elif axis is 'z':
            axes = 'rxyz'
        quaternion = [grasp3DPose.pose.orientation.x, grasp3DPose.pose.orientation.y, grasp3DPose.pose.orientation.z, grasp3DPose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion, axes)
        eulerTemp = [euler[0], euler[1], -euler[2]]
        quaternion = tf.transformations.quaternion_from_euler(eulerTemp[0], eulerTemp[1], eulerTemp[2], axes)
        grasp3DPose.pose.orientation.x = quaternion[0]
        grasp3DPose.pose.orientation.y = quaternion[1]
        grasp3DPose.pose.orientation.z = quaternion[2]
        grasp3DPose.pose.orientation.w = quaternion[3]

    def transformGraspPoseToBottom(self, grasp3DPose):
        grasp3DPose.pose.position.z += self.objectShape3D['z'] / 2

    def addToGraspList(self, grasp3DPose, grasp2DParameter):
        self.grasp3DPose.append(grasp3DPose)
        self.grasp3DFingerAngle.append(grasp2DParameter[1])

    def createSymmetryPose(self, frame, grasp2DParameter, grasp3DPose):
        """still needs to be corrected"""
        grasp3DPose1 = copy.deepcopy(grasp3DPose)
        if frame[9] is 'x':
            grasp3DPose1.pose.position.x = -grasp3DPose1.pose.position.x
        elif frame[9] is 'y':
            grasp3DPose1.pose.position.y = -grasp3DPose1.pose.position.y
        elif frame[9] is 'z':
            grasp3DPose1.pose.position.z = self.objectShape3D['z'] - grasp3DPose1.pose.position.z
        self.transformGraspOrientation(grasp3DPose1, 'z', math.pi)
        self.inverseGraspOrientation(grasp3DPose1, 'x')
        self.addToGraspList(grasp3DPose1, grasp2DParameter)

        grasp3DPose2 = copy.deepcopy(grasp3DPose1)
        if frame[10] is 'x':
            grasp3DPose2.pose.position.x = -grasp3DPose2.pose.position.x
        elif frame[10] is 'y':
            grasp3DPose2.pose.position.y = -grasp3DPose2.pose.position.y
        elif frame[10] is 'z':
            grasp3DPose2.pose.position.z = self.objectShape3D['z'] - grasp3DPose2.pose.position.z
        self.transformGraspOrientation(grasp3DPose2, 'x', math.pi)
        self.addToGraspList(grasp3DPose2, grasp2DParameter)

        grasp3DPose3 = copy.deepcopy(grasp3DPose2)
        if frame[9] is 'x':
            grasp3DPose3.pose.position.x = -grasp3DPose3.pose.position.x
        elif frame[9] is 'y':
            grasp3DPose3.pose.position.y = -grasp3DPose3.pose.position.y
        elif frame[9] is 'z':
            grasp3DPose3.pose.position.z = self.objectShape3D['z'] - grasp3DPose3.pose.position.z
        self.transformGraspOrientation(grasp3DPose3, 'z', math.pi)
        self.inverseGraspOrientation(grasp3DPose1, 'x')
        self.addToGraspList(grasp3DPose3, grasp2DParameter)


    def writeGraspListInCSV(self):
        #filename = "/home/rmb-pz/git/care-o-bot-indigo/src/grasp_table_generation/script/sdhx_" + self.objectName + ".csv"
        filename = "/home/rmb-pz/git/care-o-bot-indigo/src/cob_manipulation/cob_grasp_generation/files/database/tissue_soft_star/sdhx_" + self.objectName + ".csv"
        with open(filename, 'wb') as csvfile:
            csvWriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            titleOfCSV = ['id\t','object\t', 'gripper_finger_1_joint\t', 'gripper_finger_2_joint\t', 'direction\t', 
                            'qw\t', 'qx\t', 'qy\t', 'qz\t', 'pos-x\t', 'pos-y\t', 'pos-z\t', 'eps_l1\t', 'vol_l1\t']
            csvWriter.writerow(titleOfCSV)
            for index, grasp3DPose in enumerate(self.grasp3DPose):
                if grasp3DPose is []:
                    continue
                output = self.transformToCSVForm(index, grasp3DPose)
                csvWriter.writerow(output)
            #print "write the file successfully"
        
    def transformToCSVForm(self, index, grasp3DPose):
        output = []
        output.append(str(index) + '\t')
        output.append(self.objectName + '\t')
        output.append(str(self.grasp3DFingerAngle[index]['start']) + '\t')
        output.append(str(self.grasp3DFingerAngle[index]['end']) + '\t')
        output.append('SIDE\t')
        output.append(str(grasp3DPose.pose.orientation.w) + '\t')
        output.append(str(grasp3DPose.pose.orientation.x) + '\t')
        output.append(str(grasp3DPose.pose.orientation.y) + '\t')
        output.append(str(grasp3DPose.pose.orientation.z) + '\t')
        output.append(str(grasp3DPose.pose.position.x) + '\t')
        output.append(str(grasp3DPose.pose.position.y) + '\t')
        output.append(str(grasp3DPose.pose.position.z) + '\t')
        output.append('0.01\t')
        output.append('0.0005\t')
        output.append(str(index) + '\t')
        return output

        
if __name__ == "__main__":
    rospy.init_node("grasp_3D_parameters")
    grasp3DParameters = grasp_3D_parameters()
    try:
        grasp3DParameters.generate3DParameter()
        pass
    except:
        rospy.logerr("grasp_3D_parameters: cannot generate 3D Parameters!")
        exit()
    rospy.loginfo("grasp_3D_parameters: generate the grasp list successfully!")
    try:
        grasp3DParameters.writeGraspListInCSV()
    except:
        rospy.logerr("grasp_3D_parameters: something is wrong while writing the data to csv file!")
        exit()
    rospy.loginfo("grasp_3D_parameters: save the paramters to the csv file successfully!")
    rospy.spin()

