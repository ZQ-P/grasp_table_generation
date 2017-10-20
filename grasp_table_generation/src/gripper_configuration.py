#!/usr/bin/python

import rospy
import std_msgs
import tf
import geometry_msgs

class gripper_configutation():
	def __init__(self):
		self.gripperParam()


	def gripperParamFest(self):
		self.gripperLength1 = "~gripperLength1"
		self.gripperLength2 = "~gripperLength2"
		self.gripperLength3 = "~gripperLength3"
		self.gripperFingerLength1 = "~gripperFingerLength1"
		self.gripperFingerLength2 = "~gripperFingerLength2"
		self.gripperFingerLength3 = "~gripperFingerLength3"
		self.gripperAngleL12 = "~gripperAngleL12"
		self.gripperAngleL23 = "~gripperAngleL23"
		
		self.gripperLength = [self.gripperLength1, self.gripperLength2, self.gripperLength3]
		self.gripperFingerLength = [self.gripperFingerLength1, self.gripperFingerLength2, self.gripperFingerLength3]
		self.gripperAngle = [self.gripperAngle12, self.gripperAngle23]

		
	def setGripper2DParam(gripper2DPosition, gripper2DOrientation, gripperAngleFinger12, gripperAngleFinger23):
		self.gripper2DOrientation = gripper2DOrientation
		self.gripper2DPosition = gripper2DPosition
		self.gripperAngleFinger12 = gripperAngleFinger12
		self.gripperAngleFinger23 = gripperAngleFinger23
		self.gripperAngleFinger = [self.gripperAngleFinger12, self.gripperAngleFinger23]


	def getGripper2DPosition(self):
		return self.gripper2DPosition

	def getGripper2DFest(self):
		return self.gripper2DFest
	
	def getGripper2DMove(self):
		return self.gripper2DMove

	def getGripperLength(self):
		return self.gripperLength

	def getGripperFingerLength(self):
		return self.gripperFingerLength
		
	def gripper2DPointCoordination(self):
		self.gripper2DFest = Gripper2DPoints(self, self.gripperLength, self.gripperAngle)
		self.gripper2DMove = Gripper2DPoints(self, self.gripperFingerLength, self.gripperAngleFinger)

	def gripper2DPoints(self, length, angle):
		for i in range(len(length)):
			if i is 0:
				gripper2D = []
				gripper2D.append(self.gripper2DPosition)
				orientation = []
				orientation.append(self.gripper2DOrientation)
			else:
				orientationFalg = orientation[i-1] + angle[i-1]
				orientation.append(orientationFalg)
			gripper2DFlag = createPointWithLineSegment(gripper2D[i], length[i], orientation[i])
			gripper2D.append(gripper2DFlag)
		del gripper2D[0]
		return gripper2D

		#self.gripper2DFest1 = createPointWithLineSegment(self.gripper2DPosition, self.gripperLength1, self.gripper2DOrientation)
		#gripperOrientation = self.gripper2DOrientation + self.gripperAngleL12
		#self.gripper2DFest2 = createPointWithLineSegment(self.gripper2DFest1, self.gripperLength2, gripperOrientation)
		#gripperOrientation = self.gripper2DOrientation + self.gripperAngleL12 + self.gripperAngleL23
		#self.gripper2DFest3 = createPointWithLineSegment(self.gripper2DFest2, self.gripperLength3, gripperOrientation)
		#self.gripper2DMove1 = createPointWithLineSegment(self.gripper2DPosition, self.gripperFingerLength1, self.gripper2DOrientation)
		#gripperOrientation = self.gripper2DOrientation + self.gripperAngleFinger12
		#self.gripper2DMove2 = createPointWithLineSegment(self.gripper2DMove1, self.gripperFingerLength2, gripperOrientation)
		#gripperOrientation = self.gripper2DOrientation + self.gripperAngleFinger12 + self.gripperAngleFinger23
		#self.gripper2DMove3 = createPointWithLineSegment(self.gripper2DMove2, self.gripperFingerLength3, gripperOrientation)

	def createPointWithLineSegment(startPoint, length, angle):
		endPointX = startPoint[0] + length*math.acos(angle)
		endPointY = startPoint[1] + length*math.asin(angle)
		endPoint = [endPointX, endPointY]
		return endPoint


	def Point2DTo3D(self):
		
		#self.gripperQuaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)



if __name__ is "__main__":
	gripperInfo = gripper_configuration()

