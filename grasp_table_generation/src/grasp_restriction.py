#!/usr/bin/python

import rospy
import std_msgs
import tf
import math

class grasp_restriction(status, objectInfo2D, gripperInfo2D):
	def __init__(self, status):
		 self.disPointLineTolerance = 0.1
		
		if status is 1:
			self.distanceObjectPalm = #
			self.angleMax =  
			self.lengthLMin = 
			self.lengthRMin = 

		if status is 2:
			self.angleLMax = 
			self.lengthLMin = 

		if status is 3:
			self.angleLMax = 
			self.angleRMax = 
			self.lengthLMin = 
			self.lengthRMin =

		if status is 4:
			
			


	
	def restrictionUni(self):
		gripperInfo2Dcoordinate(self)
		checkGripperOutObject(self)
		checkObjectNearGripper(self)
		return True



	def restriction1(self):
		restrictionUni(self)
		for i in [0, 1]:
			posL[i] = self.gripper2DFest3[i] - self.gripper2DFest2[i]
			posR[i] = self.gripper2DMove3[i] - self.gripper2DMove2[i]
		self.criteriaAngleL = math.fabs(math.atan2(posL[1], posL[0]) - math.pi/2)
		self.criteriaAngleR = math.fabs(math.atan2(posR[1], posR[0]) - math.pi/2)
		if (self.criteriaAngleL>self.angleMax | self.criteriaAngleR>self.angleMax):
			print "the angle of the gripper is wrong!"
			return False
		self.criteriaDistanceL = math.sqrt((self.objectCornerLD[1]-self.gripper2DFest3[1])**2 + (self.objectCornerLD[0]-self.gripper2DFest3[0])**2)
		self.criteriaDistanceR = math.sqrt((self.objectCornerRD[1]-self.gripper2DMove3[1])**2 + (self.objectCornerRD[0]-self.gripper2DMove3[0])**2)
		if (self.criteriaDistanceL<self.lengthLMin | self.criteriaDistanceR<self.lengthRMin):
			print "the distance of the gripper is wrong!"
			return False
		return True


	def criteria1(self):
		


	def restriction2(self):
		

	def restriction3(self):
		

	def restriction4(self):

	

	def gripperInfo2Dcoordinate(self):
		self.gripper2DOrigin = gripperInfo2D.gripperPosition
		gripper2DFest1X = 
		gripper2DFest1Y = 
		self.gripper2DFest1 = [gripper2DFest1X, gripper2DFest1Y]
		gripper2DFest2X = 
		gripper2DFest2Y = 
		self.gripper2DFest2 = 
		gripper2DFest3X = 
		gripper2DFest3Y = 
		self.gripper2DFest3 = 
		gripper2DMove1X = 
		gripper2DMove1Y = 
		self.gripper2DMove1 = 
		gripper2DMove2X = 
		gripper2DMove2Y = 
		self.gripper2DMove2 = 
		gripper2DMove3X = 
		gripper2DMove3Y =
		self.gripper2DMove3 = 
		objectXL = -objectInfo2D[0]/2
		objectXR = objectInfo2D[0]/2
		objectYU = objectInfo2D[1]
		objectYD = 0
		self.objectCornerLD = [objectXL, objectYD]
		self.objectCornerLU = [objectXL, objectYU]
		self.objectCornerRD = [objectXR, objectYD]
		self.objectCornerRU = [objectXR, objectYU]

	# without tolerance of the real object and gripper 
	def lineInObject(point1, point2):
		objectXL = -objectInfo2D[0]/2
		objectXR = objectInfo2D[0]/2
		objectYU = objectInfo2D[1]
		objectYD = 0
		if point1[0]<objectXL & point2[0]<objectXL:
			print "line not in left side of the object"
			return False
		if point1[0]>objectXR & point2[0]>objectXR:
			print "line not in right side of the object"
			return False
		if point1[1]<objectYD & point2[1]<objectYD:
			print "line not in down side of the object"
			return False
		if point1[1]>objectYU & point2[1]>objectYU:
			print "line not in up side of the object"
			return False
		
		if point1[0]<objectXR & point1[0]>objectXL & point1[1]<objectYU & point1[1]>objectYD:
			print "point in the object"
			return True
		if point2[0]<objectXR & point2[0]>objectXL & point2[1]<objectYU & point2[1]>objectYD:
			print "point in the object"
			return True
		
		pointInterpolationXU = point1[0] + (point2[0]-point1[0])*(objectYU-point1[1])/(point2[1]-point1[1])
		pointInterpolationXD = point1[0] + (point2[0]-point1[0])*(objectYD-point1[1])/(point2[1]-point1[1])
		if (pointInterpolationXU<objectXR & pointInterpolationXU>objectXL):
			print "line in the object"
			return True
		if (pointInterpolationXD<objectXR & pointInterpolationXD>objectXL):
			print "line in the object"
			return True
		return False


	def checkGripperOutObject(self):
		if lineInObject(self.gripper2DOrigin, self.gripper2DFest1):
			print "wrong line 1"
			return False
		if lineInObject(self.gripper2DFest1, self.gripper2DFest2):
			print "wrong line 2"
			return False
		if lineInObject(self.gripper2DFest2, self.gripper2DFest3):
			print "wrong line 3"
			return False
		if lineInObject(self.gripper2DMove1, self.gripper2DMove2):
			print "wrong line 4"
			return False
		if lineInObject(self.gripper2DMove2, self.gripper2DMove3):
			print "wrong line 5"
			return False


	def PointOnLine(point1, point2, point, self)
		flagX = False
		flagY = False
		if (point1[0]<point[0] & point2[0]>point[0]):
			flagX = True
		if (point1[0]>point[0] & point2[0]<point[0]):
			flagX = True
		if (point1[1]<point[1] & point2[1]>point[1]):
			flagY = True
		if (point1[1]>point[1] & point2[1]<point[1]):
			flagY = True
		if not (flagX&flagY):
			print "point is not between the line!"
			return False
		disPointLine = math.fabs((point2[1]-point1[1])*point[0]-(point2[0]-point1[0])*point[1]+point2[0]*point1[1]-point2[1]*point[0])/math.sqrt((point2[1]-point1[1])**2+(point2[0]-point1[0])**2)
		if disPointLine < self.disPointLineTolerance:
			return True
		return False


	def checkObjectNearGripper(self):
		flagLD = PointOnLine(self.gripper2DFest2, self.gripper2DFest2, self.objectCornerLD, self)
		flagLU = PointOnLine(self.gripper2DFest2, self.gripper2DFest2, self.objectCornerLU, self)
		flagRD = PointOnLine(self.gripper2DFest2, self.gripper2DFest2, self.objectCornerRD, self)
		flagRU = PointOnLine(self.gripper2DFest2, self.gripper2DFest2, self.objectCornerRU, self)
		if not (flagLD | flagLU | flagRD | flagRU):
			print "object is not near the gripper!"
			return False
