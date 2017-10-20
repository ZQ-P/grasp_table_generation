#!/usr/bin/python

import rospy
import std_msgs
import tf
import geometry_msgs
from object_configuration import *
from gripper_configutation import *

class grasp_table_generation_sdhx():
	def __init__(self):
		self.objectInfo3D = object_configuration()
		self.gripperInfo3D = gripper_configutation()

		

	def objectInfo3DTo2D(self):
		self.objectInfo2D = [ [self.objectInfo3D.objectLength self.objectInfo3D.objectWidth] #xy
					[self.objectInfo3D.objectLength self.objectInfo3D.objectHeight] #xz
					[self.objectInfo3D.objectWidth self.objectInfo3D.objectLength] #yx
					[self.objectInfo3D.objectWidth self.objectInfo3D.objectHeight] #yz
					[self.objectInfo3D.objectHeight self.objectInfo3D.objectLength] #zx
					[self.objectInfo3D.objectHeight self.objectInfo3D.objectWidth] ] #zy

	def gripperInfo3DTo2D(self):
		
