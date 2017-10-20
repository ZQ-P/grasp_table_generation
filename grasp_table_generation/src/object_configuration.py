#!/usr/bin/python

import rospy
import std_msgs
import tf
import math

class ObjectConfiguration():
	def __init__(self):
		self.initPublisher()
		self.initParameter()
		self.initObjectDescription3D()
		self.initObjectDescription2D()

	def initPublisher(self):
		self.brObjectFrame = tf.TransformBroadcaster()
		rate = rospy.Rate(10.0)

	def initParameter(self):
		self.object2DFrameName = {}
		self.objectCenterFrameName = '/object3DCenter'
		self.objectOriginalCoordnationFrameName = '/objectOriginalCoordination'
		self.objectOriginalCoordnationFromCenter = {'translation': [], 'rotation': []}
		self.object2DFrameTransformFrom3D = {}
		self.object2DFrameTransformFrom3D.update({'xy': [[0, -self.objectShape3D['y']/2, 0], [0, 0, 0]]})
		self.object2DFrameTransformFrom3D.update({'xz': [[0, 0, -self.objectShape3D['z']/2], [math.pi/2, 0, 0]]})
		self.object2DFrameTransformFrom3D.update({'yx': [[-self.objectShape3D['x']/2, 0, 0], [math.pi, 0, -math.pi/2]]})
		self.object2DFrameTransformFrom3D.update({'yz': [[0, 0, -self.objectShape3D['z']/2], [math.pi/2, math.pi/2, 0]]})
		self.object2DFrameTransformFrom3D.update({'zx': [[-self.objectShape3D['x']/2, 0, 0], [-math.pi/2, 0, -math.pi/2]]})
		self.object2DFrameTransformFrom3D.update({'zy': [[0, -self.objectShape3D['y']/2, 0], [0, -math.pi/2, 0]]})

	def createObjectCenterFrame(self):
		self.brObjectFrame.sendTransform([0, 0, 0], [0, 0, 0, 1], rospy.Time.now(), self.objectCenterFrameName, '/world')
		
	def createObjectOriginalCoordnationFrame(self):
		quaternion = tf.transformations.quaternion_from_euler(self.objectOriginalCoordnationFromCenter['rotation'])
		self.brObjectFrame.sendTransform([0, 0, 0], quaternion, rospy.Time.now(), self.objectOriginalCoordnationFrameName, self.objectCenterFrameName)
		
	def initObjectDescription3D(self):
		objectLength = '~objectLength'
		objectWidth =  '~objectWidth'
		objectHeight = '~objectHeight'
		self.objectShape3D = {'x': objectLength, 'y': objectWidth, 'z': objectHeight}
       	#self.objectCoordnationPose3D = '~coordinationPosition'
		self.objectWeightLevel = ''
		self.objectHardnessLevel = ''

	def initObjectDescription2D(self):
		self.objectShape2D = {}
		objectProjectedPlanes = ['xy', 'xz', 'yx', 'yz', 'zx', 'zy']
		for i, objectProjectedPlane in enumerate(objectProjectedPlanes):
			self.createObject2DShapeAndFrame(objectProjectedPlane)
			print 'the projected plane of object: ', objectProjectedPlane

	def createObject2DShapeAndFrame(self, projectedPlane):
		self.createObject2DShape(projectedPlane)
		self.broadcastObject2DFrame(projectedPlane)

	def createObject2DShape(self, projectedPlane):
		objectShape2DX = self.objectShape3D[projectedPlane[0]]
		objectShape2DY = self.objectShape3D[projectedPlane[1]]
		self.objectShape2D.update({projectedPlane: [objectShape2DX, objectShape2DY]})

	def broadcastObject2DFrame(self, projectedPlane):
		object2DFrameTranslation = self.object2DFrameTransformFrom3D[projectedPlane][0]
		object2DFrameRotation = self.object2DFrameTransformFrom3D[projectedPlane][1]
		object2DFrameQuaternion = tf.transformations.quaternion_from_euler(object2DFrameRotation)
		object2DFrameName = '/object2D' + projectedPlane
		self.object2DFrameName.update({projectedPlane: object2DFrameName})
		self.brObjectFrame.sendTransform(object2DFrameTranslation, object2DFrameQuaternion, rospy.Time.now(), object2DFrameName, self.objectCenterFrameName)

	def getObjectshape2D(self):
		return self.objectShape2D

if __name__ is '__main__':
	rospy.init_node('object_configuration')
	objectConfig = object_configuration()
	rospy.spin()