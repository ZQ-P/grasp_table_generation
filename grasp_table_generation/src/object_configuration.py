#!/usr/bin/python

import math
import rospy
import tf

class ObjectConfiguration():
    def __init__(self):
        self.initPublisher()
        self.initParameter()
        self.initBaseFrame()
        self.initObjectDescription3D()
        self.initObjectDescription2D()

    def initPublisher(self):
        self.brObjectFrame = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)

    def initParameter(self):
        self.object2DFrameName = {}
        self.objectCenterFrameName = '/object3DCenter'
        self.objectOriginalCoordnationFrameName = '/objectOriginalCoordination'
        self.objectOriginalCoordnationFromCenter = {'translation': [0,0,0], 'rotation': [0, 0, 0]}
        self.object2DFrameTransformFrom3D = {}
        self.object2DFrameTransformFrom3D.update({'xy': [[0, self.objectShape3D['y']/-2, 0], [0, 0, 0]]})
        self.object2DFrameTransformFrom3D.update({'xz': [[0, 0, self.objectShape3D['z']/-2], [math.pi/2, 0, 0]]})
        self.object2DFrameTransformFrom3D.update({'yx': [[self.objectShape3D['x']/-2, 0, 0], [math.pi, 0, -math.pi/2]]})
        self.object2DFrameTransformFrom3D.update({'yz': [[0, 0, self.objectShape3D['z']/-2], [math.pi/2, math.pi/2, 0]]})
        self.object2DFrameTransformFrom3D.update({'zx': [[self.objectShape3D['x']/-2, 0, 0], [-math.pi/2, 0, -math.pi/2]]})
        self.object2DFrameTransformFrom3D.update({'zy': [[0, self.objectShape3D['y']/-2, 0], [0, -math.pi/2, 0]]})

    def initBaseFrame(self):
        self.broadcastObjectCenterFrame()
        self.broadcastObjectOriginalCoordnationFrame()

    def broadcastObjectCenterFrame(self):
        self.brObjectFrame.sendTransform([0, 0, 0], [0, 0, 0, 1], rospy.Time.now(), self.objectCenterFrameName, '/world')

    def broadcastObjectOriginalCoordnationFrame(self):
        translation = self.objectOriginalCoordnationFromCenter['translation']
        roll = self.objectOriginalCoordnationFromCenter['rotation'][0]
        pitch = self.objectOriginalCoordnationFromCenter['rotation'][1]
        yaw = self.objectOriginalCoordnationFromCenter['rotation'][2]
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        self.brObjectFrame.sendTransform(translation, quaternion, rospy.Time.now(), self.objectOriginalCoordnationFrameName, self.objectCenterFrameName)

    def initObjectDescription3D(self):
        objectLength = '~objectLength'
        objectWidth = '~objectWidth'
        objectHeight = '~objectHeight'
        self.objectShape3D = {'x': objectLength, 'y': objectWidth, 'z': objectHeight}
        self.objectCoordnationPose3D = '~coordinationPosition'
        self.objectWeightLevel = ''
        self.objectHardnessLevel = ''

    def initObjectDescription2D(self):
        self.objectShape2D = {}
        objectProjectedPlanes = ['xy', 'xz', 'yx', 'yz', 'zx', 'zy']
        for i, objectProjectedPlane in enumerate(objectProjectedPlanes):
            self.createObject2DShape(objectProjectedPlane)
            print 'the projected plane of object (description): ', objectProjectedPlane

    def createObject2DShapeAndFrame(self, projectedPlane):
        self.createObject2DShape(projectedPlane)
        self.broadcastObject2DFrame(projectedPlane)

    def createObject2DShape(self, projectedPlane):
        objectShape2DX = self.objectShape3D[projectedPlane[0]]
        objectShape2DY = self.objectShape3D[projectedPlane[1]]
        self.objectShape2D.update({projectedPlane: [objectShape2DX, objectShape2DY]})

    def broadcastObject2DFrames(self):
        objectProjectedPlanes = ['xy', 'xz', 'yx', 'yz', 'zx', 'zy']
        for i, objectProjectedPlane in enumerate(objectProjectedPlanes):
            self.broadcastObject2DFrame(objectProjectedPlane)
            print 'the projected plane of object (frame): ', objectProjectedPlane

    def broadcastObject2DFrame(self, projectedPlane):
        object2DFrameTranslation = self.object2DFrameTransformFrom3D[projectedPlane][0]
        object2DFrameRotation = self.object2DFrameTransformFrom3D[projectedPlane][1]
        roll = object2DFrameRotation[0]
        pitch = object2DFrameRotation[1]
        yaw = object2DFrameRotation[2]
        object2DFrameQuaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        object2DFrameName = '/object2D' + projectedPlane
        self.object2DFrameName.update({projectedPlane: object2DFrameName})
        self.brObjectFrame.sendTransform(object2DFrameTranslation, object2DFrameQuaternion, rospy.Time.now(), object2DFrameName, self.objectCenterFrameName)

    def getObjectshape2D(self):
        return self.objectShape2D

if __name__ is '__main__':
    rospy.init_node('ObjectConfiguration')
    try:
        objectConfig = ObjectConfiguration()
        objectConfig.broadcastObject2DFrames()
        rospy.spin()
    except:
        exit()
