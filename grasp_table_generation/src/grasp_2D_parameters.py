#!/usr/bin/python

import rospy
import math
from gripper_shape import gripper_shape
from object_configuration import object_configuration

class grasp_2D_parameters(gripper_shape):
    def __init__(self):
        self.initGrasp2DParameters()
        self.initGrasp2DTolerances()
        self.initGrasp2DVariables()
        self.initObject2D()


    def initGrasp2DParameters(self):
        self.gripperAngleMove = {'angleFestMove': 0, 'angleMoveMove': 0}
        self.gripperOriginalPoint = {'position': [0, 0], 'orientation': 0} 
        self.gripperSlopeAngle = 0

    def initGrasp2DTolerances(self):
        self.lengthLineConformMin = 1
        self.distanceLineConformMax = 0.2
        self.distancePointOnLineMax = 0.2
        self.gripperOriginalPointPositionAccuracy = 0.1
        self.gripperOriginalPointOrientationAccuracy = 0.01
        self.gripperAngleMoveAccuracy = 0.01

    def initGrasp2DVariables(self):
        self.gripperValidLengthFest = {}
        self.gripperValidLengthMove = {}
        self.gripperAngleMoveRoom = {'angleFestMove': [0, 0], 'angleMoveMove': [0, 0]}
        self.gripperOriginalPointPositionRoom = {'x': [0, 0], 'y': [0, 0]}
        self.gripperOriginalPointOrientationRoom = [0, 0]
        self.gripperSlopeAngleRoom = [0, 0]

    def initObject2D(self):
        objectConfig = object_configuration()
        self.objectShapes2D = objectConfig.getObjectshape2D()
        self.objectShape = {'x': 0, 'y': 0}

    def initGripperEffectiveLength(self):
        for part, length in self.gripperLengthFest:
            self.gripperValidLengthFest.update({part: length*math.cos(self.gripperSlopeAngle)})  
        for part, length in self.gripperLengthMove:
            self.gripperValidLengthMove.update({part: length*math.cos(self.gripperSlopeAngle)})




    def getParametersFromDifferentPose(self):
        for plane, objectLengthWidth in self.objectShapes2D.items():
                self.objectShape['x'] = objectLengthWidth[0]
                self.objectShape['y'] = objectLengthWidth[1]
                self.determineParametersFromParallelGrasp()

    def determineParametersFromParallelGrasp(self):
        self.createSearchRoomFromParallelGrasp()
        
        return

    def createSearchRoomFromParallelGrasp(self):
        
        return

    def ConformToRestrictionFromParallelGrasp(self):
        return

    def calculateCriteriaFromParallelGrasp(self):
        return




    def determineParametersFromPlanarPointContact(self):
        return

    def determineParametersFromPointContacts(self):
        return
    
    def determineParametersFromSlopeGrasp(self):
        return

    def isPointOnLine(point, line):
        return

    def isPointOnLineSegment(point, lineSegment):
        return

    def coincideLines(startLine, endLine):
        return

    def getDistanceFromPointToLine(point, line):
        return

    def getDistanceBetweenPoints(startPoint, endPoint):
        differenceX = endPoint[0] - startPoint[0]
        differerceY = endPoint[1] - startPoint[1]
        distance = math.sqrt(differenceX**2 + differerceY**2)
        return distance

    def getPointWithLineSegment(startPoint, lineLength, lineAngle):
		endPointX = startPoint[0] + lineLength*math.acos(lineAngle)
		endPointY = startPoint[1] + lineLength*math.asin(lineAngle)
		endPoint = [endPointX, endPointY]
		return endPoint



    def ConformToRestrictionFromPlanarPointContact(self):
        return

    def ConformToRestrictionFromPointContacts(self):
        return



    def calculateCriteriaFromPlanarPointContact(self):
        return

    def calculateCriteriaFromPointContacts(self):
        return

    def createSearchRoomFromContacts(self):
        return



