#!/usr/bin/env python

import rospy
import math
import numpy
import copy
import scipy
from gripper_shape import gripper_shape
from ObjectConfiguration import ObjectConfiguration

class grasp_2D_parameters(gripper_shape):
    def __init__(self, objectX, objectY):
        """initialize the parameters, arguments"""
        gripper_shape.__init__(self)
        self.initGrasp2DParameters()
        self.initGrasp2DTolerances()
        self.initGrasp2DVariables()
        self.initObject2D(objectX, objectY)


    def initGrasp2DParameters(self):
        """initialize and declare parameters of gripper"""
        self.gripperAngleMove = {'start': 0, 'end': 0}
        self.gripperOriginalPoint = {'position': [0, 0], 'orientation': math.pi/2}
        self.gripperSlopeAngle = 0
        self.evaluationValue = 1000000
        self.existence = False

    def initGrasp2DTolerances(self):
        """initialize and declare parameters of tolerances"""
        self.lengthLineConformMin = 1
        self.distanceObectGripperMin = 5
        #self.distanceLineConformMax = 0.2
        self.distancePointOnLineMax = 1
        self.gripperOriginalPointPositionAccuracy = 1
        self.gripperOriginalPointOrientationAccuracy = 0.01
        self.gripperAngleMoveAccuracy = 0.01

    def initGrasp2DVariables(self):
        """initialize and declare the parameters, that are needed later"""
        self.objectShapePoints = {}
        self.gripperValidLengthFest = {}
        self.gripperValidLengthMove = {}
        self.gripperPointsPositionFest = {}
        self.gripperPointsPositionMove = {}
        self.gripperAngleMoveRoom = {'start': [0, 0], 'end': [0, 0]}
        self.gripperOriginalPointPoseRoom = {'x': [0, 0], 'y': [0, 0], 'r': [0, 0]}
        #self.gripperOriginalPointOrientationRoom = [0, 0]
        self.gripperSlopeAngleRoom = [0, 0]

    def initObject2D(self, objectX, objectY):
        """initialize parameters of object"""
        self.objectShape = {'x': objectX, 'y': objectY}
        self.objectShapePoints.update({'leftUp': [-0.5*self.objectShape['x'], self.objectShape['y']]})
        self.objectShapePoints.update({'leftDown': [-0.5*self.objectShape['x'], 0]})
        self.objectShapePoints.update({'rightUp': [0.5*self.objectShape['x'], self.objectShape['y']]})
        self.objectShapePoints.update({'rightDown': [0.5*self.objectShape['x'], 0]})

    def initGripperEffectiveLength(self):
        """translate the gripper length to the effective gripper length based on gripper slope"""
        for part, length in self.gripperLengthFest.items():
            self.gripperValidLengthFest.update({part: length*math.cos(self.gripperSlopeAngle)})
        for part, length in self.gripperLengthMove.items():
            self.gripperValidLengthMove.update({part: length*math.cos(self.gripperSlopeAngle)})


    def createGripperPointsPosition(self):
        """create coordinates of gripper fest and moving joints"""
        self.createGripperPointsPositionFest()
        self.createGripperPointsPositionMove()

    def createGripperPointsPositionFest(self):
        """create coordinates of gripper fest joints"""
        startPointPosition = self.gripperOriginalPoint['position']
        startPointOrientation = self.gripperOriginalPoint['orientation']
        segmentLength = self.gripperValidLengthFest['start']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'start': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleFest['start']
        segmentLength = self.gripperValidLengthFest['between']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'between': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleFest['end']
        segmentLength = self.gripperValidLengthFest['end']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'end': endPointPosition})

    def createGripperPointsPositionMove(self):
        """create coordinates of gripper moving joints"""
        startPointPosition = self.gripperOriginalPoint['position']
        startPointOrientation = self.gripperOriginalPoint['orientation']
        segmentLength = self.gripperValidLengthMove['start']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'start': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation = self.gripperAngleMove['start']
        segmentLength = self.gripperValidLengthMove['between']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'between': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation = self.gripperAngleMove['end']
        segmentLength = self.gripperValidLengthMove['end']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'end': endPointPosition})

    # test the function!!!!! select a good orientation
    def findParametersWithExhaustiveSearch(self, graspType):
        """using exhaustive search to find optimized parameters"""
        """graspType = '1': parallel grasp"""
        """graspType = '2': grasp with planar and point contacts"""
        """graspType = '3': grasp with point contacts"""
        self.initGrasp2DParameters()
        startX = self.gripperOriginalPointPoseRoom['x'][0]
        endX = self.gripperOriginalPointPoseRoom['x'][1]
        stepX = self.gripperOriginalPointPositionAccuracy
        startY = self.gripperOriginalPointPoseRoom['y'][0]
        endY = self.gripperOriginalPointPoseRoom['y'][1]
        stepY = self.gripperOriginalPointPositionAccuracy
        startR = self.gripperOriginalPointPoseRoom['r'][0]
        endR = self.gripperOriginalPointPoseRoom['r'][1]
        stepR = self.gripperOriginalPointOrientationAccuracy
        startAS = self.gripperAngleMoveRoom['start'][0]
        endAS = self.gripperAngleMoveRoom['start'][1]
        stepAS = self.gripperOriginalPointOrientationAccuracy
        startAE = self.gripperAngleMoveRoom['end'][0]
        endAE = self.gripperAngleMoveRoom['end'][1]
        stepAE = self.gripperOriginalPointOrientationAccuracy
        point = None
        angle = None
        if self.confirmToObjectShapeRestricrion(graspType):
            for self.gripperOriginalPoint['position'][0] in numpy.arange(startX, endX, stepX):
                for self.gripperOriginalPoint['position'][1] in numpy.arange(startY, endY, stepY):
                    for self.gripperOriginalPoint['orientation'] in numpy.arange(startR, endR, stepR):
                        for self.gripperAngleMove['start'] in numpy.arange(startAS, endAS, stepAS):
                            for self.gripperAngleMove['end'] in numpy.arange(startAE, endAE, stepAE):
                                self.createGripperPointsPosition()
                                if not self.conformToRestriction(graspType):
                                    continue
                                evalValue = self.calculateCriteria(graspType)
                                if evalValue > self.evaluationValue:
                                    continue
                                self.evaluationValue = evalValue
                                point = copy.deepcopy(self.gripperOriginalPoint)
                                angle = copy.deepcopy(self.gripperAngleMove)
        if point is not None:
            self.existence = True
            self.gripperOriginalPoint = point
            self.gripperAngleMove = angle

    def confirmToObjectShapeRestricrion(self, graspType):
        """check if the shape of object confirms to the shape of gripper"""
        if graspType is '1':
            return self.confirmToObjectShapeRestricrionFromParallelGrasp()
        if graspType is '2':
            return self.confirmToObjectShapeRestricrionFromPlanarPointContact()
        if graspType is '3':
            return self.confirmToObjectShapeRestricrionFromPointContacts()
        else:
            print "there is no such grasping type!"
            exit()

    def conformToRestriction(self, graspType):
        """check if choosed parameters confirm to the restriction of grasping"""
        if graspType is '1':
            return self.conformToRestrictionFromParallelGrasp()
        if graspType is '2':
            return self.conformToRestrictionFromPlanarPointContact()
        if graspType is '3':
            return self.conformToRestrictionFromPointContacts()
        else:
            print "there is no such grasping type!"
            exit()

    def calculateCriteria(self, graspType):
        """calculte the evaluated value based on the function of evaluation and choosed parameters"""
        if graspType is '1':
            return self.calculateCriteriaFromParallelGrasp()
        if graspType is '2':
            return self.calculateCriteriaFromPlanarPointContact()
        if graspType is '3':
            return self.calculateCriteriaFromPointContacts()
        else:
            print "there is no such grasping type!"
            exit()



    def determineParametersFromParallelGrasp(self):
        """calculate optimized parameters for grasping"""
        self.gripperSlopeAngle = 0
        self.initGrasp2DParameters()
        self.initGripperEffectiveLength()
        self.createSearchRoomFromParallelGrasp()
        self.findParametersWithExhaustiveSearch('1')
    
    # position y maybe need to change, but be careful if using exaustive search
    # CMA-ES or genetic algorithm (GA) or Greedy 
    def createSearchRoomFromParallelGrasp(self):
        """create the searching room for the optimizing method"""
        accurateOrientation = math.pi/2 - self.gripperAngleFest['start'] - self.gripperAngleFest['end']
        self.gripperOriginalPointPoseRoom['r'][0] = accurateOrientation - 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperOriginalPointPoseRoom['r'][1] = accurateOrientation + 2*self.gripperOriginalPointOrientationAccuracy
        accuratePositionX = -self.objectShape['x']/2 + self.getGripperTotalFestLengthX()
        self.gripperOriginalPointPoseRoom['x'][0] = accuratePositionX - 2*self.gripperOriginalPointPositionAccuracy
        self.gripperOriginalPointPoseRoom['x'][1] = accuratePositionX + 2*self.gripperOriginalPointPositionAccuracy
        accuratePositionY = -self.getGripperTotalFestLengthY()
        self.gripperOriginalPointPoseRoom['y'][0] = accuratePositionY - 2*self.gripperOriginalPointPositionAccuracy
        self.gripperOriginalPointPoseRoom['y'][1] = accuratePositionY + self.gripperLengthFest['end']
        self.gripperAngleMoveRoom['start'][0] = 0 - 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperAngleMoveRoom['start'][1] = math.pi + 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperAngleMoveRoom['end'][0] = math.pi/2 - 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperAngleMoveRoom['end'][1] = math.pi/2 + 2*self.gripperOriginalPointOrientationAccuracy
        print self.gripperOriginalPointPoseRoom, self.gripperAngleMoveRoom, self.gripperAngleFest
        

    def confirmToObjectShapeRestricrionFromParallelGrasp(self):
        """check if the shape of object confirms to the shape of gripper"""
        angle = self.gripperAngleFest['start'] 
        objectLengthXMin = self.gripperValidLengthFest['between']*math.sin(angle) - self.gripperValidLengthMove['between'] \
                            - self.gripperOriginalPointPositionAccuracy
        objectLengthXMax = self.gripperValidLengthFest['between']*math.sin(angle) + self.gripperValidLengthMove['between'] \
                            + self.gripperOriginalPointPositionAccuracy
        if self.objectShape['x'] < objectLengthXMin:
            return False
        if self.objectShape['x'] > objectLengthXMax:
            return False
        return True

    def conformToRestrictionFromParallelGrasp(self):
        """check if choosed parameters confirm to the restriction of grasping"""
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineGripperMove = [self.gripperPointsPositionMove['between'], self.gripperPointsPositionMove['end']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        lineObjectRight = [self.objectShapePoints['rightUp'], self.objectShapePoints['rightDown']]
        if not self.isLinesCoincide(lineGripperFest, lineObjectLeft):
            return False
        if not self.isLinesCoincide(lineGripperMove, lineObjectRight):
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        if distance < self.distanceObectGripperMin:
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['end'], self.objectShapePoints['leftDown'])
        if distance < self.lengthLineConformMin:
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['end'], self.objectShapePoints['rightDown'])
        if distance < self.lengthLineConformMin:
            return False
        angleMax = math.pi/2 + 2*self.gripperOriginalPointOrientationAccuracy
        angleMin = math.pi/2 - 2*self.gripperOriginalPointOrientationAccuracy
        if self.isAngleAlmostEqual(math.pi/2, self.gripperAngleMove['start']):
            distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        else:
            distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightDown'])
        if distance < self.distanceObectGripperMin:
            return False
        return True


    def calculateCriteriaFromParallelGrasp(self):
        """calculte the evaluated value based on the function of evaluation and choosed parameters"""
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineGripperMove = [self.gripperPointsPositionMove['between'], self.gripperPointsPositionMove['end']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        lineObjectRight = [self.objectShapePoints['rightUp'], self.objectShapePoints['rightDown']]
        L1 = self.getCoincideLineLength(lineGripperFest, lineObjectLeft)
        L2 = self.getCoincideLineLength(lineGripperMove, lineObjectRight)
        L3 = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        if self.isAngleAlmostEqual(math.pi/2, self.gripperAngleMove['start']):
            L4 = 0
        else:
            L4 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightDown'])
        criteria1 = (L1+L2)/(self.objectShape['x']+self.objectShape['y'])
        criteria2 = math.fabs(L1-L2)/max(self.gripperValidLengthFest['end'], self.gripperValidLengthMove['end'])
        criteria3 = (L3+L4)/max(self.gripperValidLengthFest['end'], self.gripperValidLengthMove['end'])
        criteria4 = (self.objectShape['y'] - (L1+L2)/2)/self.objectShape['y']
        evalValue = -criteria1 + criteria2 + criteria3 + criteria4
        return evalValue

    def isAngleAlmostEqual(self, angle1, angle2):
        """check if 2 float angels equal"""
        flag = (math.fabs(angle1-angle2) <= 3*self.gripperOriginalPointOrientationAccuracy)
        return flag

    def getGripperTotalFestLengthX(self):
        """calculate the total length of fest gripper in direction x"""
        theta = self.gripperOriginalPoint['orientation']
        startLengthX = self.gripperValidLengthFest['start']*math.sin(theta-math.pi/2)
        theta += self.gripperAngleFest['start']
        betweenLengthX = self.gripperLengthFest['between']*math.sin(theta-math.pi/2)
        theta += self.gripperAngleFest['end']
        endLengthX = self.gripperValidLengthFest['end']*math.sin(theta-math.pi/2)
        totalLengthX = startLengthX + betweenLengthX + endLengthX
        return totalLengthX

    def getGripperTotalFestLengthY(self):
        """calculate the total length of fest gripper in direction y"""
        theta = self.gripperOriginalPoint['orientation']
        startLengthY = self.gripperValidLengthFest['start']*math.cos(theta-math.pi/2)
        theta += self.gripperAngleFest['start']
        betweenLengthY = self.gripperLengthFest['between']*math.cos(theta-math.pi/2)
        theta += self.gripperAngleFest['end']
        endLengthY = self.gripperValidLengthFest['end']*math.cos(theta-math.pi/2)
        totalLengthY = startLengthY + betweenLengthY + endLengthY
        return totalLengthY


    def determineParametersFromPlanarPointContact(self):
        """calculate optimized parameters for grasping"""
        self.gripperSlopeAngle = 0
        self.initGrasp2DParameters()
        self.initGripperEffectiveLength()
        self.createSearchRoomFromPlanarPointContact()
        self.findParametersWithExhaustiveSearch('2')

    def createSearchRoomFromPlanarPointContact(self):
        """create the searching room for the optimizing method"""
        accurateOrientation = math.pi - self.gripperAngleFest['start'] - self.gripperAngleFest['end']
        self.gripperOriginalPointPoseRoom['r'][0] = accurateOrientation - 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperOriginalPointPoseRoom['r'][1] = accurateOrientation + 2*self.gripperOriginalPointOrientationAccuracy
        accuratePositionX = -self.objectShape['x']/2 + self.getGripperTotalFestLengthX()
        self.gripperOriginalPointPoseRoom['x'][0] = accuratePositionX - 5*self.gripperOriginalPointPositionAccuracy
        self.gripperOriginalPointPoseRoom['x'][1] = accuratePositionX + 3*self.gripperOriginalPointPositionAccuracy
        accuratePositionY = -self.getGripperTotalFestLengthY()
        self.gripperOriginalPointPoseRoom['y'][0] = accuratePositionY
        self.gripperOriginalPointPoseRoom['y'][1] = accuratePositionY + self.objectShape['y']
        self.gripperAngleMoveRoom['start'][0] = 0
        self.gripperAngleMoveRoom['start'][1] = math.pi/2
        self.gripperAngleMoveRoom['end'][0] = math.pi/2
        self.gripperAngleMoveRoom['end'][1] = math.pi

    def confirmToObjectShapeRestricrionFromPlanarPointContact(self):
        """check if the shape of object confirms to the shape of gripper"""
        if self.objectShape['x'] < self.getGripperTotalFestLengthX():
            return False
        if self.objectShape['x'] > (self.getGripperTotalFestLengthX()+self.gripperLengthMove['between']):
            return False
        if self.objectShape['y'] < (self.gripperLengthMove['between']-self.gripperLengthMove['end']):
            return False
        if self.objectShape['y'] > self.gripperLengthMove['between']:
            return False
        return True


    def conformToRestrictionFromPlanarPointContact(self):
        """check if choosed parameters confirm to the restriction of grasping"""
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineGripperMoveEnd = [self.gripperPointsPositionMove['between'], self.gripperPointsPositionMove['end']]
        lineGripperMoveBetween = [self.gripperPointsPositionMove['start'], self.gripperPointsPositionMove['between']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        if not self.isLinesCoincide(lineGripperFest, lineObjectLeft):
            return False
        if not self.isPointOnLineSegment(self.objectShapePoints['rightUp'], lineGripperMoveEnd):
            return False
        if not self.isPointOnLineSegment(self.objectShapePoints['rightDown'], lineGripperMoveBetween):
            return False
        
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        if distance < self.distanceObectGripperMin:
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['end'], self.objectShapePoints['leftDown'])
        if distance < self.lengthLineConformMin:
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        if distance < self.distanceObectGripperMin:
            return False
        distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightUp'])
        if distance < self.distanceObectGripperMin:
            return False
        #distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['end'], self.objectShapePoints['rightUp'])
        #if distance < self.lengthLineConformMin:
        #    return False

        return True

    def calculateCriteriaFromPlanarPointContact(self):
        """calculte the evaluated value based on the function of evaluation and choosed parameters"""
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        slopeGripperFestEnd = self.gripperOriginalPoint['orientation'] + self.gripperAngleFest['start'] + self.gripperAngleFest['end']
        objectCenterPoint = [0, self.objectShape['y']/2]
        L1 = self.getCoincideLineLength(lineGripperFest, lineObjectLeft)
        L2 = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        L3 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        L4 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightUp'])
        line5 = [[-0.5*self.objectShape[x], 0.5*L1], slopeGripperFestEnd-math.pi/2]
        L5 = self.getDistanceFromPointToLine(objectCenterPoint, line5, '2')
        line6 = [self.objectShapePoints['rightDown'], self.gripperAngleMove['start']-math.pi/2]
        L6 = self.getDistanceFromPointToLine(objectCenterPoint, line6, '2')
        line7 = [self.objectShapePoints['rightUp'], self.gripperAngleMove['end']-math.pi/2]
        L7 = self.getDistanceFromPointToLine(objectCenterPoint, line7, '2')
        L8 = self.objectShape['y']/2 - L1/2
        criteria1 = (L5+L6+L7)/((self.objectShape['x']+self.objectShape['y'])/2)
        criteria2 = 2*L8/self.objectShape['y']
        criteria3Max = max(self.gripperValidLengthFest['end'], self.gripperValidLengthMove['between'], self.gripperValidLengthMove['end'])
        criteria3 = (L2 + L3 + 0.5*L4)/criteria3Max
        criteria4 = L1/self.objectShape['y']
        evalValue = criteria1 + criteria2 + criteria3 - criteria4
        return evalValue


    def determineParametersFromPointContacts(self):
        """calculate optimized parameters for grasping"""
        self.gripperSlopeAngle = 0
        self.initGrasp2DParameters()
        self.initGripperEffectiveLength()
        self.createSearchRoomFromPointContacts()
        self.findParametersWithExhaustiveSearch('3')

    def createSearchRoomFromPointContacts(self):
        """create the searching room for the optimizing method"""
        accuratePositionX = -self.objectShape['x']/2 + self.getGripperTotalFestLengthX()
        accuratePositionY = -self.getGripperTotalFestLengthY()
        self.gripperOriginalPointPoseRoom['r'][0] = 0
        self.gripperOriginalPointPoseRoom['r'][1] = math.pi/2
        self.gripperOriginalPointPoseRoom['x'][0] = accuratePositionX
        self.gripperOriginalPointPoseRoom['x'][1] = -self.objectShape['x']/2 + accuratePositionY
        self.gripperOriginalPointPoseRoom['y'][0] = accuratePositionY
        self.gripperOriginalPointPoseRoom['y'][1] = accuratePositionY + self.objectShape['y']
        self.gripperAngleMoveRoom['start'][0] = 0
        self.gripperAngleMoveRoom['start'][1] = math.pi/2
        self.gripperAngleMoveRoom['end'][0] = math.pi/2
        self.gripperAngleMoveRoom['end'][1] = math.pi

    def confirmToObjectShapeRestricrionFromPointContacts(self):
        """check if the shape of object confirms to the shape of gripper"""
        if self.objectShape['x'] < self.getGripperTotalFestLengthX():
            return False
        if self.objectShape['x'] > (self.getGripperTotalFestLengthX()+self.gripperLengthMove['between']):
            return False
        if self.objectShape['y'] < (self.gripperLengthMove['between']-self.gripperLengthMove['end']):
            return False
        if self.objectShape['y'] > self.gripperLengthMove['between']:
            return False
        return True

    def conformToRestrictionFromPointContacts(self):
        """check if choosed parameters confirm to the restriction of grasping"""
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineGripperMoveStart = [self.gripperPointsPositionMove['start'], self.gripperPointsPositionMove['between']]
        lineGripperMoveEnd = [self.gripperPointsPositionMove['between'], self.gripperPointsPositionMove['end']]
        if not self.isPointOnLineSegment(self.objectShapePoints['leftDown'], lineGripperFest):
            return False
        if not self.isPointOnLineSegment(self.objectShapePoints['rightDown'], lineGripperMoveStart):
            return False
        if not self.isPointOnLineSegment(self.objectShapePoints['rightUp'], lineGripperMoveEnd):
            return False
        if self.gripperPointsPositionFest['start'][1] > 0:
            return False
        return True

    def calculateCriteriaFromPointContacts(self):
        """calculte the evaluated value based on the function of evaluation and choosed parameters"""
        L1 = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        L2 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        L3 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightUp'])
        slopeGripperFestEnd = self.gripperOriginalPoint['orientation'] + self.gripperAngleFest['start'] + self.gripperAngleFest['end']
        objectCenterPoint = [0, self.objectShape['y']/2]
        line4 = [[-0.5*self.objectShape[x], 0.5*L1], slopeGripperFestEnd-math.pi/2]
        L4 = self.getDistanceFromPointToLine(objectCenterPoint, line4, '2')
        line5 = [self.objectShapePoints['rightDown'], self.gripperAngleMove['start']-math.pi/2]
        L5 = self.getDistanceFromPointToLine(objectCenterPoint, line5, '2')
        line6 = [self.objectShapePoints['rightUp'], self.gripperAngleMove['end']-math.pi/2]
        L6 = self.getDistanceFromPointToLine(objectCenterPoint, line6, '2')
        criteria1Max = max(self.gripperValidLengthFest['end'], self.gripperValidLengthMove['between'], self.gripperValidLengthMove['end'])
        criteria1 = (L1 + L2 + 0.5*L3)/criteria1Max
        criteria2 = (L5+L6+L7)/((self.objectShape['x']+self.objectShape['y'])/2)
        evalValue = criteria1 + criteria2
        return evalValue



    #set slope angle haven't been finished
    def determineParametersFromSlopeGrasp(self):
        """calculate optimized parameters for grasping"""
        self.initGrasp2DParameters()
        self.setGripperSlopeAngle(1)
        self.initGripperEffectiveLength()
        self.createSearchRoomFromParallelGrasp()
        self.findParametersWithExhaustiveSearch('1')

    def setGripperSlopeAngle(self, angle):
        self.gripperSlopeAngle = angle

    def ConformToRestrictionFromSlopeGrasp(self):
        """check if choosed parameters confirm to the restriction of grasping"""
        return


    def isPointOnLine(self, point, line):
        distance = self.getDistanceFromPointToLine(point, line)
        if distance < self.distancePointOnLineMax:
            return True
        else:
            return False

    def isPointOnLineSegment(self, point, lineSegment):
        if not self.isPointOnLine(point, lineSegment):
            return False
        xMin = min(lineSegment[0][0], lineSegment[1][0]) - self.gripperOriginalPointPositionAccuracy
        xMax = max(lineSegment[0][0], lineSegment[1][0]) + self.gripperOriginalPointPositionAccuracy
        yMin = min(lineSegment[0][1], lineSegment[1][1]) - self.gripperOriginalPointPositionAccuracy
        yMax = max(lineSegment[0][1], lineSegment[1][1]) + self.gripperOriginalPointPositionAccuracy
        if ((point[0] >= xMin)and(point[0] <= xMax)) and ((point[1] >= yMin)and(point[1] <= yMax)):
            return True
        else:
            return False

    def isLinesCoincide(self, startLine, endLine):
        numberPointOnLineSegment = 0
        if self.isPointOnLineSegment(startLine[0], endLine):
            numberPointOnLineSegment += 1
        if self.isPointOnLineSegment(startLine[1], endLine):
            numberPointOnLineSegment += 1
        if self.isPointOnLineSegment(endLine[0], startLine):
            numberPointOnLineSegment += 1
        if self.isPointOnLineSegment(endLine[1], startLine):
            numberPointOnLineSegment += 1
        if numberPointOnLineSegment > 1:
            return True
        else:
            return False

    def getCoincideLineLength(self, startLine, endLine):
        if not self.isLinesCoincide(startLine, endLine):
            return 0
        point = []
        if self.isPointOnLineSegment(startLine[0], endLine):
            point.append(startLine[0])
        if self.isPointOnLineSegment(startLine[1], endLine):
            point.append(startLine[1])
        if self.isPointOnLineSegment(endLine[0], startLine):
            point.append(endLine[0])
        if self.isPointOnLineSegment(endLine[1], startLine):
            point.append(endLine[1])
        distance = []
        for index1, point1 in enumerate(point):
            for index2, point2 in enumerate(point):
                distance.append(self.getDistanceBetweenPoints(point1, point2))
        return max(distance)

    def getDistanceFromPointToLine(self, point, line, lineType='1'):
        """ax+by+c=0 line is build up with two points with lineType = '1'"""
        """ax+by+c=0 line is build up with one point and slope with lineType = '2'"""
        if lineType is '1':
            a = line[1][1] - line[0][1]
            b = line[1][0] - line[0][0]
            c = line[0][1]*line[1][0] - line[0][0]*line[1][1]
        elif lineType is '2':
            a = math.sin(line[1])
            b = math.cos(line[1])
            c = -a*line[0][0] -b*line[0][1]
        else:
            print "wrong input! (getDistanceFromPointToLine)"
            exit()

        distance = math.fabs(a*point[0]-b*point[1]+c)/math.sqrt(a**2+b**2)
        return distance

    def getDistanceBetweenPoints(self, startPoint, endPoint):
        differenceX = endPoint[0] - startPoint[0]
        differerceY = endPoint[1] - startPoint[1]
        distance = math.sqrt(differenceX**2 + differerceY**2)
        return distance

    def getPointWithLineSegment(self, startPoint, lineLength, lineAngle):
        endPointX = startPoint[0] + lineLength*math.cos(lineAngle)
        endPointY = startPoint[1] + lineLength*math.sin(lineAngle)
        endPoint = [endPointX, endPointY]
        return endPoint
 
    #return value maybe wrong
    def getLineCrossingPoint(self, line1, line2):
        k1 = (line1[1][1]-line1[0][1])/(line1[1][0]-line1[0][0])
        b1 = k1*line1[0][0] + line1[0][1]
        k2 = (line1[1][1]-line1[0][1])/(line1[1][0]-line1[0][0])
        b2 = k1*line1[0][0] + line1[0][1]
        if k1 is k2:
            print "Lines sind parallel!"
            return False
        pointX = (b2-b1)/(k1-k2)
        pointY = k1*pointX + b1
        point = [pointX, pointY]
        return point

    def isLineSegmentsCrossing(self, line1, line2):
        point = self.getLineCrossingPoint(line1, line2)
        if point is False:
            return False
        if self.isPointOnLineSegment(point, line1) and self.isPointOnLineSegment(point, line2):
            return True
        else:
            print "the crossing point is not on lines!"
            return False



if __name__ == '__main__':
    rospy.init_node('grasp_2D_parameters')
    graspParameters = grasp_2D_parameters(60, 10)
    try:
        graspParameters.determineParametersFromParallelGrasp()
    except:
        exit()
    print graspParameters.existence
    print graspParameters.gripperOriginalPoint
    print graspParameters.gripperAngleMove
    print graspParameters.evaluationValue
    rospy.spin()
