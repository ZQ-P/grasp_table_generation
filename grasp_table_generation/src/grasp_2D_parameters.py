#!/usr/bin/python

#import rospy
import math
import numpy
from gripper_shape import gripper_shape
from object_configuration import ObjectConfiguration

class grasp_2D_parameters(gripper_shape):
    def __init__(self):
        self.initGrasp2DParameters()
        self.initGrasp2DTolerances()
        self.initGrasp2DVariables()
        self.initObject2D()


    def initGrasp2DParameters(self):
        self.gripperAngleMove = {'start': 0, 'end': 0}
        self.gripperOriginalPoint = {'position': [0, 0], 'orientation': 0} 
        self.gripperSlopeAngle = 0
        self.evaluationValue = 100000
        self.existence = False

    def initGrasp2DTolerances(self):
        self.lengthLineConformMin = 1
        self.distanceObectGripperMin = 0.8
        #self.distanceLineConformMax = 0.2
        self.distancePointOnLineMax = 0.2
        self.gripperOriginalPointPositionAccuracy = 0.1
        self.gripperOriginalPointOrientationAccuracy = 0.01
        self.gripperAngleMoveAccuracy = 0.01

    def initGrasp2DVariables(self):
        self.objectShapePoints = {}
        self.gripperValidLengthFest = {}
        self.gripperValidLengthMove = {}
        self.gripperPointsPositionFest = {}
        self.gripperPointsPositionMove = {}
        self.gripperAngleMoveRoom = {'start': [0, 0], 'end': [0, 0]}
        self.gripperOriginalPointPoseRoom = {'x': [0, 0], 'y': [0, 0], 'r': [0, 0]}
        #self.gripperOriginalPointOrientationRoom = [0, 0]
        self.gripperSlopeAngleRoom = [0, 0]

    #ObjectConfiguration hier stellen oder von andere class
    def initObject2D(self):
        objectConfig = ObjectConfiguration()
        self.objectShapes2D = objectConfig.getObjectshape2D()
        self.objectShape = {'x': 0, 'y': 0}

    def initGripperEffectiveLength(self):
        for part, length in self.gripperLengthFest:
            self.gripperValidLengthFest.update({part: length*math.cos(self.gripperSlopeAngle)})
        for part, length in self.gripperLengthMove:
            self.gripperValidLengthMove.update({part: length*math.cos(self.gripperSlopeAngle)})

    ##### finished transfer to loop
    def gripperPointsPosition(self):
        #pointPrefix = ['start', 'between', 'end']
        #for index, part in enumerate(pointPrefix):
        #self.gripperPointsPositionFest.update(self.gripperOriginalPoint)
        startPointPosition = self.gripperOriginalPoint['position']
        startPointOrientation = self.gripperOriginalPoint['orientation']
        segmentLength = self.gripperLengthFest['start']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'start': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleFest['start']
        segmentLength = self.gripperLengthFest['between']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'between': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleFest['end']
        segmentLength = self.gripperLengthFest['end']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionFest.update({'end': endPointPosition})

        startPointPosition = self.gripperOriginalPoint['position']
        startPointOrientation = self.gripperOriginalPoint['orientation']
        segmentLength = self.gripperLengthMove['start']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'start': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleMove['start']
        segmentLength = self.gripperLengthMove['between']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'between': endPointPosition})
        startPointPosition = endPointPosition
        startPointOrientation += self.gripperAngleMove['end']
        segmentLength = self.gripperLengthMove['end']
        endPointPosition = self.getPointWithLineSegment(startPointPosition, segmentLength, startPointOrientation)
        self.gripperPointsPositionMove.update({'end': endPointPosition})

    #the function needs to change, receive the self.obectShape from the outside
    def getParametersFromDifferentPose(self):
        for plane, objectLengthWidth in self.objectShapes2D.items():
                self.objectShape['x'] = objectLengthWidth[0]
                self.objectShape['y'] = objectLengthWidth[1]
                self.objectShapePoints.update({'leftUp': [-0.5*self.objectShape['x'], self.objectShape['y']]})
                self.objectShapePoints.update({'leftDown': [-0.5*self.objectShape['x'], 0]})
                self.objectShapePoints.update({'rightUp': [0.5*self.objectShape['x'], self.objectShape['y']]})
                self.objectShapePoints.update({'rightDown': [0.5*self.objectShape['x'], 0]})
                self.determineParametersFromParallelGrasp()

    def determineParametersFromParallelGrasp(self):
        self.createSearchRoomFromParallelGrasp()
        self.findParametersWithExhaustiveSearch()

    def createSearchRoomFromParallelGrasp(self):
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
        self.gripperAngleMoveRoom['start'][1] = math.pi
        self.gripperAngleMoveRoom['end'][0] = math.pi/2 - 2*self.gripperOriginalPointOrientationAccuracy
        self.gripperAngleMoveRoom['end'][1] = math.pi/2 + 2*self.gripperOriginalPointOrientationAccuracy

    def findParametersWithExhaustiveSearch(self):
        self.initGrasp2DParameters()
        startX = self.gripperOriginalPointPoseRoom['x'][0]
        endX = self.gripperOriginalPointPoseRoom['x'][1]
        stepX = 0.5*self.gripperOriginalPointPositionAccuracy
        startY = self.gripperOriginalPointPoseRoom['y'][0]
        endY = self.gripperOriginalPointPoseRoom['y'][1]
        stepY = 0.5*self.gripperOriginalPointPositionAccuracy
        startR = self.gripperOriginalPointPoseRoom['r'][0]
        endR = self.gripperOriginalPointPoseRoom['r'][1]
        stepR = 0.5*self.gripperOriginalPointOrientationAccuracy
        startAS = self.gripperAngleMoveRoom['start'][0]
        endAS = self.gripperAngleMoveRoom['start'][1]
        stepAS = 0.5*self.gripperOriginalPointOrientationAccuracy
        startAE = self.gripperAngleMoveRoom['end'][0]
        startAE = self.gripperAngleMoveRoom['end'][1]
        stepAE = 0.5*self.gripperOriginalPointOrientationAccuracy
        point = None
        angle = None
        for self.gripperOriginalPoint['position'][0] in numpy.arange(startX, endX, stepX):
            for self.gripperOriginalPoint['position'][1] in numpy.arange(startY, endY, stepY):
                for self.gripperOriginalPoint['orientation'] in numpy.arange(startR, endR, stepR):
                    for self.gripperAngleMove['start'] in numpy.arange(startAS, endAS, stepAS):
                        for self.gripperAngleMove['end'] in numpy.arange(startAE, endAE, stepAE):
                            if self.conformToRestrictionFromParallelGrasp():
                                continue
                            evalValue = self.calculateCriteriaFromParallelGrasp()
                            if evalValue > self.evaluationValue:
                                continue
                            point = self.gripperOriginalPoint
                            angle = self.gripperAngleMove

        if point is not None:
            self.existence = True
            self.gripperOriginalPoint = point
            self.gripperAngleMove = angle




    # restriction of obect shape have not been considered yet.
    def conformToRestrictionFromParallelGrasp(self):
        self.gripperPointsPosition()
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
        if (self.gripperAngleMove['start'] < angleMax) and (self.gripperAngleMove['start'] > angleMin):
            distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        else:
            distance = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightDown'])
        if distance < self.lengthLineConformMin:
            return False

        return True


    def calculateCriteriaFromParallelGrasp(self):
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineGripperMove = [self.gripperPointsPositionMove['between'], self.gripperPointsPositionMove['end']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        lineObjectRight = [self.objectShapePoints['rightUp'], self.objectShapePoints['rightDown']]
        L1 = self.getCoincideLineLength(lineGripperFest, lineObjectLeft)
        L2 = self.getCoincideLineLength(lineGripperMove, lineObjectRight)
        L3 = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        L4 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightDown'])
        criteria1 = (L1+L2)/(self.objectShape['x']+self.objectShape['y'])
        criteria2 = math.fabs(L1-L2)/max(self.gripperLengthFest['end'], self.gripperLengthMove['end'])
        criteria3 = (L1+L2)/(self.gripperLengthFest['end']+self.gripperLengthMove['end'])
        criteria4 = (self.objectShape['y'] - (L1+L2)/2)/self.objectShape['y']
        evalValue = -criteria1 + criteria2 + criteria3 + criteria4
        return evalValue

    def getGripperTotalFestLengthX(self):
        theta = self.gripperOriginalPoint['orientation']
        startLengthX = self.gripperLengthFest['start']*math.sin(theta-math.pi/2)
        theta += self.gripperAngleFest['start']
        betweenLengthX = sel.gripperLengthFest['between']*math.sin(theta-math.pi/2)
        theta += self.gripperAngleFest['end']
        endLengthX = self.gripperLengthFest['end']*math.sin(theta-math.pi/2)
        totalLengthX = startLengthX + betweenLengthX + endLengthX
        return totalLengthX

    def getGripperTotalFestLengthY(self):
        theta = self.gripperOriginalPoint['orientation']
        startLengthY = self.gripperLengthFest['start']*math.cos(theta-math.pi/2)
        theta += self.gripperAngleFest['start']
        betweenLengthY = sel.gripperLengthFest['between']*math.cos(theta-math.pi/2)
        theta += self.gripperAngleFest['end']
        endLengthY = self.gripperLengthFest['end']*math.cos(theta-math.pi/2)
        totalLengthY = startLengthY + betweenLengthY + endLengthY
        return totalLengthY


    def determineParametersFromPlanarPointContact(self):
        self.createSearchRoomFromPlanarPointContact()
        self.findParametersWithExhaustiveSearch()

    def createSearchRoomFromPlanarPointContact(self):
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

    # restriction of obect shape have not been considered yet.
    def conformToRestrictionFromPlanarPointContact(self):
        self.gripperPointsPosition()
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
        lineGripperFest = [self.gripperPointsPositionFest['between'], self.gripperPointsPositionFest['end']]
        lineObjectLeft = [self.objectShapePoints['leftUp'], self.objectShapePoints['leftDown']]
        slopeGripperFestEnd = self.gripperOriginalPoint['orientation'] + self.gripperAngleFest['start'] + self.gripperAngleFest['end']
        objectCenterPoint = [0, self.objectShape['y']/2]
        L1 = self.getCoincideLineLength(lineGripperFest, lineObjectLeft)
        L2 = self.getDistanceBetweenPoints(self.gripperPointsPositionFest['between'], self.objectShapePoints['leftDown'])
        L3 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['start'], self.objectShapePoints['rightDown'])
        L4 = self.getDistanceBetweenPoints(self.gripperPointsPositionMove['between'], self.objectShapePoints['rightUp'])
        line5 = [[-0.5*self.objectShape[x], 0.5*L1], slopeGripperFestEnd]
        L5 = self.getDistanceFromPointToLine(objectCenterPoint, line5, '2')
        line6 = [self.objectShapePoints['rightDown'], self.gripperAngleMove['start']]
        L6 = self.getDistanceFromPointToLine(objectCenterPoint, line6, '2')
        line7 = [self.objectShapePoints['rightUp'], self.gripperAngleMove['end']]
        L7 = self.getDistanceFromPointToLine(objectCenterPoint, line7, '2')
        L8 = self.objectShape['y']/2 - L1/2
        criteria1 = (L5+L6+L7)/((self.objectShape['x']+self.objectShape['y'])/2)
        criteria2 = 2*L8/self.objectShape['y']
        criteria3 = (L2 + L3 + 0.5*L4)/max(self.gripperLengthFest['end'], self.gripperLengthMove['between'], self.gripperLengthMove['end'])
        criteria4 = L1/self.objectShape['y']
        evalValue = criteria1 + criteria2 + criteria3 - criteria4
        return evalValue





    def determineParametersFromPointContacts(self):
        return

    def ConformToRestrictionFromPointContacts(self):
        return

    def calculateCriteriaFromPointContacts(self):
        return




    def determineParametersFromSlopeGrasp(self):
        return

    def ConformToRestrictionFromSlopeGrasp(self):
        return

    def createSearchRoomFromContacts(self):
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
        xMin = min(lineSegment[0][0], lineSegment[1][0])
        xMax = max(lineSegment[0][0], lineSegment[1][0])
        yMin = min(lineSegment[0][1], lineSegment[1][1])
        yMax = max(lineSegment[0][1], lineSegment[1][1])
        if ((point[0] >= xMin)or(point[0] <= xMax)) and ((point[1] >= yMin)or(point[1] <= yMax)):
            return True
        else:
            return False

    def isLinesCoincide(self, startLine, endLine):
        start1 = self.isPointOnLineSegment(startLine[0], endLine)
        start2 = self.isPointOnLineSegment(startLine[1], endLine)
        end1 = self.isPointOnLineSegment(endLine[0], startLine)
        end2 = self.isPointOnLineSegment(endLine[1], startLine)
        if (start1 or start2) and (end1 or end2):
            return True
        else:
            return False

    def getCoincideLineLength(self, startLine, endLine):
        if not self.isLinesCoincide(startLine, endLinestartLine, endLine):
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

    def getDistanceFromPointToLine(self, point, line, type='1'):
        """ax+by+c=0 line is build up with two points with type = '1'"""
        """ax+by+c=0 line is build up with one point and slope with type = '2'"""
        if type is '1':
            a = line[1][1] - line[0][1]
            b = line[0][0] - line[1][0]
            c = line[0][1]*(line[1][0]-line[0][0]) - line[0][0]*(line[1][1]-line[0][1])
        elif type is '2':
            a = math.sin(line[1])
            b = math.cos(line[1])
            c = -a*line[0][0] -b*line[0][1]
        else:
            print "wrong input! (getDistanceFromPointToLine)"
            exit()

        distance = math.fabs(a*point[0]+b*point[1]+c)/math.sqrt(a**2+b**2)
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

    def isLineSegmentsCrossing(self, line1, line2):
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
        if self.isPointOnLineSegment(point, line1) and self.isPointOnLineSegment(point, line2):
            return True
        else:
            print "the crossing point is not on lines!"
            return False

