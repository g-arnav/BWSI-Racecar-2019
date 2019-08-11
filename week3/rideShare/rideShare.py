# /usr/bin/env python

# import libraries
from tf.transformations import euler_from_quaternion
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import math

AUTONOMOUS_MODE = True
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"
POSE_TOPIC = '/pf/viz/inferred_pose'
COORDINATE_CONSTANT = 0.3


# PoseStamped format:
# pose:
#  position:
#    X
#    y
#    z
#  orientation:
#    X
#    y
#    z
#    w
#

class rideshareDrive(object):
    def __init__(self):
        rospy.init_node("rideshare")
        # initialize publishers and subscribers
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.local_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.localize)


        # initialize cmd object
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0
        self.state = 0
        self.functions = ["self.findARTag()", "self.reachDestination()"]

        self.currentPosition = []
        self.reachedInterPoint = False
        self.detectedTag = None

        self.landmarks = {"start": (2.14, -3.26),
                          "ARTags": (3.39, -0.86),
                          "interPoint": (2.59, -1.79),
                          "house1": (1.33, -3.12),
                          "house2": (-0.40, -2.70),
                          "house5": (0.04, -0.84)}

        self.goalPosition = self.landmarks["interPoint"]


        self.lidarData = None

    def approximateCoordinates(self, coords):
        firstVertex = ((coords[0] + COORDINATE_CONSTANT), (coords[1] + COORDINATE_CONSTANT))
        secondVertex = ((coords[0] - COORDINATE_CONSTANT), (coords[1] - COORDINATE_CONSTANT))
        return (firstVertex, secondVertex)

    def checkCoordinates(self, coordsCurrent, coordsDestination):
        approxCoords = self.approximateCoordinates(coordsDestination)
        toReturn = False
        if coordsCurrent.x > approxCoords[1][0] and coordsCurrent.x < approxCoords[0][0]:
            if coordsCurrent.y > approxCoords[1][1] and coordsCurrent.y < approxCoords[0][1]:
                toReturn = True
        return toReturn

    def driveCallback(self, data):  # finds the state of the car

        self.lidarData = data

        exec (self.functions[self.state])

    def findARTag(self):  # state when car looks for AR tags
        while self.currentPosition == []:
            print "No position yet"
            time.sleep(0.5)
        ARTags = self.landmarks["ARTags"]
        interPoint = self.landmarks['interPoint']
        if not self.checkCoordinates(self.currentPosition, ARTags) and self.reachedInterPoint == False:
            self.goalPosition = interPoint
            if self.checkCoordinates(self.currentPosition, interPoint):
                self.goalPosition = ARTags
                self.reachedInterPoint = True
        elif not self.checkCoordinates(self.currentPosition, ARTags):  # and self.reachedInterPoint == True
            self.goalPosition = ARTags
        else:  # self.currentPosition == ARTags
            self.detectedTag = arCallback()
            self.goalPosition = self.landmarks["house" + str(self.detectedTag)]
            self.state = 1
        print self.goalPosition, self.state,
        self.potentialField()

    def reachDestination(self):  # state when car looks for destination (based on AR tags)
        if checkCoordinates(self.currentPosition, self.goalPosition):
            print(str(self.goalPosition) + " reached!")
            self.reachedInterPoint = False
            self.state = 0
        else:
            self.potentialField()

    def localize(self, data):  # callback function to save inferred pose based on localization
        self.pose = data.pose
        self.currentPosition = self.pose.position
        x = self.pose.orientation.x
        y = self.pose.orientation.y
        z = self.pose.orientation.z
        w = self.pose.orientation.w

        self.angle = euler_from_quaternion((x, y, z, w))[2]

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        # TODO: Write your state changes here
        ret = list()
        for i in range(0, len(tags.markers)):
            ret.append(tags.markers[i].id)
        self.arStates = ret
        return self.arStates

    def drive(self, spd, angle):

        self.cmd.drive.speed = spd
        self.cmd.drive.steering_angle = angle

        self.drive_pub.publish(self.cmd)

    def potentialField(self):  # potential field with attraction to goal point

        def shiftGoal(self, goal):  # changes goal coordinates from being relative to the map to being relative to the car
            shifted = [goal[0] - self.pose.position.x, goal[1] - self.pose.position.y]
            theta = math.atan(shifted[0] / shifted[1]) + (math.pi - self.angle)
            distance = (shifted[0] ** 2 + shifted[1] ** 2) ** 0.5
            shifted[0] = distance * math.cos(theta)
            shifted[1] = distance * math.sin(theta)

            print shifted,

            return shifted

        def convertPoints(data, shiftedGoal):
            '''Convert all current LIDAR data to cartesian coordinates'''

            cartPointsX = []
            cartPointsY = []

            for i in range(1080):
                cartPointsX.append(data.ranges[i] * math.cos((i * 270 / 1080 - 45) * 6.28 / 360))
                cartPointsY.append(data.ranges[i] * math.sin((i * 270 / 1080 - 45) * 6.28 / 360))
            # cartPointsX.append(-shiftedGoal[0])
            # cartPointsY.append(-shiftedGoal[1])
            return cartPointsX, cartPointsY

        def calcFinalVector(cartPointsX, cartPointsY):
            '''Calculate the final driving speed and angle'''
            hor = 0
            ver = 0
            for i in range(1080):
                ver += cartPointsY[i]
                hor += cartPointsX[i]
            if hor > 0:
                return [0.5, math.atan(ver / hor) * 360 / 6.28]
            elif hor < 0:
                return [0.5, 180 + math.atan(ver / hor) * 360 / 6.28]

        def drive_callback(finalVector):
            '''Publishes drive commands'''
            speed = 5
            angle = -(90 - finalVector[1]) / 19.48
            angle /= 5
            # asdsfgsfg
            if abs(angle) < 0.03:
                angle = 0
            return speed, angle

        goal = self.goalPosition

        data = self.lidarData
        shiftedGoal = shiftGoal(self, goal)

        x, y = convertPoints(data, shiftedGoal)
        finalVector = calcFinalVector(x, y)
        spd, angle = drive_callback(finalVector)
        print finalVector, angle


        self.drive(0.5, angle)


def main():
    try:
        ic = rideshareDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            pass
            # ic.drive_pub.publish(ic.cmd)
    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()
