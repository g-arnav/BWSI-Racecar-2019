#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time

class PotentialField:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")

    def __init__(self):
        self.data = None
        self.data_len = None
        self.cmd = AckermannDriveStamped()

        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        #cartesian points -- to be filled (tuples)
        self.cartPoints = None

        #[speed, angle]
        self.finalVector = [0.5, 0]

    def scan(self, data):
        self.data = data
        self.data_len = len(data.ranges)

        self.cartPoints = [None for x in range(self.data_len + 1)]

        self.drive()


    def drive(self):
        '''Publishes drive commands'''
        self.speed, self.angle = self.controller()

        rospy.loginfo("speed: {} angle: {}".format(self.speed, self.angle))


        if self.speed < 0:
            self.cmd.drive.speed = -0.5
            self.cmd.drive.steering_angle = - self.angle

            self.drive_pub.publish(self.cmd)
            time.sleep(0.5)

        else:

            self.cmd.drive.speed = self.speed
            self.cmd.drive.steering_angle = self.angle

            self.drive_pub.publish(self.cmd)

    def controller(self):
        self.convertPoints()
        self.calcFinalVector(self.cartPoints)
        x = self.finalVector[0]
        y = self.finalVector[1]

        angle = np.arctan((x/y))
        angle = -np.rad2deg(angle) / 45

        # speed = (y + 1000) / 250

        # speed = -100/y + 1
        speed = 5

        return y/20, x/10


    def convertPoints(self):
        '''Convert all current LIDAR data to cartesian coordinates'''
        for i in range(self.data_len):
            x = self.data.ranges[i] * np.sin((i * self.data.angle_increment) + self.data.angle_min)
            y = self.data.ranges[i] * np.cos((i * self.data.angle_increment) + self.data.angle_min)
            # rospy.loginfo(str(np.rad2deg((i * self.data.angle_increment) + self.data.angle_min)))
            self.cartPoints[i] = (x, y)
            # rospy.loginfo(str(self.cartPoints[i]))
        self.cartPoints[-1] = (0, -.01)


    def calcFinalVector(self, points):
        '''Calculate the final driving speed and angle'''
        vector = [0, 0]
        for i in range(len(points)):
            x = points[i][0]
            y = points[i][1]
            mag = ((x**2) + (y**2)) ** 0.5
            if x != 0:
                vector[0] += -(x/(mag ** 2))
            vector[1] += -(y/(mag ** 2))
            # rospy.loginfo("mag: {} x: {} y: {} vec_x: {} vec_y: {}".format(mag, x, y, -(x/(mag ** 2)), -(y/(mag ** 2))))
        self.finalVector = vector





if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()
