#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from color_segmentation import *
import time
from math import *

class Navigation:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT

    kmax = 1.5
    carCharge = .9
    jetCharge = 2
    wallCharge = 0.8
    lidarPoints = 1080

    initSpeed = 5
    steeringFactor = .002  # 0.009
    speedFactor = 1.2  # 1.3


    def __init__(self):
        # Initialize your publishers and
        # subscroribers

        self.data = None

        self.lidar = None
        self.derivs = [0] * 1080

        self.angle = 0
        self.spd = 0

        self.objects = []  # [lidar index, distance]

        self.turn_thresh = 1
        self.smooth_thresh = 0.1


        self.goal = []  # [distance, theta]

        self.current_speed = self.initSpeed
        # x (steering)
        self.iHat = []
        # y (speed)
        self.jHat = []
        self.starttime = time.time()
        # [speed, angle]
        self.finalVector = [0, 0.5]

        self.lineprev = 0




    def find_goals(self, spd):

        self.derivs = [self.lidar[i] - self.lidar[i + 1] for i in range(len(self.lidar) - 1)]

        if self.lidar == None:
            return

        g = []

        for i in range(len(self.lidar) - 1):

            data = self.lidar[i]

            der = self.derivs[i]

            if data > 60:
                if i == 0:
                    self.lidar[i] = 0
                else:
                    self.lidar[i] = self.lidar[i-1]
                data = self.lidar[i]

            if abs(der) > self.turn_thresh:
                sign = abs(der) / der
                check_width = max(80 - (data * 12), 20)


                for j in range(i + 1, min(int(i + check_width), len(self.derivs))):

                    if self.derivs[j] != 0:

                        if abs(self.derivs[j]) / self.derivs[j] != sign and abs(self.lidar[j] - data) < self.smooth_thresh:

                            self.lidar[i:j] = [data] * (j-i)
                            self.derivs[i:j] = [0] * (j-i)

        #     der = self.derivs[i]
        #
        #     if abs(der) > self.turn_thresh:
        #         sign = abs(der) / der
        #
        #         theta = (i * self.data.angle_increment) + self.data.angle_min
        #
        #         x = data * np.sin(theta)
        #         y = data * np.cos(theta)
        #
        #         x += sign * self.DESIRED_DISTANCE
        #
        #         print(data, theta)
        #
        #         g.append([(x**2 + y**2) ** .5, np.arctan(x/y) / np.pi, max(data, self.lidar[i + 1])])
        #
        # goal = [i for i, lis in enumerate(g) if max(v[2] for v in g) in lis]
        #
        # print(self.lidar.index(max(self.lidar)), self.lidar[self.lidar.index(max(self.lidar)) - 1], max(self.lidar), self.lidar[self.lidar.index(max(self.lidar)) + 1], self.derivs[self.lidar.index(max(self.lidar))])


        sorte = self.lidar

        sorte.sort(reverse=True)
        k = 0

        theta = 0

        while k < 50:
            ind = self.data.ranges.index(sorte[k])
            print(ind, self.data.ranges[ind],  self.data.ranges[ind + 1], self.data.ranges[ind - 1])

            if ind != 0 and ind != len(sorte):
                if self.data.ranges[ind] - self.data.ranges[ind + 1] > self.data.ranges[ind] - self.data.ranges[ind - 1] and self.data.ranges[ind] - self.data.ranges[ind + 1] > self.turn_thresh:
                    print(ind, 'left')
                    ind += 50
                    theta = (ind * self.data.angle_increment) + self.data.angle_min
                    break

                elif self.data.ranges[ind] - self.data.ranges[ind - 1] > self.data.ranges[ind] - self.data.ranges[ind + 1] and self.data.ranges[ind] - self.data.ranges[ind - 1] > self.turn_thresh:
                    print(ind, 'right')
                    ind -= 50
                    theta = (ind * self.data.angle_increment) + self.data.angle_min
                    break
            k += 1

        self.angle = theta






        # if g == []:
        #     self.angle = 0
        # else:
        #     goal = g[goal[0]][1]
        #     self.angle = goal

        self.spd = 1.5




    def wall_follower(self, spd, dist, side=0):


        if self.data == None:
            return 0

        left_distance = self.lidar[len(self.lidar) * 16/100]
        right_distance = self.lidar[len(self.lidar) * (99 - 16) /100]

        distances = []
        indexes = []

        if (left_distance < right_distance and side == 0) or side == -1:
            average = np.mean(self.lidar[0:int(len(self.lidar) * 2/5)])
            for i in range(0, int(len(self.lidar) * 2/5)):
                if self.lidar[i] < 2 * average:
                    distances.append(self.lidar[i] * -np.sin((i * self.data.angle_increment) + self.data.angle_min))
                    indexes.append(self.lidar[i] * -np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = 1
        else:
            average = np.mean(self.lidar[int(len(self.lidar) * 3/5):len(self.lidar)])

            for i in range(int(len(self.lidar) * 3/5), len(self.lidar)):
                if self.lidar[i] < 2 * average:

                    distances.append(self.lidar[i] * np.sin((i * self.data.angle_increment) + self.data.angle_min))
                    indexes.append(self.lidar[i] * np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = -1

        # alt_m, alt_b = self.ransac(distances, indexes)

        m, b = self.regression(distances, indexes)
        # rospy.loginfo("ransac: m: {} b: {} reg: m: {} b: {}".format(alt_m, alt_b, m, b))
        if not -1.5 < m < 1.5:
            return 0

        min_left = min(self.lidar[0:len(self.lidar) * 2/5])
        min_right = min(self.lidar[len(self.lidar) * 3/5:len(self.lidar)])

        # self.DESIRED_DISTANCE = min_left / 2 + min_right / 2
        #
        # self.DESIRED_DISTANCE = min(0.5, self.DESIRED_DISTANCE)

        p = -wall * (b - dist)

        f_distances = []
        f_indexes = []

        average = np.mean(self.lidar[int(len(self.lidar) * 2 / 5):int(len(self.lidar) * 3 / 5)])
        # for i in range(int(len(self.lidar) * 2 / 5), int(len(self.lidar) * 3 / 5)):
        #     if self.lidar[i] < 2 * average:
        #         f_distances.append(self.lidar[i] * -np.sin((i * self.data.angle_increment) + self.data.angle_min + np.pi/2))
        #         f_indexes.append(self.lidar[i] * -np.cos((i * self.data.angle_increment) + self.data.angle_min + np.pi/2))
        #
        # f_m, f_b = self.regression(f_distances, f_indexes)

        # rospy.loginfo(str(f_b))

        if average < 0.75:
            # print("wall")
            self.spd = spd

            if left_distance < right_distance:
                self.angle = 1
            else:
                self.angle = -1
            return


        p_angle = (m * 0.5) + (p * .5)
        tempAngle = (p_angle) #+ (d_angle*.5)

        if abs(tempAngle) < .03:
            tempAngle = 0

        # rospy.loginfo("m: {} p: {} actual: {}".format(m, p, tempAngle))

        self.spd = 1.5
        self.angle = tempAngle

    def potential_fields(self, spd):


        self.iHat = []
        self.jHat = []
        for i, item in enumerate(self.lidar):
            angle = float(i) / self.lidarPoints * (3 * pi / 2) - (pi / 4)
            k = self.kmax - exp(abs((pi / 2) - angle)) * 0.2
            maga = -k * (self.carCharge * self.wallCharge) / item
            self.iHat.append(-maga * cos(angle))
            self.jHat.append(maga * sin(angle))

        self.finalVector = [sum(self.iHat), sum(self.jHat) + (k * self.jetCharge)]
        # print(self.finalVector)

        self.angle = self.steeringFactor * self.finalVector[0]
        self.spd = spd

    def line_follower(self, spd, img):
        LOOKING_FOR = 'RED'
        ANGLE_CONSTANT = 0.1
        THRESHOLD_AREA = 5000
        RED = np.array([[0, 44, 147], [8, 254, 255]])
        flag_box, imagePush, imageHSV, _ = cd_color_segmentation(img[int(img.shape[0] * 0.33):img.shape[0]], np.array([0, 44, 147]), np.array([8, 254, 255]))

        # print(str((self.flag_box[1][0] - self.flag_box[0][0]) * (self.flag_box[1][1] - self.flag_box[0][1])))
        if flag_box is None:
            print('no red detected')

        areaOfBox = (flag_box[1][0] - flag_box[0][0]) * (flag_box[1][1] - flag_box[0][1])
        the_boolean = flag_box is None or areaOfBox < THRESHOLD_AREA
        if LOOKING_FOR == 'RED' and the_boolean:
            flag_box, imagePush, imageHSV, _ = cd_color_segmentation(img[int(img.shape[0] * 0.33):img.shape[0]], np.array([0, 44, 147]), np.array([8, 254, 255]))
            print('red boi')


        CENTER = imagePush.shape[1] / 2

        xAvg = (flag_box[0][0] + flag_box[1][0]) / 2
        # print('avg: ' + str(xAvg))
        error = xAvg - CENTER
        turn_angle = -1 * error * ANGLE_CONSTANT * np.pi / 180
        print('turn angle: ' + str(turn_angle))

        if turn_angle < -1:
            self.angle = -1
        # print('turning right')
        elif turn_angle > 1:
            self.angle = 1
        # print('turning left')
        else:
            self.angle = turn_angle

        if areaOfBox < 50:
            self.angle = self.lineprev
        else:
            self.lineprev = self.angle

        self.spd = spd

    def regression(self, x, y):
        x = np.array(x)
        y = np.array(y)

        A = np.vstack([y, np.ones(len(y))]).T
        m, b = np.linalg.lstsq(A, x)[0]

        return m, b

    def network(self):
        pass





"""Lidar data is now stored in self.data, which can be accessed
using self.data.ranges (in simulation, returns an array).
Lidar data at an index is the distance to the nearest detected object
self.data.ranges[0] gives the leftmost lidar point
self.data.ranges[100] gives the rightmost lidar point
self.data.ranges[50] gives the forward lidar point
"""
# returns the output of your alg, the new angle to drive in