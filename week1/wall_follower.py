#!/usr/bin/env python
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from time import time

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    DESIRED_DISTANCE = 1

    def __init__(self):
        # Initialize your publishers and
        # subscroribers

        self.data = None
        self.angle = 0

        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        self.wall = 0

        self.time = time()
        self.prev_m = 0
        self.prev_p = 0
        # self.cmd.drive.speed = 1



    def scan(self, data):
        # stores the lidar data so you can work with it
        self.data = data
        # calls function that controls driving
        self.drive()


    def drive(self):
        """controls driving"""


        # gets the angle required
        self.angle = self.simple_PD()
        # self.angle = .33
        # sets speed and driving angle
        # self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.speed = 100
        self.cmd.drive.steering_angle = self.angle
        # publishes the command
        self.drive_pub.publish(self.cmd)


    def find_wall(self):


        # if lidar data has not been received, do nothing
        if self.data == None:
            return 0
        ## TO DO: Find Alg for Wall Following ##


        left_distance = self.data.ranges[16]
        right_distance = self.data.ranges[100 - 16]
        self.DESIRED_DISTANCE = left_distance / 2 + right_distance/2
        if left_distance < right_distance:
            low_position = max(-left_distance, -2 * self.DESIRED_DISTANCE) + self.DESIRED_DISTANCE

            # if self.data.ranges.index(min(self.data.ranges[0:60])) > 40:
            #     a_angle = 45
            # else:
            low_index = self.data.ranges.index(min(self.data.ranges[0:20]))
            low_angle = np.rad2deg(self.data.angle_min) + (low_index + 1) * np.rad2deg(self.data.angle_increment)
            a_angle = 90 + low_angle

        elif left_distance > right_distance:
            low_position = min(right_distance, 2 * self.DESIRED_DISTANCE) - self.DESIRED_DISTANCE

            # if self.data.ranges.index(min(self.data.ranges[40:100])) < 60:
            #     a_angle = -45
            # else:
            low_index = self.data.ranges.index(min(self.data.ranges[80:100]))
            low_angle = np.rad2deg(self.data.angle_min) + (low_index + 1) * np.rad2deg(self.data.angle_increment)
            a_angle = low_angle - 90

        p_angle = low_position * 90

        # if 40 < low_index < 60:
        #     tempAngle = (a_angle * 0.0) + (p_angle * 1.0)
        #     rospy.loginfo('wall ahead').
        # else:

        # if 40 < self.data.ranges.index(min(self.data.ranges[0:100])) < 60:
        #     if left_distance < right_distance:
        #         tempAngle = 60
        #     else:
        #         tempAngle = -60


        if self.data.ranges[50] < 4:
            if left_distance < right_distance:
                tempAngle = (1 - self.data.ranges[50]/4) * 90
            else:
                tempAngle = -(1 - self.data.ranges[50]/4) * 90
        else:

            tempAngle = (a_angle * 0.5) + (p_angle * 0.5)

        rospy.loginfo("p: {} a: {} t: {}".format(p_angle, a_angle, tempAngle))


        return tempAngle

    def simple_PD(self):
        if self.data == None:
            return 0

        left_distance = self.data.ranges[len(self.data.ranges) * 16/100]
        right_distance = self.data.ranges[len(self.data.ranges) * (99 - 16) /100]

        distances = []
        indexes = []

        if left_distance < right_distance:
            average = np.mean(self.data.ranges[0:int(len(self.data.ranges) * 2/5)])
            for i in range(0, int(len(self.data.ranges) * 2/5)):
                if self.data.ranges[i] < 2 * average:
                    distances.append(self.data.ranges[i] * -np.sin((i * self.data.angle_increment) + self.data.angle_min))
                    indexes.append(self.data.ranges[i] * -np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = 1
        else:
            average = np.mean(self.data.ranges[int(len(self.data.ranges) * 3/5):len(self.data.ranges)])

            for i in range(int(len(self.data.ranges) * 3/5), len(self.data.ranges)):
                if self.data.ranges[i] < 2 * average:

                    distances.append(self.data.ranges[i] * np.sin((i * self.data.angle_increment) + self.data.angle_min))
                    indexes.append(self.data.ranges[i] * np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = -1

        # alt_m, alt_b = self.ransac(distances, indexes)

        m, b = self.regression(distances, indexes)
        # rospy.loginfo("ransac: m: {} b: {} reg: m: {} b: {}".format(alt_m, alt_b, m, b))
        if not -1.5 < m < 1.5:
            return 0

        min_left = min(self.data.ranges[0:len(self.data.ranges) * 2/5])
        min_right = min(self.data.ranges[len(self.data.ranges) * 3/5:len(self.data.ranges)])

        self.DESIRED_DISTANCE = min_left / 2 + min_right / 2

        self.DESIRED_DISTANCE = min(0.5, self.DESIRED_DISTANCE)

        p = -wall * (b - self.DESIRED_DISTANCE)

        f_distances = []
        f_indexes = []

        average = np.mean(self.data.ranges[int(len(self.data.ranges) * 2 / 5):int(len(self.data.ranges) * 3 / 5)])
        for i in range(int(len(self.data.ranges) * 2 / 5), int(len(self.data.ranges) * 3 / 5)):
            if self.data.ranges[i] < 2 * average:
                f_distances.append(self.data.ranges[i] * -np.sin((i * self.data.angle_increment) + self.data.angle_min + np.pi/2))
                f_indexes.append(self.data.ranges[i] * -np.cos((i * self.data.angle_increment) + self.data.angle_min + np.pi/2))

        f_m, f_b = self.regression(f_distances, f_indexes)

        rospy.loginfo(str(f_b))

        if -f_b < self.DESIRED_DISTANCE * 2:
            rospy.loginfo("wall")
            return wall

        iter_time = time() - self.time
        der_m = (m - self.prev_m) / iter_time
        der_p = (p - self.prev_p) / iter_time


        p_angle = (m * 0.5) + (p * .5)
        d_angle = (der_m * .5) + (der_p * .5)
        tempAngle = (p_angle*.5) #+ (d_angle*.5)

        if abs(tempAngle) < .03:
            tempAngle = 0

        rospy.loginfo("m: {} p: {} actual: {}".format(d_angle, p_angle, tempAngle))

        return tempAngle

    def ran_PD(self):
        if self.data == None:
            return 0

        left_distance = self.data.ranges[len(self.data.ranges) * 16/100]
        right_distance = self.data.ranges[len(self.data.ranges) * (99 - 16) /100]

        distances = []
        indexes = []

        if left_distance < right_distance:
            for i in range(0, int(len(self.data.ranges) * 2/5)):
                distances.append(self.data.ranges[i] * -np.sin((i * self.data.angle_increment) + self.data.angle_min))
                indexes.append(self.data.ranges[i] * -np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = 1
        else:

            for i in range(int(len(self.data.ranges) * 3/5), len(self.data.ranges)):
                distances.append(self.data.ranges[i] * np.sin((i * self.data.angle_increment) + self.data.angle_min))
                indexes.append(self.data.ranges[i] * np.cos((i * self.data.angle_increment) + self.data.angle_min))
            wall = -1

        m, b = self.ransac(distances, indexes)


        if not -1.5 < m < 1.5:
            return 0

        p = -wall * (b - self.DESIRED_DISTANCE)


        if self.data.ranges[len(self.data.ranges) / 2] < self.DESIRED_DISTANCE * 2:
            rospy.loginfo("wall")
            return wall

        iter_time = time() - self.time
        der_m = (m - self.prev_m) / iter_time
        der_p = (p - self.prev_p) / iter_time

        tempAngle = (0.5 * ((m * 0.5) + (p * .5))) + (0.5 * ((der_m * .5) + (der_p * .5)))

        if abs(tempAngle) < .03:
            tempAngle = 0

        rospy.loginfo("m: {} p: {} actual: {}".format(m, p, tempAngle))

        return tempAngle

    def regression(self, x, y):
        x = np.array(x)
        y = np.array(y)

        A = np.vstack([y, np.ones(len(y))]).T
        m, b = np.linalg.lstsq(A, x)[0]

        return m, b

    def ransac(self, x, y, iterations=200, n=0.5, t=2, d=0.2):

        s = time()
        data_len = len(x)
        n *= data_len
        d *= data_len

        points = []
        for i in range(data_len):
            points.append((x[i], y[i]))

        best_m = None
        best_b = None

        best_error = 100000

        iter = 0
        while iter < iterations:
            rand_ind = np.random.randint(0, data_len, int(n))
            maybe_x = []
            maybe_y = []
            for i in rand_ind:
                maybe_x.append(x[i])
                maybe_y.append(y[i])
            m, b = self.regression(maybe_x, maybe_y)
            inliers = []
            l = list(range(0, data_len))
            for i in rand_ind:
                l[i] = False
            for i in l:
                if i:
                    if self.p2l_dist(m, b, x[i], y[i]) < t:
                        inliers.append(i)

            if len(inliers) > int(d):
                for i in inliers:
                    maybe_x.append(x[i])
                    maybe_y.append(y[i])
                bet_m, bet_b = self.regression(maybe_x, maybe_y)
                error = 0
                for i in range(data_len):
                    error += self.p2l_dist(bet_m, bet_b, x[i], y[i])
                if error < best_error:
                    best_error = error
                    best_m, best_b = bet_m, bet_b

            iter += 1
        rospy.loginfo(str(best_error))

        rospy.loginfo(str("time: " + str(time() - s)))
        return best_m, best_b


    def p2l_dist(self, m, b, x, y):
        return np.abs(-m*x + y - b) / (((-m)**2 + 1)**.5)










"""Lidar data is now stored in self.data, which can be accessed
using self.data.ranges (in simulation, returns an array).
Lidar data at an index is the distance to the nearest detected object
self.data.ranges[0] gives the leftmost lidar point
self.data.ranges[100] gives the rightmost lidar point
self.data.ranges[50] gives the forward lidar point
"""
# returns the output of your alg, the new angle to drive in

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()