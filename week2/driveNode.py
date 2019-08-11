# /usr/bin/env python

# import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation, signIdentify
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
from native_model import forward
# from turnRectStarter import sift_det

import datetime

# initalize global variables

AUTONOMOUS_MODE = True

# What should your drive topic be?
DRIVE_TOPIC = '/drive'



class driveStop(object):
    """class that will help the robot drive and stop at certain conditions
    """

    def __init__(self):
        """initalize the node"""


        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

        self.img_pub = rospy.Subscriber

        """ initialize the box dimensions"""

        """driving code"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0

        """get the camera data from the class Zed_converter in Zed"""

        self.bridge = CvBridge()



        self.camera_data = Zed_converter(False, save_image=False)

        self.min_value = 0

        self.img_pub = rospy.Publisher('/zed/zed_node/boxes', Image)
        self.msg = Image

        self.c_low_range = np.array([0, 200, 200])
        self.c_high_range = np.array([100, 255, 255])

        self.s_low_range = np.array([0, 0, 0])
        self.s_high_range = np.array([100, 100, 255])

        self.mode = 0

        self.signs = []

        self.dir = -1


    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        ((xL, yL), (xR, yR)) = self.flag_box
        height = abs(yL - yR)
        width = abs(xL - xR)
        center = (height/2 + yR, width/2 + xL)
        return (height, width), center

    def driveStop_car_callback(self, data):
        """laser scan callback function"""
        # checks if the image is valid first

        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")


        # sift = sift_det('./images/oneway.jpg', self.camera_data.cv_image)
        #
        # print(sift)

        if self.mode == 0:

            spd = 1
            angle = 0

            d_front = sum(data.ranges[520:560]) / 40

            print(d_front)

            if d_front < 2.9:
                self.mode = 1

            self.drive(spd, angle)

        elif self.mode == 1:

            spd = 1
            angle = 0

            d_front = sum(data.ranges[520:560]) / 40

            dir = self.detect_sign()


            print(d_front, dir)

            if dir == 'left':
                self.signs.append(0)
            else:
                self.signs.append(1)

            if d_front < 2.8:

                if float(sum(self.signs)) / len(self.signs) < 0.5:
                    self.dir = 1
                else:
                    self.dir = -1

                self.mode = 2

            self.drive(spd, angle)

        elif self.mode == 2:

            time.sleep(2)

            spd = 1
            angle = self.dir

            self.drive(spd, angle)

            time.sleep(1.4)

            self.mode = 3

        elif self.mode == 3:

            spd = 1
            angle = 0
            distance = 10

            self.flag_box, filtered_pic = cd_color_segmentation(self.camera_data.cv_image, self.c_low_range, self.c_high_range)
            ((height, width), (cen_x, cen_y)) = self.size_calc()

            if height > 9 and width > 6 and cen_x != 0 and cen_y != 0:
                inpt = [float(height) / 150, float(width) / 350, float(cen_x) / 376, float(cen_y) / 672]
                net_out = forward(inpt)
                distance = net_out[0] * 4
                angle = net_out[1] * 720 + 180
                angle += 0
                angle = ((angle - 540) / 4) / 70


            if distance < 1.5:
                self.mode = 4

            self.drive(spd, angle)

        elif self.mode == 4:

            distance = min(data.ranges[450: 630])

            spd = (distance - 00) * 0.6
            angle = 0

            self.flag_box, filtered_pic = cd_color_segmentation(self.camera_data.cv_image, self.c_low_range, self.c_high_range)
            ((height, width), (cen_x, cen_y)) = self.size_calc()

            if height > 9 and width > 6 and cen_x != 0 and cen_y != 0:
                inpt = [float(height) / 150, float(width) / 350, float(cen_x) / 376, float(cen_y) / 672]
                net_out = forward(inpt)
                angle = net_out[1] * 720 + 180
                angle += 20
                angle = ((angle - 540) / 4) / 70

            if spd < 0:
                spd = 0
                self.mode = 5

            self.drive(spd, angle)

        elif self.mode == 5:

            spd = 0
            angle = 0

            self.drive(spd, angle)

        print("Vivek says the mode is {}".format(self.mode))

    def detect_sign(self):

        dir = signIdentify(self.camera_data.cv_image)#, './images/onewayarrow.png')

        return dir

    def drive(self, spd, angle):
        """write driving commands here! You don't need to publish the drive command,
        that is being taken care of in the main() function"""

        self.cmd.drive.speed = spd
        self.cmd.drive.steering_angle = angle

        self.pub.publish(self.cmd)




        # rospy.loginfo("size: {} center: {} meters: {}".format(size, center, distance))





def main():
    global AUTONOMOUS_MODE
    try:
        ic = driveStop()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if AUTONOMOUS_MODE:
                ic.pub.publish(ic.cmd)

    except:
        exit()


if __name__ == "__main__":
    main()
