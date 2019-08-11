# /usr/bin/env python

# import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
import datetime
from color_segmentation import *

# initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"

AUTONOMOUS_MODE = True


class driveStop(object):
    """class that will help the robot drive and stop at certain conditions
    """

    def __init__(self):
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=2)
        rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)
        self.joy_sub = rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

        """ initialize the box dimensions"""
        self.flag_box = ((0, 0), (0, 0))
        self.flag_center = (0, 0)
        self.flag_size = 0

        """driving stuff"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0

        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image=False)
        self.imagePush = None

        self.bridge = CvBridge()
        self.min_value = 0

        self.pictures = []
        self.i = 0

        self.light_l = 10000

    def driveStop_car_callback(self, data):
        """laser scan callback function"""
        # checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")


        start, pic, picture, a = startLights(self.camera_data.cv_image)

        if a < self.light_l:
            self.light_l = a

        if a > self.light_l + 1000:
            start = True
        else:
            start = False

        print(start, a, self.light_l)

        # print(signIdentify(self.camera_data.cv_image))

# infinite loop


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