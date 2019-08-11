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

        self.joy_sub = rospy.Subscriber("vesc/joy", Joy, self.joy_callback)

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

        self.factor = 0
        self.cone_width = 0

        self.input_data = []
        self.label_data = []

        self.map = None
        self.l_data = None

        self.lower_ind = 180
        self.upper_ind = 900

        self.joy_data = None

    def joy_callback(self, data):
        self.joy_data = data

    def driveStop_car_callback(self, data):
        """laser scan callback function"""
        # checks if the image is valid first

        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")

        steering = self.joy_data.axes[3]
        speed = self.joy_data.axes[1]


        self.l_data = data

        # print "                              ", self.map[540 - 180], "      ", my_ranges[540 - 180]

        # applies the current filter to the image and returns a bounding box


        # finds the size of the box
        self.input_data.append([data])
        self.label_data.append([speed, steering])



        if len(self.input_data) == 500:
            self.save_data()



    def check_map(self, array, threshold=0.05):

        indexes = []

        for i in range(len(array)):
            if self.map[i] - array[i] > threshold:
                indexes.append(i)

        return indexes

    def save_data(self):

        data = np.array([self.input_data, self.label_data])

        if len(self.input_data) > 0:

            print "                                             Saved"

            np.save("data/{}".format(datetime.datetime.now()), data)

            self.input_data = []
            self.label_data = []

        else:
            print "Empty data list"





def main():
    global AUTONOMOUS_MODE
    try:
        ic = driveStop()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if AUTONOMOUS_MODE:
                ic.pub.publish(ic.cmd)

    except:
        ic.save_data()
        exit()


if __name__ == "__main__":
    main()
