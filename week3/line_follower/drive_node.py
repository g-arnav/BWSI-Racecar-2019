# /usr/bin/env python

# import libraries and color segmentation code
import rospy
import cv2
import time
import numpy as np
from newZed import Zed_converter
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, Joy, Image
import datetime
import math

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

        """ initialize the box dimensions"""
        self.red_bounds = [np.array([0, 44, 147]), np.array([20, 254, 255])]
        self.yel_bounds = [np.array([25, 100, 200]), np.array([50, 255, 255])]
        self.blu_bounds = [np.array([100, 50, 100]), np.array([150, 255, 255])]

        """driving stuff"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 0
        self.cmd.drive.steering_angle = 0

        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image=False)
        self.imagePush = None

        self.bridge = CvBridge()
        self.min_value = 0

        self.cur_line = 'red'
        self.n_lines = 3


    def driveStop_car_callback(self, data):
        """laser scan callback function"""
        # checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")

        red_points, self.imagePush = cd_color_segmentation(self.camera_data.cv_image, self.red_bounds[0], self.red_bounds[1])
        # self.yel_vector, red_img = cd_color_segmentation(self.camera_data.cv_image, self.yel_bounds[0], self.yel_bounds[1])
        # self.blu_vector, red_img = cd_color_segmentation(self.camera_data.cv_image, self.blu_bounds[0], self.blu_bounds[1])

        angle = self.calc(red_points)
        speed = 0.5


        # applies the current filter to the image and stores the image in imagePush
        # self.vector, self.imagePush = cd_color_segmentation(self.camera_data.cv_image)

        # finds the size of the box

        # outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)

        # if len(self.pictures) % 100 == 0:
        #     self.save()
        #     print("saved")

        if AUTONOMOUS_MODE:
            self.drive(speed, angle)
        else:
            pass

    def calc_angle(self, angle, midpoint):


        print(angle, midpoints[1], mid, final)
        # print(final)

        return final


    def drive(self, spd, angle):
        self.cmd.drive.speed = spd
        self.cmd.drive.steering_angle = angle

        self.pub.publish(self.cmd)

    def calc(self, points):
        vx, vy, x0, y0 = points
        mid_x = 400 - x0

        ang = math.atan2(vy, vx)
        print(mid_x, ang)
        return -ang*.05 + mid_x*0.5

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
        ic.save()
        print("saved")
        exit()


if __name__ == "__main__":
    main()