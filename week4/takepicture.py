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

    def size_calc(self):
        """ calculate the x and y size of the box in pixels"""
        pix_width = self.flag_box[1][0] - self.flag_box[0][0]
        pix_height = self.flag_box[1][1] - self.flag_box[0][1]

        self.box_size = pix_width * pix_height

    def driveStop_car_callback(self, data):
        """laser scan callback function"""
        # checks if the image is valid first
        while self.camera_data.cv_image is None:
            time.sleep(0.5)
            print("sleeping")

        self.i += 1

        np.save("pictures/{}".format(self.i), self.camera_data.cv_image)
        print("took picture")
        time.sleep(0.5)

        # applies the current filter to the image and stores the image in imagePush

        # finds the size of the box
        self.size_calc()

        # outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.imagePush, "bgr8"))
        except CvBridgeError as e:
            print("Exarror bridging Zed image", e)

        # if len(self.pictures) % 100 == 0:
        #     self.save()
        #     print("saved")

        if AUTONOMOUS_MODE:
            self.drive()
        else:
            pass

    def drive(self):
        pass

    def save(self):
        np.save("pictures/{}".format(datetime.datetime.now()), self.pictures)

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