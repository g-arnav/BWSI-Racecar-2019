#!/usr/bin/env python

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

# initalize global variables
DRIVE_TOPIC = "/drive"
IMAGE_TOPIC = "/zed/zed_node/color_seg_output"
ANGLE_CONSTANT = 0.1
THRESHOLD_AREA = 5000
SMALL_THRESHOLD = 400

RED = np.array([[0, 44, 147], [8, 254, 255]])
# YELLOW = np.array([[19, 14, 150], [33, 163, 255]])
# BLUE = np.array([[56, 109, 150], [111, 255, 255]])

AUTONOMOUS_MODE = True


class driveStop(object):
    """class that will help the robot drive and stop at certain conditions
    """
    LOOKING_FOR = 'RED'

    def __init__(self):
        # help me
        """initalize the node"""
        rospy.init_node("driveStop")
        self.pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.image_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=2)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.driveStop_car_callback)

        """ initialize the box dimensions"""
        self.flag_box = ((0, 0), (0, 0))
        self.flag_center = (0, 0)
        self.flag_size = 0

        """driving stuff"""
        self.cmd = AckermannDriveStamped()
        self.cmd.drive.speed = 1
        self.cmd.drive.steering_angle = 0

        """get the camera data from the class Zed_converter in Zed"""
        self.camera_data = Zed_converter(False, save_image=False)
        self.imagePush = None
        self.imageHSV = None

        self.bridge = CvBridge()
        self.min_value = 0

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

        # applies the current filter to the image and stores the image in imagePush
        self.flag_box, self.imagePush, self.imageHSV = cd_color_segmentation(self.camera_data.cv_image, RED)
        print('red looked for')

        # print(str((self.flag_box[1][0] - self.flag_box[0][0]) * (self.flag_box[1][1] - self.flag_box[0][1])))
        if self.flag_box is None:
            print('no red detected')

        areaOfBox = (self.flag_box[1][0] - self.flag_box[0][0]) * (self.flag_box[1][1] - self.flag_box[0][1])
        the_boolean = self.flag_box is None or areaOfBox < THRESHOLD_AREA
        if self.LOOKING_FOR == 'RED' and the_boolean:
            self.flag_box, self.imagePush, self.imageHSV = cd_color_segmentation(self.camera_data.cv_image, RED)
            print('red boi')
        ##          elif areaOfBox > SMALL_THRESHOLD:
        ##                    self.LOOKING_FOR = 'BLUE'
        ##                    print('big boi, following blue')
        ##            else:
        ##                    self.cmd.speed = 0
        ##                    print('small boi, stopping')
        ##                    self.LOOKING_FOR = 'BLUE'

        ##                      if self.flag_box is None or (self.flag_box[1][0] - self.flag_box[0][0]) * (self.flag_box[1][1] - self.flag_box[0][1]) < THRESHOLD_AREA:
        ##                            self.flag_box, self.imagePush, self.imageHSV = cd_color_segmentation(self.camera_data.cv_image, BLUE)
        ##                            print('red looked for')

        # outputs the image to the IMAGE_TOPIC
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(self.imageHSV, cv2.COLOR_HSV2BGR), "bgr8"))
        except CvBridgeError as e:
            print("Error bridging Zed image", e)

        if AUTONOMOUS_MODE:
            self.drive()
        else:
            pass

    def drive(self):
        CENTER = self.imagePush.shape[1] / 2
        ##            print('CENTER = ' + str(CENTER))
        # lineX = []
        ##            print(self.flag_box[0][0])
        ##            print(self.flag_box[1][0])
        # print('HSV vals: ' + str(self.imageHSV[self.flag_box[0][0]][self.flag_box[0][1]]))
        # for xPos in range(self.flag_box[0][0], self.flag_box[1][0]):
        # if xPos >= self.imagePush.shape[0]:
        # continue
        # print('HSV vals: ' + str(self.imageHSV[xPos][self.flag_box[0][1]]))
        # if 0 < self.imageHSV[xPos][self.flag_box[1][1]][0] < 8 and 44 < self.imageHSV[xPos][self.flag_box[1][1]][1] < 254 and 147 < self.imageHSV[xPos][self.flag_box[1][1]][2] < 255:
        # if len(lineX) == 0:
        # lineX = [0]*2
        # lineX[0] = xPos
        # else:
        # lineX[1] = xPos

        # print(lineX)
        xAvg = (self.flag_box[0][0] + self.flag_box[1][0]) / 2
        # print('avg: ' + str(xAvg))
        error = xAvg - CENTER
        turn_angle = -1 * error * ANGLE_CONSTANT * np.pi / 180
        print('turn angle: ' + str(turn_angle))

        if turn_angle < -1:
            self.cmd.drive.steering_angle = -1
        # print('turning right')
        elif turn_angle > 1:
            self.cmd.drive.steering_angle = 1
        # print('turning left')
        else:
            self.cmd.drive.steering_angle = turn_angle


def main():
    global AUTONOMOUS_MODE
    try:
        ic = driveStop()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if AUTONOMOUS_MODE:
                ic.pub.publish(ic.cmd)

    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()
