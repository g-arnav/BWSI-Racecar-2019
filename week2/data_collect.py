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

        self.factor = 0
        self.cone_width = 0

        self.input_data = []
        self.label_data = []

        self.map = None
        self.l_data = None

        self.lower_ind = 180
        self.upper_ind = 900

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

        self.flag_box, filtered_pic = cd_color_segmentation(self.camera_data.cv_image)

        cv2.rectangle(filtered_pic, self.flag_box[0], self.flag_box[1], (0, 255, 0))
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(filtered_pic, "bgr8"))

        self.l_data = data

        my_ranges = self.l_data.ranges[self.lower_ind:self.upper_ind]


        if self.map == None:

            self.map = my_ranges

            print "Map created"
            print "starting in 10"

            for i in range(10):
                time.sleep(1)
                print("starting in {}".format(9 - i))

            print "Starting"

            return


        ind = self.check_map(my_ranges)

        # print "                              ", self.map[540 - 180], "      ", my_ranges[540 - 180]

        if len(ind) == 0:
            print("No object on lidar")
            return

        distance = my_ranges[min(ind)]
        angle = min(ind) + self.lower_ind

        # applies the current filter to the image and returns a bounding box


        # finds the size of the box
        ((height, width), (cen_x, cen_y)) = self.size_calc()

        if height * width < 60:
            print("No object on camera")
            return

        self.input_data.append([height, width, cen_x, cen_y])
        self.label_data.append([distance, angle])


        assert len(self.input_data) == len(self.label_data), "Strange data"

        if len(self.input_data) == 500:
            self.save_data()

        print("Distance: {} Angle: {}".format(distance, angle))

        if AUTONOMOUS_MODE:
            self.drive()
        else:
            pass

    def drive(self):
        """write driving commands here! You don't need to publish the drive command,
        that is being taken care of in the main() function"""
        size, center = self.size_calc()

        distance = self.factor * self.cone_width / size[1]


        # rospy.loginfo("size: {} center: {} meters: {}".format(size, center, distance))

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

            np.save("data1/{}".format(datetime.datetime.now()), data)

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
