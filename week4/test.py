from tf.transformations import euler_from_quaternion
import rospy
import time
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import math

AUTONOMOUS_MODE = True
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"
POSE_TOPIC = '/pf/viz/inferred_pose'
COORDINATE_CONSTANT = 0.3

#PoseStamped format:
#pose:
#  position:
#    X
#    y
#    z
#  orientation:
#    X
#    y
#    z
#    w
#

class rideshareDrive(object):
    def __init__(self):
        rospy.init_node("rideshare")
        #initialize publishers and subscribers
        self.local_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.localize)

    def localize(self, data): # callback function to save inferred pose based on localization
        self.pose = data.pose
        x = self.pose.orientation.x
        y = self.pose.orientation.y
        z = self.pose.orientation.z
        w = self.pose.orientation.w

        self.euler = euler_from_quaternion((x, y, z, w))

        # print self.pose.orientation

        print self.euler[0], self.euler[1], self.euler[2]







def main():
    try:
        ic = rideshareDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()
