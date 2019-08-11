import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from wall_follower import WallFollower
from newZed import Zed_converter

from navigation import Navigation


AUTONOMOUS_MODE = True
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"
POSE_TOPIC = "/pf/viz/inferred_pose"
COORDINATE_CONSTANT = 3.2

class finalRaceDrive(object):
    DESIRED_DISTANCE = 1


    def __init__(self):
        rospy.init_node("finalRace")
        self.lidarData = None
        self.cmd = AckermannDriveStamped()

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)

        self.navigation = Navigation()
        #self.sound_pub = rospy.Publisher("state", String, queue_size=1)

        #self.camera_data = Zed_converter(False, save_image=False)

        #self.currentPosition = (x,y)

        self.pleaseGo = True

        self.camera_data = Zed_converter(False, save_image=False)


    def scan(self,data):
        self.data = data
        self.navigation.lidar = data.ranges


    def drive(self):
        spd = self.navigation.spd
        angle = self.navigation.angle
        self.cmd.drive.speed = spd
        self.cmd.drive.steering_angle = angle
        print("spd: {} angle: {}".format(self.cmd.drive.speed, self.cmd.drive.steering_angle))
        self.drive_pub.publish(self.cmd)


    def driveCallback(self, data):
        self.scan(data)
        # self.navigation.potential_fields()
        # self.navigation.find_goals()
        self.navigation.wall_follower(1.5, 1, -1)
        self.drive()



def main():
    try:
        ic = finalRaceDrive()
        rospy.Rate(100)
        while not rospy.is_shutdown():
            pass
            # ic.drive_pub.publish(ic.cmd)
    except rospy.ROSInterruptException:
        exit()


if __name__ == "__main__":
    main()
