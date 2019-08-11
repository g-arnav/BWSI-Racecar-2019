import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
#from wall_follower import simple_PD, regression, p2l_dist
#from color_segmentation import startLights, cd_color_segmenation
#from testLight import driveStop_car_callback
from navigation import Navigation
from color_segmentation import *
import time
from newZed import Zed_converter

AUTONOMOUS_MODE = True
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
AR_TOPIC = "/ar_pose_marker"
POSE_TOPIC = "/pf/viz/inferred_pose"
JOY_TOPIC = "/vesc/joy"
COORDINATE_CONSTANT = 3.2

class finalRaceDrive(object):
    def __init__(self):
        rospy.init_node("finalRace")
        self.lidarData = None
        self.cmd = AckermannDriveStamped()
        self.AR = 0

        self.pleaseGo = False

        self.direction = None

        self.navigation = Navigation()
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.joy_sub = rospy.Subscriber(JOY_TOPIC, Joy, self.joy_callback)
        self.signList = []

        self.camera_data = Zed_converter(False, save_image=False)

        self.lastTag = 0
        #self.landmarks = {"gStart"    : ("""unknown"""),
        #                  "sHighway"  : ("""unknown"""),
        #                  "hOff"      : ("""unknown"""),
        #                  "oWind"     : ("""unknown"""),
        #                  "wBeaver"   : ("""unknown"""),
        #                  "bWash"     : ("""unknown"""), ###
        #                  "wOut"      : ("""unknown"""), ###
        #                  "wHighway"  : ("""unknown"""),
        #                  "hEnd"      : ("""unknown"""),
        #                  "eBrick"    : ("""unknown"""), ###
        #                  "cUnder"    : ("""unknown"""),
        #                  "eRamp"     : ("""unknown"""),
        #                  "rDonut"    : ("""unknown"""),
        #                  "dDonut"    : ("""unknown"""),
        #                  "dOff"      : ("""unknown"""),
        #                  "oConfuse"  : ("""unknown"""),
        #                  "bConfuse"  : ("""unknown"""), ###
        #                  "uFunnel"   : ("""unknown"""),
        #                  "fGrass"    : ("""unknown"""),
        #                  "gGrass"    : ("""unknown""")}

        #self.stateFunctions = ("simple_PD()", #gStart - sHighway
        #                       "fillerFunction()", #sHighway - hOff
        #                       "fillerFunction()", #hOff - oWind
        #                       "fillerFunction()", #oWind - wBeaver
        #                       "simple_PD()", #wBeaver - bWash                          SHORTCUT
        #                       "fillerFunction()", #wBeaver - wHighway
        #                       "fillerFunction()", #bWash - wOut                        SHORTCUT
        #                       "simple_PD()" #wOut - wHighway                           SHORTCUT
        #                       "fillerFunction()", #wHighway - #hEnd
        #                       "fillerFunction()", #hEnd - eRamp
        #                       "simple_PD()", #eRamp - rDonut
        #                       "fillerFunction()", #rDonut - dDonut
        #                       "fillerFunction()", #dDonut - dOff
        #                       "simple_PD()", #dOff - oConfuse
        #                       "fillerFunction()", #oConfuse - cUnder
        #                       "fillerFunction()", #hEnd - eBrick                      SHORTCUT
        #                       "fillerFunction()", #eBrick - bConfuse                 SHORTCUT
        #                       "fillerFunction()", #bConfuse - cUnder                   SHORTCUT
        #                       "simple_PD()", #cUnder - uFunnel
        #                       "fillerFunction()", #uFunnel - fGrass
        #                       "fillerFunction()", #fGrass - gGrass
        #                       "fillerFunction()") #gGrass - gStart)


        self.tagsAndStates = ("self.navigation.potential_fields(2)", #0
                              "self.highway()", #madefaster
                              "self.navigation.potential_fields(1.5)",
                              "self.navigation.wall_follower(1, 0.6, side = 1)",
                              "self.beforeGraves()",
                              "self.graves()", #5 #should be proportional? switch back
                              "self.navigation.potential_fields(2)",
                              "self.highway()",
                              "self.navigation.wall_follower(1.5, 1, side = -1)",
                              "self.navigation.potential_fields(1)",
                              "self.detectSign()", #10
                              "self.navigation.wall_follower(1.5, 1, side = -1)",
                              "self.navigation.wall_follower(1.5, 1, side = -1)",
                              "self.navigation.potential_fields(1.7)",
                              "self.final()",
                              "self.final()", #15
                              "self.final()",
                              "self.final()")

        self.auto = False
        self.sign = 0
        self.time = None

        self.signState = 0
        self.line = None

    def driveCallback(self, data):
        self.scan(data)

        if not self.pleaseGo:
            print("not ready!")
            if self.auto:
                self.pleaseGo = self.finalLights()
        else:
            exec(self.tagsAndStates[self.AR])
            self.drive()

    def joy_callback(self, data):
        if data.buttons[5] == 1:
            self.auto = True
        else:
            self.auto = False

    def finalLights(self):
        toReturn, __, __, __ = startLights(self.camera_data.cv_image)

        print(toReturn)
        return toReturn

    def drive(self):
        self.cmd.drive.speed = self.navigation.spd
        self.cmd.drive.steering_angle = self.navigation.angle
        print("state: {} speed: {} angle: {}".format(self.AR, self.cmd.drive.speed, self.cmd.drive.steering_angle))
        self.drive_pub.publish(self.cmd)

    def detectSign(self):
        # if self.signState == 0:
        #     self.navigation.potential_fields(0.5)
        #     if self.data[540 < 0.4]:
        #         self.signState = 1
        # if self.signState == 1:
        #     self.navigation.spd = -0.5
        #     self.navigation.angle = 0
        #     self.drive()
        #     time.sleep(.8)
        #     self.signState = 2
        # if self.signState == 2:
        #     self.navigation.spd = 0
        #     self.navigation.angle = 0
        #     self.drive()
        #     time.sleep(0.2)
        #     dir = signIdentify(self.camera_data.cv_image)
        #     if dir == left:
        #         self.dir = 1
        #     else:
        #         self.dir = -1
        #     self.time = time.time()
        #     self.signState = 3
        # if self.signState == 3:
        #     self.navigation.wall_follower(1.5, 1, self.dir)
        #     if time.time() - self.time < 1:
        #         self.signState=4
        # if self.signState == 4:
        #     self.navigation.spd = 0
        #     self.navigation.angle = 0
        #     self.drive()
        #     time.sleep(0.2)
        #     dir = signIdentify(self.camera_data.cv_image)
        #     if dir == left:
        #         self.dir = 1
        #     else:
        #         self.dir = -1
        #     self.time = time.time()
        #     self.signState = 5
        # if self.signState == 5:
        #     self.navigation.wall_follower(1.5, 1, self.dir)
        # if self.sign == 0:
        #     if self.time == None:
        #         self.time = time.time()
        #     if time.time() - self.time < 0.8:
        #         self.navigation.potential_fields(1)
        #     else:
        #         self.sign = 1
        # elif len(self.signList) < 20:
        #     try:
        #         lastSign = signIdentify(self.camera_data.cv_image)
        #         if lastSign == "right":
        #             self.signList.append(-1)
        #         else:
        #             self.signList.append(1)
        #     except:
        #         self.signList = []
        #     self.navigation.potential_fields(0.5)
        # elif self.direction == None:
        #     signMean = np.mean(self.signList)
        #     if signMean > 0:
        #         self.direction = "left"
        #     else:
        #         self.direction = "right"
        #     self.timeStart = time.time()
        #
        # else:
        #     self.navigation.wall_follower(1.5, 1, side=self.direction)
        #     if time.time() - self.timeStart > 1.2:
        #         self.direction = None
        #         self.signList = []
        self.navigation.wall_follower(1.5, 1, side=-1)

    def scan(self,data):
        self.data = list(data.ranges)
        for i, point in enumerate(self.data):
            if point > 60:
                self.data[i] = self.data[i-1]
        self.navigation.data = data
        self.navigation.lidar = self.data

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        if tags.markers:
            if tags.markers[-1].id > self.AR and tags.markers[-1].id < 19:
                self.AR = tags.markers[-1].id
                print(self.AR)


    def highway(self):
        forward_dist = max(self.data[400:680])
        self.navigation.wall_follower(4, 1.5, side=-1)

    def beforeGraves(self):
        _, filter, a, __ = cd_color_segmentation(self.camera_data.cv_image[200:], np.array([0, 44, 147]), np.array([8, 254, 255]))
        if a < 5000:
            self.navigation.potential_fields(1.5)
        else:
            self.AR = 5

    def graves(self):
        _, filter, a, __ = cd_color_segmentation(self.camera_data.cv_image[200:], np.array([0, 44, 147]), np.array([8, 254, 255]))
        if a < 5000 and self.line == None :
            self.navigation.potential_fields(1.5)
        else:
            self.navigation.line_follower(0.75, self.camera_data.cv_image)
            self.line = True

    def final(self):
        forward_dist = max(self.data[400:680])
        if forward_dist < 2:
            self.navigation.potential_fields(2)
        else:
            self.navigation.wall_follower(2.5, 1, side=-1)


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
