import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
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
COORDINATE_CONSTANT = 3.2

class finalRaceDrive(object):
    def __init__(self):
        rospy.init_node("finalRace")
        self.lidarData = None
        self.state = 0 #wall_follower
        self.cmd = AckermannDriveStamped()
        self.arStates = None
        self.pleaseGo = False
        self.direction = None

        self.navigation = Navigation()
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.driveCallback)
        self.ar_sub = rospy.Subscriber(AR_TOPIC, AlvarMarkers, self.arCallback)
        self.local_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.localize)
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


        self.tagsAndStates = ("self.navigation.potential_fields(1)", #0

                              "self.navigation.wall_follower(1,side = -1)",
                              "self.navigation.wall_follower(1,side = 1)",
                              "self.navigation.potential_fields()",
                              "self.navigation.potential_fields()", #5 #should be proportional?
                              "self.navigation.wall_follower(1,side = 1)",
                              "self.navigation.wall_follower(1,side = -1)",
                              "self.navigation.wall_follower(1,side = -1)",
                              "self.navigation.potential_fields()",
                              "self.finalSign()", #10
                              "self.navigation.potential_fields()",
                              "self.navigation.potential_fields()",
                              "self.navigation.wall_follower(1,side = -1)",
                              "self.navigation.potential_fields(speed = 1.7)",
                              "self.navigation.wall_follower(1,side = -1)", #15
                              "self.navigation.wall_follower(1,side = -1)",
                              "self.navigation.wall_follower(1,side = -1)")

    def finalLights(self):
        toReturn, __, __, __ = startLights(self.camera_data.cv_image)

        print(toReturn)
        return toReturn

    def drive(self):
        self.cmd.drive.speed = self.navigation.spd
        self.cmd.drive.steering_angle = self.navigation.angle
        print("state: {} speed: {} angle: {}".format(self.lastTag, self.cmd.drive.speed, self.cmd.drive.steering_angle))
        self.drive_pub.publish(self.cmd)

    def finalSign(self):
        print("it's sign time!")
        if len(self.signList < 20):
            try:
                lastSign = signIdentify(self.camera_data.cv_image)
                if lastSign == "right":
                    self.signList.append(-1)
                else:
                    self.signList.append(1)
            except:
                self.signList = []
                self.navigation.potential_fields(0.5)
        elif self.direction == None:
            signMean = np.mean(self.signList)
            if signMean > 0:
                self.direction = "left"
            else:
                self.direction = "right"
            self.timeStart = time.time()
            self.signList = []

        else:
            self.navigation.wall_follower(1.5,1,side = self.direction)
            if time.time() - self.timeStart > 3:
                self.direction = None


    def driveCallback(self, data):
        self.scan(data)

        if not self.pleaseGo:
            print("not ready!")
            self.pleaseGo = self.finalLights()
    
        else:
            if self.arStates is not None:
                if self.arStates[0] == self.lastTag+1:
                    self.lastTag = self.arStates[0]
                    print("tag is now ",self.lastTag)
            exec(self.tagsAndStates[self.lastTag])
            self.drive()

    def approximateCoordinates(coords,xDif,yDif):
        firstVertex = ((coords[0] + xDif), (coords[1] + yDif))
        secondVertex = ((coords[0] - xDif), (coords[1] - yDif))
        return (firstVertex, secondVertex)

    def checkCoordinatesHorizontal(coordsCurrent, coordsDestination):
        approxCoords = approximateCoordinates(coordsDestination,2,0.3)
        toReturn = False
        if coordsCurrent[0] > approxCoords[1][0] and coordsCurrent[0] < approxCoords[0][0]:
            if coordsCurrent[1] > approxCoords[1][1] and coordsCurrent[1] < approxCoords[0][1]:
                toReturn = True
        return toReturn

    def checkCoordinatesVertical(coordsCurrent, coordsDestination):
        approxCoords = approximateCoordinates(coordsDestination,0.3,2)
        toReturn = False
        if coordsCurrent[0] > approxCoords[1][0] and coordsCurrent[0] < approxCoords[0][0]:
            if coordsCurrent[1] > approxCoords[1][1] and coordsCurrent[1] < approxCoords[0][1]:
                toReturn = True
        return toReturn

    def checkCoordinatesSpot(coordsCurrent, coordsDestination):
        approxCoords = approximateCoordinates(coordsDestination,0.3,0.3)
        toReturn = False
        if coordsCurrent[0] > approxCoords[1][0] and coordsCurrent[0] < approxCoords[0][0]:
            if coordsCurrent[1] > approxCoords[1][1] and coordsCurrent[1] < approxCoords[0][1]:
                toReturn = True
        return toReturn

    def scan(self,data):
        self.data = data
        self.navigation.data = data

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        # TODO: Write your state changes here
        ret = list()
        for i in range(0, len(tags.markers)):
            ret.append(tags.markers[i].id)
        self.arStates = ret
        return self.arStates
        #what does this do?

    def localize(self, data):  # callback function to save inferred pose based on localization
        self.pose = data.pose
        self.x = self.orientation.x
        self.y = self.orientation.y
        self.z = self.orientation.z
        self.w = self.orientation.w

        self.windmill = self.landmarks.windmill

    def findARTag(self):  # state when car looks for AR tags
        self.detectedTag = arCallback()

    def windmillResponse(self):
        pass #will write later

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
