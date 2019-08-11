 #!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
class Safety:
    DRIVE_IN_TOPIC = '/drive'
    DRIVE_OUT_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/default'
    SCAN_TOPIC = '/scan'
    #SAFE_DIST = .9

    def __init__(self):
        self.sub_scan = rospy.Subscriber(self.SCAN_TOPIC,LaserScan,callback=self.scan_callback)
        self.sub_drive = rospy.Subscriber(self.DRIVE_IN_TOPIC,AckermannDriveStamped,callback=self.drive_callback)
        self.pub_drive = rospy.Publisher(self.DRIVE_OUT_TOPIC,AckermannDriveStamped,queue_size=1)
        self.list = []
        self.spd = 0
        self.state = 0

    def state_callback(self, msg):
        self.state = msg

    def scan_callback(self, msg):
        #print("scan_callback")
        #print(msg)
        self.scan = msg

    def drive_callback(self, msg):
        #print("drive_callback")
        if self.is_safe(msg):
            self.pub_drive.publish(msg)
            self.list = []
        else:
            if self.list == []:
                self.list.append(msg.drive.speed)
            cmd = AckermannDriveStamped()
            left = (sum(self.scan.ranges[len(self.scan.ranges)/2 -100: len(self.scan.ranges)/2 -50]))/50
            right = (sum(self.scan.ranges[len(self.scan.ranges)/2 +50: len(self.scan.ranges)/2 +100]))/50

            if right / left > 1.1:
                cmd.drive.steering_angle = 0.35
                cmd.drive.speed = self.spd
                self.pub_drive.publish(cmd)
            elif left / right > 1.1:
                cmd.drive.steering_angle = - 0.35
                cmd.drive.speed = self.spd
                self.pub_drive.publish(cmd)
            elif 0.9 < right / left < 1.1:
                cmd.drive.steering_angle = 0
                cmd.drive.speed = -.8
                self.pub_drive.publish(cmd)
            if cmd.drive.speed > .5:
                self.list.append(cmd.drive.speed)

    def is_safe(self, msg):

        current_spd = msg.drive.speed

        SAFE_DIST = 0.75

        
        # if current_spd > 5:
        #     SAFE_DIST = 3
        # elif 3 < current_spd < 5:
        #     SAFE_DIST = 2
        # else:
        #     SAFE_DIST = .9

        d = np.mean(self.scan.ranges[520: 560])

        print(d, SAFE_DIST)


        if d < SAFE_DIST:
            self.spd = d / (SAFE_DIST) * current_spd
            print self.spd
            print('SAFETY WORKS!')
            #cmd.drive.speed = .9(msg.drive.speed)
            return False
            # return True
        else:    
            return True



print('started')
rospy.init_node('safety')
safety = Safety()
rospy.spin()

