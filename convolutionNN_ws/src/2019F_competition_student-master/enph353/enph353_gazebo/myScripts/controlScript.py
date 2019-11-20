#!/usr/bin/env python

# This is the priliminary control script that will talk
# with the competition rostopics

from __future__ import print_function

import roslib
roslib.load_manifest('enph353_gazebo')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

from keras import layers
from keras import models
from keras import optimizers

from keras.utils import plot_model
from keras import backend


class controlNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.publishVel = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
        self.publishLP = rospy.Publisher("/license_plate", String, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # checks if robot is at parking lot
        parkingLotFlag = self.atParkingLot(cv_image)

        if parkingLotFlag is True:
            # LP_image = ""  # <-- need to change
            LP_msg = self.determineLicensePlate(cv_image)
            self.publishLP.publish(LP_msg)

            velocity = self.determineVelocity(cv_image)  # function that determines velocity, need to change this later
            self.publishVel.publish(velocity)

            parkingLotFlag = False

        else:
            velocity = self.determineVelocity(cv_image)  # function that determines velocity, need to change this later
            self.publishVel.publish(velocity)

    # FOR RANYA OR ZACH TO COMPLETE
    def atParkingLot(self, cameraImg):
        # Determine if robot has reached a parking lot
        flag = True
        return flag

    # FOR RANYA TO COMPLETE
    def determineLicensePlate(self, LP_image):
        # call on image cropper -> should return five images
        # or save them locally for this script to access
        # runs CNN model for each letter/number image inputted.
        LP_msg = ""

        for i in range(5):
            

        return LP_msg

    # FOR ZACH TO COMPLETE
    def determineVelocity(self, cameraImg):
        velocity = Twist()
        return velocity


def main(args):
    controlNode = controlNode()
    rospy.init_node('controlNode', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == 'main':
    main(sys.argv)
