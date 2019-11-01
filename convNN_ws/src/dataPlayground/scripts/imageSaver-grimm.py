#!/usr/bin/env python

# This script saves the images from the video feed of the robot whilte the
# robot is being driven by the user (user controlled)

from __future__ import print_function

import roslib
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class robotPhotographer:

    def __init__(self):
        print("node initialized")
        self.bridge = CvBridge()
        self.fileNameIncrement = 0
        self.npcType = "intersection"  # change based on which one is wanted

        # Need to initial subscriber for subscribing to image feed
        self.imageSubscriber = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
        self.publish = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def callback(self, data):
        # sleeps for 5 seconds
        # we don't need every frame saved, just the significant ones
        print("callback started")
        rospy.sleep(5)

        try:
            robotImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("image should have changed")
        except CvBridgeError as e:
            print(e)

        # create filename for image
        filename = self.fileNameCreator(self.npcType)

        # save image using photographer
        self.photographer(robotImage, filename)
        self.publish.publish(Twist())
        rospy.sleep(10)

    """
    @brief: Saves current frame of robot camera feed to the images
            folder in the same directory as this script
    @param: robotImage: cv image from robot camera plugin
            filename: string containing filename of image without .jpg
    """
    def photographer(self, robotImage, filename):
        # saves image to a file in current directory

        # cv2.imwrite("/grimmNPC_images/" + filename + ".jpg", robotImage)
        filenameWithExtension = "grimmNPC_images/" + filename + ".jpg"
        print("filename with extension: ", filenameWithExtension)

        cv2.imwrite("grimmNPC_images/" + filename + ".jpg", robotImage)

        print("image saved")

    """
    @brief: Creates a filename for image
    @param: npc: raptor or pedistrian, specified at top of script
    @return: string containing filename without .jpg
    """
    def fileNameCreator(self, npc):
        self.fileNameIncrement = self.fileNameIncrement + 1
        filename = npc + "_" + str(self.fileNameIncrement)
        print("filename: " + filename)
        return filename


"""
@brief: Main function that calls on the robotPhotographer class
        and makes an instance of it

"""
def main(args):
    print("Photographer started")
    rb = robotPhotographer()
    rospy.init_node('robotPhotographer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
