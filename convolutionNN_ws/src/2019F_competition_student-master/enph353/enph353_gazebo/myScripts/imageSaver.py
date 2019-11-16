#!/usr/bin/env python

# This script saves the images from the video feed of the robot while the
# robot is being driven by the user (user controlled)


from __future__ import print_function

import roslib
roslib.load_manifest('enph353_gazebo')
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
        print("Node initialized")
        self.bridge = CvBridge()
        self.fileNameIncrement = 0
        self.npcType = "lot_P0_AA00_"  # change based on which one is wanted

        # Need to initial subscriber for subscribing to image feed
        self.imageSubscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    def callback(self, data):
        # we don't need every frame saved, just the significant ones
        print("Callback started")

        try:
            robotImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # create filename for image
        filename = self.fileNameCreator(self.npcType)

        # save image using photographer
        self.photographer(robotImage, filename)

    """
    @brief: Saves current frame of robot camera feed to the images
            folder in the same directory as this script
    @param: robotImage: cv image from robot camera plugin
            filename: string containing filename of image without .jpg
    """
    def photographer(self, robotImage, filename):
        # saves image to a file in current directory
        relPath = "rawImages/"  # "grimmNPC_images/"
        filenameWithExtension = relPath + filename + ".jpg"
        print("Filename with extension: ", filenameWithExtension)

        cv2.imwrite(relPath + filename + ".jpg", robotImage)

        print("Image saved")

    """
    @brief: Creates a filename for image
    @param: npc: raptor or pedistrian, specified at top of script
    @return: string containing filename without .jpg
    """
    def fileNameCreator(self, npc):
        self.fileNameIncrement = self.fileNameIncrement + 1
        filename = npc + str(self.fileNameIncrement)
        print("Filename: " + filename)
        return filename


"""
@brief: Main function that calls on the robotPhotographer class
        and makes an instance of it

"""
def main(args):
    rb = robotPhotographer()
    rospy.init_node('robotPhotographer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
