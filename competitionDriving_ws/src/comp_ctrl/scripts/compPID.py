#!/usr/bin/env python


from __future__ import print_function

import roslib
roslib.load_manifest('comp_ctrl')
import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

# Copied code for node from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython


class image_converter:

    def __init__(self):
        # we want to subscribe to the image that is published automatically by the camera
        # then we want to publish the velocity which is automatically heard by the robot
        # self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

        self.publish = rospy.Publisher("/R1/skid_vel", Twist, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Gets the velocity message from the determineVelocity function
        velocity = self.determineVelocity(cv_image)
        self.publish.publish(velocity)

    # Finds the edge conditions for each search line by iteration of "height"
    # pixel line of pixels up
    def edgePass(self, height, newMask, w):
        left = -34
        right = -34

        for x in range(int(w/2)):
            if (newMask[height, x] > 0):
                left = x
                break

        for x in range(int(w/2), w):
            if (newMask[height, w-x-1] > 0):
                right = w-x
                break
        if (left == 0 or left == -34):
            gotLeft = False
        else:
            gotLeft = True
        if (right == w or right == -34):
            gotRight = False
        else:
            gotRight = True

        return left, right, gotLeft, gotRight

    # determineVelocity function calculate the velocity for the robot based
    # on the position of the line in the image.
    def determineVelocity(self, cv_image):
        # get a mask for the road color

        center = -34
        lower = 85
        upper = 95
        lower_bgr = np.array([lower, lower, lower])
        upper_bgr = np.array([upper, upper, upper])

        mask = cv2.inRange(cv_image, lower_bgr, upper_bgr)
        newMask = mask
        cv2.imshow("Camera", mask)
        cv2.waitKey(1)

        h, w = newMask.shape[0:2]

        gap = int(w/10)

        # find the conditions for the edges of the given search lines
        edgeConditions = []
        searchLines = [int(0.83*h), int(0.72*h), int(0.61*h)]
        for search in range(len(searchLines)):
            left, right, gotLeft, gotRight = self.edgePass(searchLines[search], newMask, w)
            edgeConditions.append(left)
            edgeConditions.append(right)
            edgeConditions.append(gotLeft)
            edgeConditions.append(gotRight)

        leftTotal = edgeConditions[2] + edgeConditions[6] + edgeConditions[10]
        rightTotal = edgeConditions[3] + edgeConditions[7] + edgeConditions[11]

        # evaluate situation (corner, straight, intersection, etc)
        if (leftTotal == 0 and rightTotal == 0):  # either totally lost or at a T intersection
            center = 1  # value of extreme left turn
            print("T")
        elif(leftTotal == 0):  # either a sharp corner or a left intersection
            if((edgeConditions[9]-edgeConditions[1]) > int(0.23*w)):  # True -> corner
                center = 1  # value of extreme left turn
                print("Corner")
            else:  # left turn intersection
                center = edgeConditions[5] - int(0.13*w)  # approximate lane center for the intersection
                print("L Intersect")
        else:  # straights including parking space straights
            bottomCenter = int((edgeConditions[1]+edgeConditions[0])/2)
            midCenter = int((edgeConditions[5]+edgeConditions[4])/2)
            topCenter = int((edgeConditions[9]+edgeConditions[8])/2)
            aveCenter = int((bottomCenter + midCenter + topCenter)/3)
            # checks to ensure that the parking space is not messing with the center
            if (bottomCenter - aveCenter > gap/2):
                aveCenter = int((midCenter + topCenter)/2)
            elif (midCenter - aveCenter > gap/2):
                aveCenter = int((bottomCenter + topCenter)/2)
            elif (topCenter - aveCenter > gap/2):
                aveCenter = int((bottomCenter + midCenter)/2)
            center = aveCenter
            print("straight")
            print(center)

        # compute state 0 through 9
        stateNumber = center / gap
        print(stateNumber)  # debugging printing TODO remove when complete

        velocity = Twist()
        """
        # goes through different options of turning
        if stateNumber > 5:
            # turn right Cop
            velocity.linear.x = 0
            velocity.angular.z = -0.5
        elif stateNumber < 4:
            # turn left
            velocity.linear.x = 0
            velocity.angular.z = 0.5
        else:
            # go straight
            velocity.linear.x = 0.4
            velocity.angular.z = 0
            """
        return velocity


# the main function is what is run
# calls on the image_converter class and initializes a node
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()  # spin() keeps python from exiting until the node is stopped
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
