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
from time import sleep

# Copied code for node from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython


class image_converter:

    def __init__(self):
        # we want to subscribe to the image that is published automatically by the camera
        # then we want to publish the velocity which is automatically heard by the robot
        # self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

        self.publish = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)

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
        searchIndent = int(-1.00*height + 700)
        # print(searchIndent)

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, x] > 0):  # looks for mask to go high
                #if (int(newMask[height, x + 1] + newMask[height, x + 2] + newMask[height, x + 3]) > 2):  # ensures it isnt a mask artifact
                    # print(int(newMask[height, x + 1] + newMask[height, x + 2] + newMask[height, x + 3]))
                left = x
                break

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, w-x-1] > 0):  # looks for mask to go high
                #if (int(newMask[height, w - x - 2] + newMask[height, w - x - 3] + newMask[height, w - x - 4]) > 2):  # ensures it isnt a mask artifact
                right = w-x
                break
        if (left <= searchIndent + 5):
            gotLeft = False
        else:
            gotLeft = True
        if (right >= w - searchIndent - 5 or right == -34):
            gotRight = False
        else:
            gotRight = True

        return left, right, gotLeft, gotRight

    def imagePresent(self, name, text, center, edgeConditions, cv_image, h, w):
        top = int(0.65*h)
        mid = int(0.70*h)
        bottom = int(0.75*h)
        topSearchIndent = int(-1.00*int(0.65*h) + 700)
        midSearchIndent = int(-1.00*int(0.70*h) + 700)
        bottomSearchIndent = int(-1.00*int(0.75*h) + 700)
        cv2.circle(cv_image, (center, bottom), 10, (0, 255, 0), -1)
        cv2.line(cv_image, (bottomSearchIndent, bottom), (w - bottomSearchIndent, bottom), (255, 0, 0), 3)
        cv2.line(cv_image, (midSearchIndent, mid), (w - midSearchIndent, mid), (255, 0, 0), 3)
        cv2.line(cv_image, (topSearchIndent, top), (w - topSearchIndent, top), (255, 0, 0), 3)
        cv2.line(cv_image, (int(w/2), 0), (int(w/2), h), (255, 0, 255), 3)
        cv2.putText(cv_image, text, (5, 100), cv2.FONT_HERSHEY_SIMPLEX,  1.0, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.putText(cv_image, str(edgeConditions), (5, 50), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (0, 0, 255), lineType=cv2.LINE_AA)
        cv2.imshow(name, cv_image)
        cv2.waitKey(1)

    def getCenter(self, edgeConditions, gap):
        usableEdges = []
        centers = []
        for line1 in range((len(edgeConditions)/4)):
            if edgeConditions[2 + 4*line1] is True:  # and edgeConditions[3 + 4*line1] is True:
                usableEdges.append(edgeConditions[0 + 4*line1])
                usableEdges.append(edgeConditions[1 + 4*line1])
        for line2 in range(len(usableEdges)/2):
            centers.append(int((usableEdges[0 + 2*line2] + usableEdges[1 + 2*line2])/2))
            averageCenter = int(sum(centers)/len(centers))
        length = len(centers)
        line3 = 0
        while (line3 < length):
            if (centers[line3] - averageCenter) > int(gap):
                centers.remove(centers[line3])
                length = length - 1
                line3 = line3 - 1
            line3 = line3 + 1

        center = int(sum(centers)/len(centers))
        return center

    def leftTurn(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publish.publish(velocity)
        sleep(0.563)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        # for debug, stop and wait
        sleep(2)

    def leftJog(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publish.publish(velocity)
        sleep(0.015)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        # for debug, stop and wait
        sleep(2)

    def rightJog(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = -0.5
        self.publish.publish(velocity)
        sleep(0.015)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        # for debug, stop and wait
        sleep(2)





    # determineVelocity function calculate the velocity for the robot based
    # on the position of the line in the image.

    def determineVelocity(self, cv_image):
        # get a mask for the road color

        center = -34
        offset = 0
        lower_hsv = np.array([0, 0, 82])
        upper_hsv = np.array([100, 255, 84])

        image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]

        gap = int(w/10)

        # find the conditions for the edges of the given search lines
        edgeConditions = []
        searchLines = [int(0.75*h), int(0.70*h), int(0.65*h)]
        # print(searchLines)
        for search in range(len(searchLines)):
            left, right, gotLeft, gotRight = self.edgePass(searchLines[search], newMask, w)
            edgeConditions.append(left)
            edgeConditions.append(right)
            edgeConditions.append(gotLeft)
            edgeConditions.append(gotRight)

        # print(edgeConditions)
        leftTotal = edgeConditions[2] + edgeConditions[6]  # + edgeConditions[10]
        rightTotal = edgeConditions[3] + edgeConditions[7]  # + edgeConditions[11]

        text = "Fail"
        # evaluate situation (corner, straight, intersection, etc)
        if(edgeConditions[3] + edgeConditions[7] + edgeConditions[11] < 2 and
           edgeConditions[2] + edgeConditions[6] + edgeConditions[10] > 1):
            center = int(0.75*w)
            text = "lost right"
        elif (edgeConditions[10] == 0 and edgeConditions[11] == 0):  # either totally lost or at a T intersection
            center = 1  # value of extreme left turn
            text = "T"
        elif(leftTotal < 1):
            # left turn intersection
            center = edgeConditions[5] - int(0.23*w)  # approximate lane center for the intersection
            if (center < 0):
                center = 0
            text = "intersection"
        else:  # straights including parking space straights
            center = self.getCenter(edgeConditions, gap)
            center = center + offset
            text = "straight"

        # compute state 0 through 9
        # self.imagePresent("camera", text, center, edgeConditions, cv_image, h, w)
        # self.imagePresent("mask", text, center, edgeConditions, interMask, h, w)
        #cv2.imshow("Camera", cv_image)
        #cv2.waitKey(1)
        # print(text)
        # stateNumber = center / gap
        velocity = Twist()
        self.leftTurn()
        # goes through different options of turning
        """
        if stateNumber > 4:
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
