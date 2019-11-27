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
        print("Starting Up")
        # we want to subscribe to the image that is published automatically by the camera
        # then we want to publish the velocity which is automatically heard by the robot
        # self.image_pub = rospy.Publisher("image_topic_2", Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback, buff_size=2**24)

        self.publishVel = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
        self.inLoop = False
        self.cooldownFlag = False
        self.going = True
        self.counter = 0
        self.cooldown = 0
        self.cornerNumber = 0
        self.edgeValue = 0
        self.pedCounter = 0
        self.crosswalkCooldown = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Gets the velocity message from the determineVelocity function
        velocity = self.determineVelocity(cv_image)
        # self.publishVel.publish(velocity)

    # Finds the edge conditions for each search line by iteration of "height"
    # pixel line of pixels up

    def edgePass(self, height, newMask, w):
        left = -34
        right = -34
        searchIndent = int(-1.00*height + 700)
        # print(searchIndent)

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, x] > 0):  # looks for mask to go high
                # if (int(newMask[height, x + 1] + newMask[height, x + 2] + newMask[height, x + 3]) > 2):  # ensures it isnt a mask artifact
                # print(int(newMask[height, x + 1] + newMask[height, x + 2] + newMask[height, x + 3]))
                left = x
                break

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, w-x-1] > 0):  # looks for mask to go high
                # if (int(newMask[height, w - x - 2] + newMask[height, w - x - 3] + newMask[height, w - x - 4]) > 2):  # ensures it isnt a mask artifact
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

    def leftCorner(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        rospy.sleep(1.120)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)

    # def forwardStep(self):
    #     velocity = Twist()
    #     # stop current motion
    #     velocity.linear.x = 0
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     rospy.sleep(0.015)
    #     # turn 90 degrees left
    #     velocity.linear.x = 0.4
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     rospy.sleep(1.260)
    #     velocity.linear.x = 0
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     # for debug, stop and wait
    #     rospy.sleep(0.015)

    def backwardStep(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = -0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.500)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        rospy.sleep(0.015)

    def leftJog(self):
        jogDelay = 0.015
        jogTime = 0.040  # + error*0.00001
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogDelay)
        # turn 90 degrees left
        velocity.linear.x = 0.0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        rospy.sleep(jogTime)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        # rospy.sleep(1.0)

    def rightJog(self):
        jogDelay = 0.015
        jogTime = 0.035  # + error*0.00001
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogDelay)
        # turn 90 degrees left
        velocity.linear.x = 0.0
        velocity.angular.z = -0.5
        self.publishVel.publish(velocity)
        rospy.sleep(jogTime)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        # rospy.sleep(1.0)

    # def forwardJog(self, error):
    #     jogDelay = 0.010
    #     jogTime = 0.071  # - error*0.01
    #     if jogTime < 0.01:
    #         jogTime = 0.01
    #     velocity = Twist()
    #     # stop current motion
    #     velocity.linear.x = 0.0
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     rospy.sleep(jogDelay)
    #     # turn 90 degrees left
    #     velocity.linear.x = 0.4
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     rospy.sleep(jogTime)
    #     velocity.linear.x = 0.0
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     # for debug, stop and wait
    #     # rospy.sleep(jogDelay)

    def atCrosswalk(self, cv_image):
        retval = False
        lowerRed = np.array([0, 150, 150])
        upperRed = np.array([5, 255, 255])

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        redMask = cv2.inRange(img, lowerRed, upperRed)
        h, w = redMask.shape[0:2]
        # blur3 = cv2.medianBlur(redMask, 23)
        # blurProper = blur3[int(h/2):h, :]

        imgSum = np.sum(redMask)
        if (imgSum > 350000):
            retval = True
        else:
            retval = False
        return retval

    def checkPedestrain(self, cv_image):
        retval = False
        lowerBlue = np.array([80, 60, 0])
        upperBlue = np.array([120, 200, 150])

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blueMask = cv2.inRange(img, lowerBlue, upperBlue)
        h, w = blueMask.shape[0:2]
        blur3 = cv2.medianBlur(blueMask, 7)
        # blur3 = cv2.medianBlur(redMask, 23)
        blurProper = blur3[int(h/3):h, int(0.45*w):int(0.55*w)]

        imgSum = np.sum(blurProper)
        if (imgSum > 10000):
            retval = True
        else:
            retval = False
        return retval

    def leftTurn(self, cornerNumber):
        if cornerNumber > 4:
            velocity = Twist()
            velocity.linear.x = 0
            velocity.angular.z = 0.0
            self.publishVel.publish(velocity)
            return False
        cornerTime = [1.135, 1.120, 1.125, 1.125, 1.115]
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        rospy.sleep(cornerTime[cornerNumber])
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)
        return True

    def getEdge(self, cv_image, height, w, edgeValue):
        for x in range(int(w/2), w):
            if (cv_image[height, x] > 0):
                return x
        print("edge Failure")
        return edgeValue

    # determineVelocity function calculate the velocity for the robot based
    # on the position of the line in the image.

    def determineVelocity(self, cv_image):
        # get a mask for the road color
        print(self.pedCounter)
        self.crosswalkCooldown = self.crosswalkCooldown - 1
        if self.crosswalkCooldown < 0:
            self.crosswalkCooldown = 0

        # center = -34
        # offset = 0
        lower_hsv = np.array([61, 50, 50])
        upper_hsv = np.array([81, 255, 255])

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]

        jogThreshold = int(w/40)
        if(self.inLoop is False):
            self.forwardStep()
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.leftTurn(self.cornerNumber)
            self.inLoop = True
            self.cornerNumber = self.cornerNumber + 1
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.edgeValue = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
            self.cooldownFlag = True
            self.cooldown = 20
        elif(self.going is False):
            velocity = Twist()
            velocity.linear.x = 0
            velocity.angular.z = 0.0
            self.publishVel.publish(velocity)
        elif self.atCrosswalk(cv_image) is True or self.pedCounter > 0:
            if self.crosswalkCooldown > 0:
                velocity = Twist()
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
            elif self.pedCounter == 0:
                velocity = Twist()
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
                self.pedCounter = 1
            elif self.pedCounter == 1:
                if(self.checkPedestrain(cv_image) is True):
                    self.pedCounter = 2
            elif self.pedCounter == 2:
                if(self.checkPedestrain(cv_image) is False):
                    self.pedCounter = 0
                    velocity = Twist()
                    velocity.linear.x = 0.4
                    velocity.angular.z = 0.0
                    self.publishVel.publish(velocity)
                    self.crosswalkCooldown = 30
            else:
                print("Ped Failure")

        elif self.cooldownFlag is True:
            self.cooldown = self.cooldown - 1
            print(self.cooldown)
            if self.cooldown < 0:
                self.cooldown = 0
                self.cooldownFlag = False
                self.edgeValue = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
        else:
            #print(self.edgeValue)
            newEdge = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
            #print(newEdge)
            if self.edgeValue - newEdge < -jogThreshold:
                self.rightJog()
                self.edgeValue = newEdge
            if self.edgeValue - newEdge > jogThreshold:
                self.leftJog()
                self.edgeValue = newEdge
            contours = np.array([[int(0.42*w), int(0.75*h)], [int(0.48*w), int(0.25*h)], [int(0.49*w), int(0.25*h)], [int(0.55*w), int(0.75*h)]])
            img = np.zeros((h, w), dtype="uint8")  # create a single channel h x w pixel black image
            cv2.fillPoly(img, pts=[contours], color=(255, 255, 255))

            finalMask = np.zeros((h, w), dtype="uint8")
            cv2.bitwise_and(newMask, img, finalMask)
            velocity = Twist()
            # leftEdge = int(3*w/7)
            # rightEdge = int(15*w/28)
            # topEdge = int(0.4*h)
            # croppedBlur = newMask[topEdge:h, leftEdge:rightEdge]
            imgSum = int(np.sum(finalMask))
            sumText = str(imgSum)
            #print(sumText)
            thresholdSet = [4000, 4000, 4250, 4250, 4300, 4250]
            threshold = thresholdSet[self.cornerNumber]
            if imgSum > threshold:
                self.counter = self.counter + 1
                if self.counter > 0:
                    velocity = Twist()
                    self.publishVel.publish(velocity)
                    self.going = self.leftTurn(self.cornerNumber)
                    velocity = Twist()
                    self.publishVel.publish(velocity)
                    self.cornerNumber = self.cornerNumber + 1
                    self.cooldownFlag = True
                    self.cooldown = 20
            else:
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
                self.counter = 0


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
