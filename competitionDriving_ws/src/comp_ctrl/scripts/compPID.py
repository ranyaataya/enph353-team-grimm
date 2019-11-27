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
            if edgeConditions[2 + 4*line1] is True:  # Checks which search lines gave back usable data, adds them to list
                usableEdges.append(edgeConditions[0 + 4*line1])
                usableEdges.append(edgeConditions[1 + 4*line1])
        for line2 in range(len(usableEdges)/2):  # takes the usuable edges and finds the center
            centers.append(int((usableEdges[0 + 2*line2] + usableEdges[1 + 2*line2])/2))
            averageCenter = int(sum(centers)/len(centers))
        length = len(centers)
        line3 = 0
        while (line3 < length):  # removes any centers that are outliers
            if (centers[line3] - averageCenter) > int(gap):
                centers.remove(centers[line3])
                length = length - 1
                line3 = line3 - 1
            line3 = line3 + 1

        center = int(sum(centers)/len(centers))
        return center

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

    def forwardStep(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.015)
        # turn 90 degrees left
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(1.260)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        rospy.sleep(0.015)

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
        jogDelay = 0.010
        jogTime = 0.035  # + error*0.00001
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
        jogDelay = 0.010
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

    def forwardJog(self, error):
        jogDelay = 0.010
        jogTime = 0.071  # - error*0.01
        if jogTime < 0.01:
            jogTime = 0.01
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogDelay)
        # turn 90 degrees left
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogTime)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        # rospy.sleep(jogDelay)

    def atSpot(self, cv_image):
        retval = False
        lowerBlue = np.array([0, 110, 145])
        upperBlue = np.array([60, 220, 245])

        img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        blueMask = cv2.inRange(img, lowerBlue, upperBlue)
        h, w = blueMask.shape[0:2]
        blur3 = cv2.medianBlur(blueMask, 23)
        blurProper = blur3[int(h/2):h, :]

        cv2.imshow("Blur 3", blurProper)
        imgSum = np.sum(blurProper)
        if (imgSum > 3000000):
            retval = True
        else:
            retval = False
        print(retval)
        return retval

    def leftTurn(self, cornerNumber):
        if cornerNumber > 4:
            velocity = Twist()
            velocity.linear.x = 0
            velocity.angular.z = 0.0
            self.publishVel.publish(velocity)
            return False
        cornerTime = [1.125, 1.127, 1.127, 1.120, 1.115]
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

    def getEdge(self, cv_image, height, w):
        for x in range(int(w/2), w):
            if (cv_image[height, x] > 0):
                return x
        print("edge Failure")
        return -34

    # determineVelocity function calculate the velocity for the robot based
    # on the position of the line in the image.

    def determineVelocity(self, cv_image):
        # get a mask for the road color

        center = -34
        offset = 0
        lower_hsv = np.array([61, 50, 50])
        upper_hsv = np.array([81, 255, 255])

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]

        # gap = int(w/20)

        # find the conditions for the edges of the given search lines
        # edgeConditions = []
        # searchLines = [int(0.83*h), int(0.73*h), int(0.62*h)]
        # # print(searchLines)
        # for search in range(len(searchLines)):
        #     left, right, gotLeft, gotRight = self.edgePass(searchLines[search], newMask, w)
        #     edgeConditions.append(left)
        #     edgeConditions.append(right)
        #     edgeConditions.append(gotLeft)
        #     edgeConditions.append(gotRight)
        # center = self.getCenter(edgeConditions, gap)
        

        # print(edgeConditions)
        # leftTotal = edgeConditions[2] + edgeConditions[6]  # + edgeConditions[10]
        # rightTotal = edgeConditions[3] + edgeConditions[7]  # + edgeConditions[11]

        # text = "Fail"
        # evaluate situation (corner, straight, intersection, etc)
        # if(edgeConditions[3] + edgeConditions[7] + edgeConditions[11] < 2 and
           # edgeConditions[2] + edgeConditions[6] + edgeConditions[10] > 1):
           #  center = int(0.75*w)
           #  text = "lost right"
        jogThreshold = int(w/25)
        if(self.inLoop is False):
            self.forwardStep()
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.leftTurn(self.cornerNumber)
            self.inLoop = True
            self.cornerNumber = self.cornerNumber + 1
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.edgeValue = self.getEdge(newMask, int(0.75*h), w)
        elif(self.going is False):
            velocity = Twist()
            velocity.linear.x = 0
            velocity.angular.z = 0.0
            self.publishVel.publish(velocity)
        elif self.cooldownFlag is True:
            self.cooldown = self.cooldown - 1
            print(self.cooldown)
            if self.cooldown < 0:
                self.cooldown = 0
                self.cooldownFlag = False
                self.edgeValue = self.getEdge(newMask, int(0.75*h), w)
        elif self.edgeValue - self.getEdge(newMask, int(0.75*h), w) < -jogThreshold:
            self.rightJog()
            self.edgeValue = self.getEdge(newMask, int(0.75*h), w)
        elif self.edgeValue - self.getEdge(newMask, int(0.75*h), w) > jogThreshold:
            self.leftJog()
            self.edgeValue = self.getEdge(newMask, int(0.75*h), w)
        else:
            velocity = Twist()
            leftEdge = int(3*w/7)
            rightEdge = int(4*w/7)
            topEdge = int(0.4*h)
            croppedBlur = newMask[topEdge:h, leftEdge:rightEdge]
            imgSum = int(np.sum(croppedBlur))
            sumText = str(imgSum)
            print(sumText)
            threshold = 6250
            if imgSum > threshold:
                self.counter = self.counter + 1
                if self.counter > 1:
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

                    # if self.counter > 2:
                    #     velocity = Twist()
                    #     self.publishVel.publish(velocity)
                    #     # rospy.sleep(3.000)
                    #     # self.backwardStep()
                    #     sleep(0.050)
                    #     #self.leftTurn()
                    #     # self.counter = 0
                    #     print("stopped")
                    #     sleep(3.050)
                    #     velocity = Twist()
                    #     velocity.linear.x = 0.4
                    #     velocity.angular.z = 0.0
                    #     self.publishVel.publish(velocity)
                    #     sleep(0.050)
                    #     self.counter = 0

            # else:
            #     # elf.counter = self.counter + 1
            #     velocity = Twist()
            #     velocity.linear.x = 0.4
            #     velocity.angular.z = 0.0
            #     self.publishVel.publish(velocity)
            #     self.counter = 0
        # elif(leftTotal < 1):
        #     # left turn intersection
        #     center = edgeConditions[1] - int(0.23*w)  # approximate lane center for the intersection
        #     if (center < 0):
        #         center = 0
        #     text = "intersection"
        # else:
        #     center = self.getCenter(edgeConditions, gap)
        #     center = center + offset
        #     text = "straight"
        #     # enter = edgeConditions[1] - int(0.23*w)
        # else:
        #     velocity = Twist()
        #     velocity.linear.x = 0.4
        #     velocity.angular.z = 0.0
        #     self.publishVel.publish(velocity)
        #     self.counter = 0

        # compute state 0 through 9
        # self.imagePresent("camera", text, center, edgeConditions, cv_image, h, w)
        # self.imagePresent("mask", text, center, edgeConditions, interMask, h, w)
        # cv2.imshow("Camera", cv_image)
        # cv2.waitKey(1)
        # print(text)
        # stateNumber = center / gap
        # error = abs(center - int(w/2))
        # velocity = Twist()

        # goes through different options of turning

        # if stateNumber > 10:
        #     self.rightJog(error - int(gap/2))
        # elif stateNumber < 8:
        #     # turn left
        #     self.leftJog(error - int(gap/2))
        # else:
        #     # go straight
        #     self.forwardJog(error)

        # return velocity

        # cv2.imshow("Camera", cv_image)
        # cv2.waitKey(1)

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
