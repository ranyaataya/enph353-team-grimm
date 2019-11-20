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
import tensorflow as tf

from PIL import Image
import numpy as np

from imageCrop_for_CNN import imageCrop
from keras.models import load_model


class controlNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.publishVel = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
        self.publishLP = rospy.Publisher("/license_plate", String, queue_size=1)

        self.counter = 0
        self.tempCounter = 0
        self.teamName = "TGrimm"
        self.teamPassword = ""
        # PUBLISH THE INITIAL MESSAGE STATING OUR TEAM NUMBER!!!!!!!!

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Checks if robot is at parking lot
        parkingLotFlag = self.atParkingLot(cv_image)

        if parkingLotFlag is True and (self.counter - self.tempCounter) >= 5:

            # Determines license plate and publishes message
            LP_msg = self.determineLicensePlate(cv_image)
            self.publishLP.publish(LP_msg)

            # Determines the velocity twist message of the robot
            # and publishes it
            # velocity = self.determineVelocity(cv_image)
            # self.publishVel.publish(velocity)

            parkingLotFlag = False
            self.tempCounter = self.counter

        else:
            # Determines the velocity twist message of the robot
            # and publishes it
            # velocity = self.determineVelocity(cv_image)
            # self.publishVel.publish(velocity)
            self.counter = self.counter + 1

        velocity = self.determineVelocity(cv_image)


    """
    @brief:  Determines if the robot has reached a parking lot
    @param:  camerImg - robot's raw camera image
    @return: flag - boolean representing if the robot is at a
                    parking lot or not. (True = parking lot &
                    False = road)
    """
    def atParkingLot(self, cameraImg):
        flag = False
        img = np.array(cameraImg)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Define limits of blue in HSV
        lowerBlue = np.array([110, 50, 50])
        upperBlue = np.array([130, 255, 255])

        # Create and apply blue mask
        blueMask = cv2.inRange(img, lowerBlue, upperBlue)
        maskedImg = cv2.bitwise_and(img, img, mask=blueMask)
        maskedImg = maskedImg[300:, :, :]

        # Sum all the pixels in the image to represent the
        # area of blue in the image
        imgSum = np.sum(maskedImg)
        thresholdSum = [8000000, 40000000]

        # Robot is at parking lot
        if imgSum > thresholdSum[0] and imgSum < thresholdSum[1]:
            flag = True

        return flag

    """"
    @brief:  Determiens the license plate and the parking lot ID
             given the robot's raw camera image
    @param:  cameraImg - Robot's raw camera image of the parking lot
    @return: A string containing the license plate (determined by
             the license plates and ID convolution model) and the
             parking lot ID
    """"
    def determineLicensePlate(self, cameraImg):
        # Calls on image cropper which crops the robot's raw camera
        # image and saves the 5 images to a local folder: competitionImages/
        imageCrop(cameraImg)
        conModel = load_model('ConvolutionModels/LPModel.h5')
        LP_msg = ""

        RELATIVE_PATH = "competitionImgs/"
        files = os.listdir(RELATIVE_PATH)

        for fileName in files[:]:
            letterNumImg = np.array(Image.open(RELATIVE_PATH + fileName))
            resizedImg = np.reshape(letterNumImg, [1, 39, 36, 3])

            predictions = LPModel.predict()
            # LP_msg = LP_msg + result  # may be wrong
            # convert from predictions one hot to strings

        return LP_msg

    """"
    @brief:  Uses a specified line of pixels to find the sides of the road
             and gives a pass or fail depending on its success/ meaningfullness
    @param:  newMask - normalized road mask of theRobot's raw camera image
             height - the height on the image in which to search for the road
             w - the width of the image/mask
    @return: left - the horizontal pixel location of the left side
                    of the road, or -34 if this failed
             right - the horizontal pixel location of the right side
                     of the road, or -34 if this failed
             gotLeft - success of getting the left side of the road
             gotRight - success of getting the right side of the road
    """"
    def edgePass(self, height, newMask, w):
        left = -34
        right = -34
        searchIndent = int(-1.00*height + 700)
        # print(searchIndent)

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, x] > 0):  # looks for mask to go high
                left = x
                break

        for x in range(searchIndent, w - searchIndent):
            if (newMask[height, w-x-1] > 0):  # looks for mask to go high
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

    """"
    @brief:  A debugging function that show the camera view
             and draws the search lines, state and edge conditions
             on the image
    @param:  name - the name of the window to present the image in
             cv_image - Robot's raw camera image of the parking lot
             text - the name of the state the robot detects
                    (e.g. intersection, straight, etc)
             center - the horizontal pixel location of the road center
             edgeConditions - the list of edge locations and pass/fail booleans
             h - height of image in pixels
             w - width of image in pixels
    """"
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

    """"
    @brief:  Determines the center of the road given a set of edge conditions
    @param:  edgeConditions - the list of edge locations and pass/fail booleans
             gap - the size of each segmentation of the image
    @return: center - the horizontal pixel location of the center of the road
    """"
    def getCenter(self, edgeConditions, gap):
        usableEdges = []
        centers = []
        for line1 in range((len(edgeConditions)/4)):
            if edgeConditions[2 + 4*line1] is True:
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

    """"
    @brief:  Performs a large left turn movement(used to rotate the robot such
             that it faces the correct direction in the outer loop)
    """"
    def leftTurn(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.015)
        # turn ~90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publish.publish(velocity)
        sleep(0.363)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)

    """"
    @brief:  Performs a large forward movement(used to move the robot into
             the outer loop)
    """"
    def forwardStep(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.015)
        # step forward into the loop
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.400)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)

    """"
    @brief:  Performs a slight left turn (used to align the camera center
             road center)
    """"
    def leftJog(self):
        jogDelay = 0.015
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(jogDelay)
        # turn left a small amount
        velocity.linear.x = 0.0
        velocity.angular.z = 0.5
        self.publish.publish(velocity)
        sleep(0.017)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)

    """"
    @brief:  Performs a slight right turn (used to align the camera center
             road center)
    """"
    def rightJog(self):
        jogDelay = 0.015
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(jogDelay)
        # turn right a small amount
        velocity.linear.x = 0.0
        velocity.angular.z = -0.5
        self.publish.publish(velocity)
        sleep(0.017)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)

    """"
    @brief:  Performs a slight forward movement (used to ensure that
             the robot does not move forward too quickly
    """"
    def forwardJog(self):
        jogDelay = 0.015
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(jogDelay)
        # turn 90 degrees left
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publish.publish(velocity)
        sleep(0.04)
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publish.publish(velocity)

    """
    @brief:  Determines the appropriate movement and calls the function to 
             run said movement
    @param:  camerImg - Robot's raw camera image
    """
    # FOR ZACH TO COMPLETE -> Zach has completed
    def determineVelocity(self, cameraImg):
        # get a mask for the road color
        cv_image = cameraImg
        center = -34
        offset = 0
        lower_hsv = np.array([0, 0, 82])
        upper_hsv = np.array([100, 255, 85])

        image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]

        gap = int(w/20)

        # find the conditions for the edges of the given search lines
        edgeConditions = []
        searchLines = [int(0.83*h), int(0.72*h), int(0.61*h)]
        for search in range(len(searchLines)):
            left, right, gotLeft, gotRight = self.edgePass(searchLines[search], newMask, w)
            edgeConditions.append(left)
            edgeConditions.append(right)
            edgeConditions.append(gotLeft)
            edgeConditions.append(gotRight)

        leftTotal = edgeConditions[2] + edgeConditions[6]  # + edgeConditions[10]
        rightTotal = edgeConditions[3] + edgeConditions[7]  # + edgeConditions[11]

        text = "Fail"

        if(self.inLoop is False):
            self.forwardStep()
            self.leftTurn()
            self.inLoop = True
        if (edgeConditions[10] == 0 and edgeConditions[11] == 0):  # either totally lost or at a T intersection
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

        stateNumber = center / gap

        # goes through different options of turning

        if stateNumber > 10:
            self.rightJog()
        elif stateNumber < 9:
            # turn left
            self.leftJog()
        else:
            # go straight
            self.forwardJog()


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
