#!/usr/bin/env python

# This is the priliminary control script that will talk
# with the competition rostopics

from __future__ import print_function

import roslib
roslib.load_manifest('convolution_net')
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

from PIL import Image as PIL_Image
import numpy as np

from imageCrop_for_CNN import imageCrop
from keras.models import load_model

from time import sleep


class controlNode:

    def __init__(self):
        print("CN node started init.")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.publishVel = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
        self.publishLP = rospy.Publisher("/license_plate", String, queue_size=1)

        self.inLoop = False
        self.cooldownFlag = False
        self.going = True
        self.counter = 0
        self.cooldown = 0
        self.cornerNumber = 0
        self.edgeValue = 0
        self.pedCounter = 0
        self.crosswalkCooldown = 0

        self.initialMsgSent = False

        self.counterLP = 25
        self.tempCounter = 0
        self.teamName = "Grimm"
        self.teamPassword = ""
        self.answerKey = ["A", "B", "C", "D", "E", "F", "G",
                         "H", "I", "J", "K", "L", "M", "N",
                         "O", "P", "Q", "R", "S", "T", "U",
                         "V", "W", "X", "Y", "Z", "0", "1",
                         "2", "3", "4", "5", "6", "7", "8",
                         "9"]

        print("CN node done init.")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.determineVelocity(cv_image)

        if(self.initialMsgSent is False):
            initialMsg = str(self.teamName + ',' + self.teamPassword + ',' + '0' + ',' + 'AA11')
            self.publishLP.publish(initialMsg)

            self.initialMsgSent = True
        # Checks if robot is at parking lot

        parkingLotFlag = self.atParkingLot(cv_image)
        # print(parkingLotFlag)

        if parkingLotFlag is True and (self.counterLP - self.tempCounter) >= 15:
            print("At parking lot\n")
            # Stops the robot
            velocity = Twist()
            self.publishVel.publish(velocity)

            # Determines license plate and publishes message
            LP_msg = self.determineLicensePlate(cv_image)
            print(LP_msg)
            fullMsg = str(self.teamName + ',' + self.teamPassword + ',' + LP_msg[0] + ',' + LP_msg[1:])
            self.publishLP.publish(fullMsg)

            parkingLotFlag = False
            self.tempCounter = self.counterLP

        else:
            self.counterLP = self.counterLP + 1



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
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
        # thresholdSum = [10000000, 17000000]
        thresholdSum = [4000000, 6500000]
        # thresholdSum = [5000000, 7500000]
        # print("imgSum: ", imgSum)

        # Robot is at parking lot
        if imgSum > thresholdSum[0] and imgSum < thresholdSum[1]:
            flag = True

        return flag

    """
    @brief:  Determiens the license plate and the parking lot ID
             given the robot's raw camera image
    @param:  cameraImg - Robot's raw camera image of the parking lot
    @return: A string containing the license plate (determined by
             the license plates and ID convolution model) and the
             parking lot ID
    """

    def determineLicensePlate(self, cameraImg):
        # Calls on image cropper which crops the robot's raw camera
        # image and saves the 5 images to a local folder: competitionImages/
        RGB_cameraImg = cv2.cvtColor(cameraImg, cv2.COLOR_BGR2RGB)
        imageCrop(RGB_cameraImg)
        LPModel = load_model('ConvolutionModels/LPModel_4.h5')
        LP_msg = ""

        print("model loaded")

        RELATIVE_PATH = "competitionImgs/"
        # files = os.listdir(RELATIVE_PATH)
        files = ["img_0.jpg", "img_1.jpg", "img_2.jpg", "img_3.jpg",
                 "img_4.jpg"]

        # for fileName in files[:]:
        for i in range(5):
            letterNumImg = np.array(PIL_Image.open(RELATIVE_PATH + files[i]))

            letterNumImg = letterNumImg/255.0

            resizedImg = np.reshape(letterNumImg, [1, 39, 36, 3])

            predictions = LPModel.predict(resizedImg)
            # print("Predictions: ", predictions)

            if(i == 1 or i == 2):
                # it's a letter
                index = np.where(predictions == np.amax(predictions[:, 0:26]))
            else:
                # it's a number
                index = np.where(predictions == np.amax(predictions[:, 26:36]))
            # index = np.where(predictions == np.amax(predictions))
            # print("Index of max value: ", index)
            index = int(index[1])
            character = self.answerKey[index]
            # print("Character: ", character)
            LP_msg = LP_msg + character

        return LP_msg
    
    """
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
    """
    # def edgePass(self, height, newMask, w):
    #     left = -34
    #     right = -34
    #     searchIndent = int(-1.00*height + 700)
    #     # print(searchIndent)

    #     for x in range(searchIndent, w - searchIndent):
    #         if (newMask[height, x] > 0):  # looks for mask to go high
    #             left = x
    #             break

    #     for x in range(searchIndent, w - searchIndent):
    #         if (newMask[height, w-x-1] > 0):  # looks for mask to go high
    #             right = w-x
    #             break
    #     if (left <= searchIndent + 5):
    #         gotLeft = False
    #     else:
    #         gotLeft = True
    #     if (right >= w - searchIndent - 5 or right == -34):
    #         gotRight = False
    #     else:
    #         gotRight = True

    #     return left, right, gotLeft, gotRight
    
    """
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
    """
    # def imagePresent(self, name, text, center, edgeConditions, cv_image, h, w): # This function is not up to date, but isn't useful
    #     top = int(0.65*h)
    #     mid = int(0.70*h)
    #     bottom = int(0.75*h)
    #     topSearchIndent = int(-1.00*int(0.65*h) + 700)
    #     midSearchIndent = int(-1.00*int(0.70*h) + 700)
    #     bottomSearchIndent = int(-1.00*int(0.75*h) + 700)
    #     cv2.circle(cv_image, (center, bottom), 10, (0, 255, 0), -1)
    #     cv2.line(cv_image, (bottomSearchIndent, bottom), (w - bottomSearchIndent, bottom), (255, 0, 0), 3)
    #     cv2.line(cv_image, (midSearchIndent, mid), (w - midSearchIndent, mid), (255, 0, 0), 3)
    #     cv2.line(cv_image, (topSearchIndent, top), (w - topSearchIndent, top), (255, 0, 0), 3)
    #     cv2.line(cv_image, (int(w/2), 0), (int(w/2), h), (255, 0, 255), 3)
    #     cv2.putText(cv_image, text, (5, 100), cv2.FONT_HERSHEY_SIMPLEX,  1.0, (0, 0, 255), lineType=cv2.LINE_AA)
    #     cv2.putText(cv_image, str(edgeConditions), (5, 50), cv2.FONT_HERSHEY_SIMPLEX,  0.5, (0, 0, 255), lineType=cv2.LINE_AA)
    #     cv2.imshow(name, cv_image)
    #     cv2.waitKey(1)

    """
    @brief:  Determines the center of the road given a set of edge conditions
    @param:  edgeConditions - the list of edge locations and pass/fail booleans
             gap - the size of each segmentation of the image
    @return: center - the horizontal pixel location of the center of the road
    """
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

    """
    @brief:  Performs a large left turn movement(used to rotate the robot such
             that it faces the correct direction in the outer loop)
    """
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

    """
    @brief:  Performs a large forward movement(used to move the robot into
             the outer loop)
    """
    def forwardStep(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.015)
        # step forward into the loop
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(1.200)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        rospy.sleep(0.015)

    """
    @brief:  Performs a slight left turn (used to align the camera center
             road center)
    """
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

    """
    @brief:  Performs a slight right turn (used to align the camera center
             road center)
    """
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

    """
    @brief:  Performs a slight forward movement (used to ensure that
             the robot does not move forward too quickly
    """
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
    #     # go forward slightly
    #     velocity.linear.x = 0.4
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
    #     rospy.sleep(jogTime)
    #     # stop forward motion
    #     velocity.linear.x = 0.0
    #     velocity.angular.z = 0.0
    #     self.publishVel.publish(velocity)
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

    def getEdge(self, cv_image, height, w, edgeValue):
        for x in range(int(w/2), w):
            if (cv_image[height, x] > 0):
                return x
        print("edge Failure")
        return edgeValue
    """
    @brief:  Determines the appropriate movement and calls the function to 
             run said movement
    @param:  camerImg - Robot's raw camera image
    """
    # FOR ZACH TO COMPLETE -> Zach has completed
    def determineVelocity(self, cv_image):
        # get a mask for the road color
        # print(self.pedCounter)
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
            # print(self.cooldown)
            if self.cooldown < 0:
                self.cooldown = 0
                self.cooldownFlag = False
                self.edgeValue = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
        else:
            # print(self.edgeValue)
            newEdge = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
            # print(newEdge)
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


def main(args):
    print("START")
    rospy.init_node('controlNode', anonymous=True)
    print("Init ros")
    cn = controlNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
