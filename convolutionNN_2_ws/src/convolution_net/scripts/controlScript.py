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
        # sets up necessary image subscriber and
        # velocity/license plate publisher
        print("CN node started init.")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image,
                                          self.callback)
        self.publishVel = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
        self.publishLP = rospy.Publisher("/license_plate", String, queue_size=1)

        self.inLoop = False  # true or false for in the loop
        self.cooldownFlag = False  # true or false for in cooldown after corner
        self.going = True  # true or false on the robot still driving
        self.counter = 0  # counter for corner threshold
        self.cooldown = 0  # holds value for corner cooldown
        self.cornerNumber = 0  # tracks which corner the robot is approaching
        # holds the value to the image center of the side of the road
        self.edgeValue = 0
        self.pedCounter = 0  # holds the state number for the pedestrain
        # holds cooldown for triggering on a crosswalk
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
    @brief:  Performs a large left turn movement(used to rotate the robot such
             that it faces the correct direction in the outer loop)
    @param:  cornerNumber - number representing that the robot is at the
             nth corner (used to custom fit each corner with a turning time)
    @return: boolean to tell the driving code whether a full loop has been
             completed or not
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
        # stop current motion
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.015)
        # step forward into the loop
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(1.200)
        # stop motion and wait
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(0.015)

    """
    @brief:  Performs a slight left turn (used to align the camera center
             road center)
    """

    def leftJog(self):
        jogDelay = 0.015
        jogTime = 0.040
        # stop current motion
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogDelay)
        # turn left slighty
        velocity.linear.x = 0.0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        rospy.sleep(jogTime)
        # stop motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)

    """
    @brief:  Performs a slight right turn (used to align the camera center
             road center)
    """

    def rightJog(self):
        jogDelay = 0.015
        jogTime = 0.035
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        rospy.sleep(jogDelay)
        # turn right slighty
        velocity.linear.x = 0.0
        velocity.angular.z = -0.5
        self.publishVel.publish(velocity)
        rospy.sleep(jogTime)
        # stop motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)

    """
    @brief:  checks camera image to see if the robot is
             at a crosswalk or not
    @param:  cv_image - the image to check for the red line
             indicating a cross walk
    @return: retval - boolean true or false for being at a
             crosswalk
    """

    def atCrosswalk(self, cv_image):
        retval = False
        # makes mask for the crosswalk's red line
        lowerRed = np.array([0, 150, 150])
        upperRed = np.array([5, 255, 255])

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        redMask = cv2.inRange(img, lowerRed, upperRed)
        h, w = redMask.shape[0:2]

        # checks if the threshold is exceeded, returns true
        # if this is the case (robot is at the crosswalk)
        imgSum = np.sum(redMask)
        if (imgSum > 350000):
            retval = True
        else:
            retval = False
        return retval

    """
    @brief:  checks camera image to see if the pedestrain is
             in the crosswalk or not
    @param:  cv_image - the image to check for the jeans of the
             pedestrain
    @return: retval - boolean true or false for the pedestrain in
             the crosswalk
    """

    def checkPedestrain(self, cv_image):
        retval = False
        # makes a mask for the pedestrains jeans
        lowerBlue = np.array([80, 60, 0])
        upperBlue = np.array([120, 200, 150])

        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        blueMask = cv2.inRange(img, lowerBlue, upperBlue)
        h, w = blueMask.shape[0:2]
        blur3 = cv2.medianBlur(blueMask, 7)
        blurProper = blur3[int(h/3):h, int(0.45*w):int(0.55*w)]

        # checks if the threshold is exceeded, returns true
        # if this is the case (pedestrain is present)
        imgSum = np.sum(blurProper)
        if (imgSum > 10000):
            retval = True
        else:
            retval = False
        return retval

    """
    @brief:  Uses a specified line of pixels to find the right side of the road
             so it can be related to the past value (gauges robot direction)
    @param:  newMask - normalized grass mask of theRobot's raw camera image
             height - the height on the image in which to search for the road
             w - the width of the image/mask
             edgeValue - the past value for the edge of the road
    @return: x or edgeValue - returns either the new road edge or the past
             road edge if the new edge is not found
    """

    def getEdge(self, cv_image, height, w, edgeValue):
        for x in range(int(w/2), w):
            if (cv_image[height, x] > 0):
                return x
        print("edge Failure")
        return edgeValue

    """
    @brief:  Determines the appropriate movement and calls the function to
             run said movement
    @param:  cv_image - Robot's raw camera image
    """

    def determineVelocity(self, cv_image):
        # decrements the crosswalkCooldown to a minumum value of 0
        self.crosswalkCooldown = self.crosswalkCooldown - 1
        if self.crosswalkCooldown < 0:
            self.crosswalkCooldown = 0

        # makes a mask of the grass
        lower_hsv = np.array([61, 50, 50])
        upper_hsv = np.array([81, 255, 255])

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]
        # sets the amount the robot must be off of
        # past center before jogging left or right
        # to correct
        jogThreshold = int(w/40)
        if(self.inLoop is False):  # checks to see if the robot is in the loop
            # steps robot into loop
            self.forwardStep()
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.leftTurn(self.cornerNumber)
            self.inLoop = True
            self.cornerNumber = self.cornerNumber + 1
            velocity = Twist()
            self.publishVel.publish(velocity)
            self.edgeValue = int(self.getEdge(newMask, int(0.9*h), w,
                                 self.edgeValue))
            #  sets cooldown to flush out image queue
            self.cooldownFlag = True
            self.cooldown = 20
        elif(self.going is False):  # checks if the robot has gone around
            # if the robot has gone all the way around, stops the robot
            velocity = Twist()
            velocity.linear.x = 0
            velocity.angular.z = 0.0
            self.publishVel.publish(velocity)
        # checks if the robot is at or stopped at a crosswalk
        elif self.atCrosswalk(cv_image) is True or self.pedCounter > 0:
            if self.crosswalkCooldown > 0:
                # if the robot has already stopped at the crosswalk, it goes
                velocity = Twist()
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
            elif self.pedCounter == 0:
                # if the robot has just seen the crosswalk, stops the robot
                velocity = Twist()
                velocity.linear.x = 0.0
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
                self.pedCounter = 1
            elif self.pedCounter == 1:
                # if the robot sees the pedestrian, go to next step
                if(self.checkPedestrain(cv_image) is True):
                    self.pedCounter = 2
            elif self.pedCounter == 2:
                if(self.checkPedestrain(cv_image) is False):
                    # if the robot has seen, but now doesn't see the pedestrian
                    # go forward, and begin cooldown for crosswalk
                    self.pedCounter = 0
                    velocity = Twist()
                    velocity.linear.x = 0.4
                    velocity.angular.z = 0.0
                    self.publishVel.publish(velocity)
                    self.crosswalkCooldown = 30
            else:
                print("Ped Failure")  # if you get here something is wrong

        # if cooldown has started and isn't done, decrement the cooldown
        # else if done, finish to cooldown and mark it done
        elif self.cooldownFlag is True:
            self.cooldown = self.cooldown - 1
            if self.cooldown < 0:
                self.cooldown = 0
                self.cooldownFlag = False
                self.edgeValue = int(self.getEdge(newMask, int(0.9*h), w,
                                     self.edgeValue))

        # if the cooldown is not in effect, check the
        # road edge and jog accordingly
        else:
            newEdge = int(self.getEdge(newMask, int(0.9*h), w, self.edgeValue))
            if self.edgeValue - newEdge < -jogThreshold:
                self.rightJog()
                self.edgeValue = newEdge
            if self.edgeValue - newEdge > jogThreshold:
                self.leftJog()
                self.edgeValue = newEdge

            # create a single channel h x w pixel black image
            # with a white area of interest as below
            contours = np.array([[int(0.42*w), int(0.75*h)],
                                 [int(0.48*w), int(0.25*h)],
                                 [int(0.49*w), int(0.25*h)],
                                 [int(0.55*w), int(0.75*h)]])
            img = np.zeros((h, w), dtype="uint8")
            cv2.fillPoly(img, pts=[contours], color=(255, 255, 255))
            finalMask = np.zeros((h, w), dtype="uint8")

            # combines area of interest and grass masks
            cv2.bitwise_and(newMask, img, finalMask)
            velocity = Twist()
            # computes the value of the mask, compares to the threshold
            # for each given corner
            imgSum = int(np.sum(finalMask))
            thresholdSet = [4000, 4000, 4250, 4250, 4300, 4250]
            threshold = thresholdSet[self.cornerNumber]
            if imgSum > threshold:
                self.counter = self.counter + 1  # acts as lowpass filter
                if self.counter > 0:
                    # if the robot is at a corner, turn and start up
                    # the cooldown counter to clear image queue
                    velocity = Twist()
                    self.publishVel.publish(velocity)
                    self.going = self.leftTurn(self.cornerNumber)
                    velocity = Twist()
                    self.publishVel.publish(velocity)
                    self.cornerNumber = self.cornerNumber + 1
                    self.cooldownFlag = True
                    self.cooldown = 20
            else:
                # if the robot isn't at a corner, just keep going
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
                self.publishVel.publish(velocity)
                self.counter = 0


def main(args):
    # initalizes and runs above driving node
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
