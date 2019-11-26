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
        self.initialMsgSent = False

        self.counter = 0
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
        # self.determineVelocity(cv_image)

        if(self.initialMsgSent is False):
            initialMsg = str(self.teamName + ',' + self.teamPassword + ',' + '0' + ',' + 'AA11')
            self.publishLP.publish(initialMsg)

            self.initialMsgSent = True
        # Checks if robot is at parking lot

        parkingLotFlag = self.atParkingLot(cv_image)

        if parkingLotFlag is True and (self.counter - self.tempCounter) >= 25:
            """
            velocity = Twist()
            self.publishVel.publish(velocity)
            """
            # Determines license plate and publishes message
            LP_msg = self.determineLicensePlate(cv_image)
            print(LP_msg)
            fullMsg = str(self.teamName + ',' + self.teamPassword + ',' + LP_msg[0] + ',' + LP_msg[1:])
            self.publishLP.publish(fullMsg)

            # Determines the velocity twist message of the robot
            # and publishes it
            # velocity = self.determineVelocity(cv_image)
            # self.publishVel.publish(velocity)

            print("At parking lot\n")
            parkingLotFlag = False
            self.tempCounter = self.counter

        else:
            # Determines the velocity twist message of the robot
            # and publishes it
            # velocity = self.determineVelocity(cv_image)
            # self.publishVel.publish(velocity)
            self.counter = self.counter + 1



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
        thresholdSum = [10000000, 17000000]

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
        LPModel = load_model('ConvolutionModels/LPModel.h5')
        LP_msg = ""

        print("model loaded")

        RELATIVE_PATH = "competitionImgs/"
        # files = os.listdir(RELATIVE_PATH)
        files = ["img_0.jpg", "img_1.jpg", "img_2.jpg", "img_3.jpg",
                 "img_4.jpg"]

        # for fileName in files[:]:
        for i in range(5):
            letterNumImg = np.array(PIL_Image.open(RELATIVE_PATH + files[i]))
            resizedImg = np.reshape(letterNumImg, [1, 39, 36, 3])

            predictions = LPModel.predict(resizedImg)
            index = np.where(predictions == np.amax(predictions))
            index = int(index[1])
            character = self.answerKey[index]
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
    def edgePass(self, height, newMask, w):
        left = -34
        right = -34
        searchIndent = int(-0.85*height + 700)
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
    def imagePresent(self, name, text, center, edgeConditions, cv_image, h, w): # This function is not up to date, but isn't useful
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
    def leftTurn(self):
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(0.015)
        # turn ~90 degrees left
        velocity.linear.x = 0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        sleep(0.363)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)

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
        sleep(0.015)
        # step forward into the loop
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(0.400)
        velocity.linear.x = 0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        # for debug, stop and wait
        sleep(0.015)

    """
    @brief:  Performs a slight left turn (used to align the camera center
             road center)
    """
    def leftJog(self, error):
        jogDelay = 0.010
        jogTime = 0.02 + error*0.001
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(jogDelay)
        # turn left slightly
        velocity.linear.x = 0.0
        velocity.angular.z = 0.5
        self.publishVel.publish(velocity)
        sleep(jogTime)
        # stop turn
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)

    """
    @brief:  Performs a slight right turn (used to align the camera center
             road center)
    """
    def rightJog(self, error):
        jogDelay = 0.010
        jogTime = 0.02 + error*0.005
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(jogDelay)
        # turn right slightly
        velocity.linear.x = 0.0
        velocity.angular.z = -0.5
        self.publishVel.publish(velocity)
        sleep(jogTime)
        # stop turn
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)

    """
    @brief:  Performs a slight forward movement (used to ensure that
             the robot does not move forward too quickly
    """
    def forwardJog(self, error):
        jogDelay = 0.010
        jogTime = 0.07 - error*0.01
        if jogTime < 0.01:
            jogTime = 0.01
        velocity = Twist()
        # stop current motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(jogDelay)
        # go forward slightly
        velocity.linear.x = 0.4
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)
        sleep(jogTime)
        # stop forward motion
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.publishVel.publish(velocity)

    """
    @brief:  Determines the appropriate movement and calls the function to 
             run said movement
    @param:  camerImg - Robot's raw camera image
    """
    # FOR ZACH TO COMPLETE -> Zach has completed
    def determineVelocity(self, cameraImg):
        # get a mask for the road color
        # cv_image = cameraImg
        center = -34
        offset = 0
        lower_hsv = np.array([0, 0, 82])
        upper_hsv = np.array([100, 255, 85])

        image = cv2.cvtColor(cameraImg, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(image, lower_hsv, upper_hsv)
        interMask = cv2.medianBlur(mask, 7)
        newMask = interMask/255  # normalizes to 0 and 1

        h, w = newMask.shape[0:2]

        gap = int(w/20)

        # find the conditions for the edges of the given search lines
        edgeConditions = []
        searchLines = [int(0.83*h), int(0.67*h), int(0.56*h)]
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
        elif(leftTotal < 2):
            # left turn intersection
            center = edgeConditions[1] - int(0.12*w)  # approximate lane center for the intersection
            if (center < 0):
                center = 0
            text = "intersection"
        else:  # straights including parking space straights
            center = self.getCenter(edgeConditions, gap)
            center = center + offset
            text = "straight"

        stateNumber = center / gap
        error = abs(center - int(w/2))
        # goes through different options of turning

        if stateNumber > 10:
            self.rightJog(error - int(gap/2))
        elif stateNumber < 9:
            # turn left
            self.leftJog(error - int(gap/2))
        else:
            # go straight
            self.forwardJog(error)


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
