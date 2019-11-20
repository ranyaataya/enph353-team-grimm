#!/usr/bin/env python

import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
from PIL import Image

"""
@brief: Crops the raw camera image from the robot's camera.
#       This is the main function that is called on by other
#       scripts.
@param: cameraImg - robot's raw camera Img
"""
def imageCrop(cameraImg):

    # Get height and width values
    height, width = cameraImg.shape[0:2]

    # Threshold colour value
    thresholdColour_blue = [1, 0, 102]
    threshold_blue_to_grey = 10
    # ============================
    BLUE_DIFF_RANGE = [80, 107]
    BLUE_DIFF_RANGE_Y = [93, 107]
    GREY_DIFF = 7
    GREY_THRESHOLD = 95
    # ============================

    heightThresholds = findHeightThreshold(cameraImg, BLUE_DIFF_RANGE_Y)

    # x Direction
    left_x = 0
    right_x = 0
    flag = False

    for x in range(width):
        imgColour = cameraImg[heightThresholds[1], x]  # RGB

        if((abs(int(imgColour[2]) - int(imgColour[0])) >= BLUE_DIFF_RANGE[0] and
            abs(int(imgColour[2]) - int(imgColour[0])) <= BLUE_DIFF_RANGE[1]) and
           (abs(int(imgColour[2]) - int(imgColour[1])) >= BLUE_DIFF_RANGE[0] and
           abs(int(imgColour[2]) - int(imgColour[1])) <= BLUE_DIFF_RANGE[1])):
            flag = True

        if(flag is True and
           abs(int(imgColour[2]) - int(imgColour[1])) <= GREY_DIFF and
           abs(int(imgColour[2]) - int(imgColour[0])) <= GREY_DIFF and
           abs(int(imgColour[1]) - int(imgColour[0])) <= GREY_DIFF and
           int(imgColour[0]) >= GREY_THRESHOLD):
            left_x = x
            break

    flag = False
    for x in range(width):
        imgColour = cameraImg[heightThresholds[1], width - x - 1]

        if((abs(int(imgColour[2]) - int(imgColour[0])) >= BLUE_DIFF_RANGE[0] and
            abs(int(imgColour[2]) - int(imgColour[0])) <= BLUE_DIFF_RANGE[1]) and
           (abs(int(imgColour[2]) - int(imgColour[1])) >= BLUE_DIFF_RANGE[0] and
           abs(int(imgColour[2]) - int(imgColour[1])) <= BLUE_DIFF_RANGE[1])):
            flag = True

        if(flag is True and
           abs(int(imgColour[2]) - int(imgColour[1])) <= GREY_DIFF and
           abs(int(imgColour[2]) - int(imgColour[0])) <= GREY_DIFF and
           abs(int(imgColour[1]) - int(imgColour[0])) <= GREY_DIFF and
           int(imgColour[0]) >= GREY_THRESHOLD):
            right_x = width - x
            break

    # y Direction
    up_y = heightThresholds[0]
    down_y = heightThresholds[2]

    # Crop Image
    croppedImg = cameraImg[up_y:down_y, left_x:right_x, :]
    croppedHeight, croppedWidth = croppedImg.shape[0:2]

    # scale cropped image to standard size, determine scaling factors
    # standard numbers based on sample image
    STANDARD_HEIGHT = 195.0
    STANDARD_WIDTH = 222.0

    scalingHeightFactor = STANDARD_HEIGHT/croppedHeight
    scalingWidthFactor = STANDARD_WIDTH/croppedWidth

    # [lower height, upper height, lower width, upper width]
    LP_bounds = [0.7, 0.9]
    lotID_bounds = [0.41, 0.63]

    # Resize the cropped image to standard size
    resizedImg = cv2.resize(croppedImg, (int(croppedWidth*scalingWidthFactor),
                                         int(croppedHeight*scalingHeightFactor)),
                            interpolation=cv2.INTER_CUBIC)

    # Create license plate image and lot ID image
    licensePlate_img = resizedImg[int(LP_bounds[0]*STANDARD_HEIGHT):
                                  int(LP_bounds[1]*STANDARD_HEIGHT), :]

    lotID_img = resizedImg[int(lotID_bounds[0]*STANDARD_HEIGHT):
                           int(lotID_bounds[1]*STANDARD_HEIGHT), :]

    # Divide images into separate letters and numbers
    lot_img_letter = lotID_img[:, 0:int(lotID_img.shape[1]/2), :]
    lot_img_num = lotID_img[:, int(lotID_img.shape[1]/2)+1:lotID_img.shape[1], :]

    # LP_charBounds = [17, 53, 54, 90, 126, 162, 165, 201]
    LP_charBounds = [17, 53, 55, 91, 132, 168, 170, 206]

    LP_img1 = licensePlate_img[:, LP_charBounds[0]:LP_charBounds[1], :]
    LP_img2 = licensePlate_img[:, LP_charBounds[2]:LP_charBounds[3], :]
    LP_img3 = licensePlate_img[:, LP_charBounds[4]:LP_charBounds[5], :]
    LP_img4 = licensePlate_img[:, LP_charBounds[6]:LP_charBounds[7], :]

    # Resize lot ID letters and numbers
    scaling_lotIDL_HeightFactor = float(LP_img1.shape[0])/lot_img_letter.shape[0]
    scaling_lotIDL_WidthFactor = float(LP_img1.shape[1])/lot_img_letter.shape[1]

    scaling_lotIDN_HeightFactor = float(LP_img1.shape[0])/lot_img_num.shape[0]
    scaling_lotIDN_WidthFactor = float(LP_img1.shape[1])/lot_img_num.shape[1]

    lot_img_letter = cv2.resize(lot_img_letter,
                                (int(lot_img_letter.shape[1] * scaling_lotIDL_WidthFactor),
                                    int(lot_img_letter.shape[0] * scaling_lotIDL_HeightFactor)),
                                interpolation=cv2.INTER_CUBIC)
    lot_img_num = cv2.resize(lot_img_num,
                             (int(lot_img_num.shape[1]*scaling_lotIDN_WidthFactor),
                              int(lot_img_num.shape[0]*scaling_lotIDN_HeightFactor)),
                             interpolation=cv2.INTER_CUBIC)

    saveImage(lot_img_num, "img_0.jpg")
    saveImage(LP_img1, "img_1.jpg")
    saveImage(LP_img2, "img_2.jpg")
    saveImage(LP_img3, "img_3.jpg")
    saveImage(LP_img4, "img_4.jpg")
    print("imageCrop Hi")

"""
@brief: Finds the row at which the middle of the parking lot's height occurs
@param: cameraImg      - image from robot's camera
        thresholdValue - RGB value of the colour at which the height should be
                         found
@return: rowIndexList  - an array of rows the min, average, and max row given
                         the threshold value
"""
def findHeightThreshold(cameraImg, thresholdValue):
    rowIndices = []

    for currRow, row in reversed(list(enumerate(cameraImg))):
        for pixel in row:
            if ((abs(int(pixel[2]) - int(pixel[0])) >= thresholdValue[0] and
                 abs(int(pixel[2]) - int(pixel[0])) <= thresholdValue[1]) and
                (abs(int(pixel[2]) - int(pixel[1])) >= thresholdValue[0] and
                 abs(int(pixel[2]) - int(pixel[1])) <= thresholdValue[1])):

                rowIndices.append(currRow)
                break

    rowIndexList = [min(rowIndices), rowIndices[int(len(rowIndices)/2)], max(rowIndices)]
    return rowIndexList


"""
@brief: Determines which label to give the image
        (i.e. which directory to place the image in)
@param: fileName - name of image file
        position - position of character in fileName
@return: relPath - character representing the directory
                   to save image in
"""
def getImagePath(fileName):
    global counter
    # relPath = "letters_and_numbers/" + fileName[position] + "/" + fileName[position] + str(counter) + ".jpg"
    # CHANGE RELATIVE PATH TO BE WHAT WE WANT!!!!
    relPath = "folder/" + fileName

    return relPath


"""
@brief: Saves image to specified path
@param: img  - image file
        fileName - name of image file
        position - position of character in fileName
@return: none
"""
def saveImage(img, fileName):
    path = getImagePath(fileName)
    img_BGR = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, img_BGR)  # imwrite assumes BGR
