#!/usr/bin/env python

# Crops the images of the pedestrian for generating data

import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
from PIL import Image

"""
@brief: Determines which label to give the image
        (i.e. which directory to place the image in)
@param: fileName - name of image file
@return: relPath - character representing the directory
                   to save image in
"""
def getImagePath(fileName):
    relPath = "test/"  # CHANGE!!!!!!
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
    cv2.imwrite(path, img_BGR)


# ================================================================

RELATIVE_PATH = 'test/'  # FILL IN!!!!
files = os.listdir(RELATIVE_PATH)
fileName = 'pedestrian240.jpg'  # 240, 212, 372, 467

setNum = 1

# for fileName in files[:]:
for i in range(1):
    print(fileName)
    cameraImg = cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))
    cameraImgHSV = cv2.cvtColor(cameraImg, cv2.COLOR_RGB2HSV)

    # Get height and width values
    height, width = cameraImg.shape[0:2]

    whiteThreshold = 250
    CHECK_HEIGHT_FOR_CROPPING = height - 250
    left_x = 0
    right_x = 0

    for x in range(width):
        imgColour = cameraImgHSV[CHECK_HEIGHT_FOR_CROPPING, x]

        if(imgColour[2] >= whiteThreshold):
            left_x = x
            break

    for x in range(width):
        imgColour = cameraImgHSV[CHECK_HEIGHT_FOR_CROPPING, width - x - 1]

        if(imgColour[2] >= whiteThreshold):
            right_x = width - x - 1
            break

    print(left_x, right_x)
    croppedImg = cameraImg[:, left_x:right_x, :]
    croppedHeight, croppedWidth = croppedImg.shape[0:2]

    # STANDARD_HEIGHT = 180.0  # RANDOM, NEED TO CHANGE
    # STANDARD_WIDTH = 230.0  # RANDOM, NEED TO CHANGE

    # scalingHeightFactor = STANDARD_HEIGHT/croppedHeight
    # scalingWidthFactor = STANDARD_WIDTH/croppedWidth

    # # Resize the cropped image to standard size
    # resizedImg = cv2.resize(croppedImg, (int(croppedWidth*scalingWidthFactor),
    #                                      int(croppedHeight*scalingHeightFactor)),
    #                         interpolation=cv2.INTER_CUBIC)

    # print("Set: ", setNum, " saved")
    # setNum = setNum + 1
    # saveImage(resizedImg, fileName)

    plt.imshow(cameraImg), plt.show()
    plt.imshow(cameraImgHSV), plt.show()
    plt.imshow(croppedImg), plt.show()
