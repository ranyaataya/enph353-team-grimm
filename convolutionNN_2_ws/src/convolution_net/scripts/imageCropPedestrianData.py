#!/usr/bin/env python

# Author: Ranya Ataya
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
    relPath = "pedestrian_and_emptyRoad/pedestrian/" + fileName
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
    print(path)
    print("\n")
    cv2.imwrite(path, img_BGR)


# ================================================================

RELATIVE_PATH = 'pedestrianCNN_rawData/'
files = os.listdir(RELATIVE_PATH)

setNum = 1

for fileName in files[:]:
    print(fileName)
    cameraImg = cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))

    # Get height and width values
    height, width = cameraImg.shape[0:2]

    whiteThreshold = 250
    CHECK_HEIGHT_FOR_CROPPING = height - 200

    left_x = 0
    right_x = 0

    # Convert image from RGB to black and white
    cameraBW = cv2.cv2.cvtColor(cameraImg, cv2.COLOR_BGR2GRAY)

    for x in range(width):
        imgColour = cameraBW[CHECK_HEIGHT_FOR_CROPPING, x]
        if(imgColour >= whiteThreshold):
            left_x = x
            break

    for x in range(width):
        imgColour = cameraBW[CHECK_HEIGHT_FOR_CROPPING, width - x - 1]

        if(imgColour >= whiteThreshold):
            right_x = width - x - 1
            break

    print(left_x, right_x)

    # Crop image
    croppedImg = cameraImg[:, left_x:right_x, :]
    croppedHeight, croppedWidth = croppedImg.shape[0:2]

    STANDARD_HEIGHT = 720.0
    STANDARD_WIDTH = 346.0

    scalingHeightFactor = STANDARD_HEIGHT/croppedHeight
    scalingWidthFactor = STANDARD_WIDTH/croppedWidth

    # Resize the cropped image to standard size
    resizedImg = cv2.resize(croppedImg, (int(croppedWidth*scalingWidthFactor),
                                         int(croppedHeight*scalingHeightFactor)),
                            interpolation=cv2.INTER_CUBIC)

    print("Set: ", setNum, " saved")
    setNum = setNum + 1
    saveImage(resizedImg, fileName)

    # To view image use: plt.imshow(cameraImg), plt.show()
