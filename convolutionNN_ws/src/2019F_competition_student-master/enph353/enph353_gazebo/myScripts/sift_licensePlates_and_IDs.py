#!/usr/bin/env python

# Author: Ranya Ataya
# Purpose: Overall purpose will be to run the training for the convolution
#          neural network. Initially the purpose is to test ORB with the
#          the license plates and parking lot IDs. ORB may make it easier
#          to extract the objects from the images since we are given a
#          blank license plate.

import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
import collections
from PIL import Image

RELATIVE_PATH = "grimmNPC_images/"
fileName = 'testImg_28.jpg'
blankImgName = 'blank_plate.png'

files = os.listdir(RELATIVE_PATH)

cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))
blankPlateImg = np.array(Image.open(blankImgName))

# Get height and width values
height, width = cameraImg.shape[0:2]
heightBlank, widthBlank = blankPlateImg.shape[0:2]

# Threshold colour value
thresholdColour_blue = [1, 0, 102]
threshold_blue_to_grey = 10

"""
Finds the row at which the middle of the parking lot's height occurs
"""
def findHeightThreshold(cameraImg, thresholdValue):
    rowIndices = []

    for currRow, row in reversed(list(enumerate(cameraImg))):
        for pixel in row:
            if tuple(thresholdValue) == tuple(pixel):
                rowIndices.append(currRow)
                break

    rowIndexList = [min(rowIndices), rowIndices[int(len(rowIndices)/2)], max(rowIndices)]
    return rowIndexList


heightThresholds = findHeightThreshold(cameraImg, thresholdColour_blue)

# x Direction
flag = False
for x in range(width):
    imgColour = cameraImg[heightThresholds[1], x]

    if(imgColour[0] <= thresholdColour_blue[0] and
       imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        flag = True

    if(flag is True and imgColour[0] > threshold_blue_to_grey and imgColour[1] > threshold_blue_to_grey):
        left_x = x
        break

flag = False
for x in range(width):
    imgColour = cameraImg[heightThresholds[1], width - x - 1]

    if(imgColour[0] <= thresholdColour_blue[0] and
       imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        flag = True

    if(flag is True and imgColour[0] > threshold_blue_to_grey and imgColour[1] > threshold_blue_to_grey):
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
LP_bounds = [0.7, 0.9]  # , 0.06, 0.72]
lotID_bounds = [0.41, 0.63]  # , 0.06, 0.72]

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

LP_charBounds = [17, 53, 54, 90, 126, 162, 165, 201]

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

print(fileName)
print(LP_img1.shape)
print(lot_img_letter.shape)
print(lot_img_num.shape)

# plt.imshow(lot_img_letter), plt.show()
# plt.imshow(LP_img2), plt.show()
# plt.imshow(LP_img3), plt.show()
# plt.imshow(LP_img4), plt.show()
