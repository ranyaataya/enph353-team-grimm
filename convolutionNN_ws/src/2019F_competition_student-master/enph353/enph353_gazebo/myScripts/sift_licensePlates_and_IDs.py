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
fileName = 'testImg_27.jpg'
blankImgName = 'blank_plate.png'

files = os.listdir(RELATIVE_PATH)

cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))
blankPlateImg = np.array(Image.open(blankImgName))

# get height and width values
height, width = cameraImg.shape[0:2]

# x direction
left_x = -1
right_x = -1
# thresholdColour_blue = [5, 5, 100]
thresholdColour_blue = [1, 0, 102]
thresholdColour_white = [100, 100, 100]
heightThreshold = 250

# =============
# test thing
# testarray = [[[32, 45, 78], [12, 34, 97], [56, 66, 78], [45, 24, 78]], [[32, 45, 78], [12, 34, 97], [56, 86, 78], [45, 24, 68]]]
# testthing = [56, 66, 78]
# num = 0
# for index, row in reversed(list(enumerate(testarray))):
#     print(np.array(row))
#     print(np.array(testthing))
#     if (testthing in row):
#         print("TEST: found on row:", index)
#         print(num)
#         break
#     num = num + 1
# =============


def findHeightThreshold(cameraImg, thresholdValue):
    print(thresholdValue)
    for index, row in reversed(list(enumerate(cameraImg))):
        # rowNonNP = list(row)
        if (thresholdValue in row):
            print("found on row:", index)
            break


findHeightThreshold(cameraImg, thresholdColour_blue)
# plt.imshow(cameraImg), plt.show()

for x in range(width):
    imgColour = cameraImg[height - heightThreshold, x]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        left_x = x
        break

for x in range(width):
    imgColour = cameraImg[height - heightThreshold, width - x - 1]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        right_x = width - x
        break

# y direction
down_y = -1
up_y = -1
RIGHT_THRESHOLD = 10

for y in range(height):
    imgColour = cameraImg[y, right_x - RIGHT_THRESHOLD]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        up_y = y
        break

for y in range(height):
    imgColour = cameraImg[height - y - 1, right_x - RIGHT_THRESHOLD]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        down_y = height - y
        break

print("left x: ", left_x, "\n right x: ", right_x, "\n up y: ", up_y, "\n down y: ", down_y)

croppedImg = cameraImg[up_y:down_y, left_x:right_x, :]
plt.imshow(croppedImg), plt.show()
# ==================================================================

# Initiate ORB detector
orb = cv2.ORB_create()

# Find keypoints and descriptors in images
keypointsFull, descriptorsFull = orb.detectAndCompute(cameraImg, None)
keypointsBlank, descriptorsBlank = orb.detectAndCompute(blankPlateImg, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptorsFull, descriptorsBlank)
matches = sorted(matches, key=lambda x: x.distance)

outImg = cv2.drawMatches(blankPlateImg, keypointsBlank, cameraImg, keypointsFull, matches[:100], None, flags=2)

# plot the matches on the combined image
# plt.imshow(outImg), plt.show()
