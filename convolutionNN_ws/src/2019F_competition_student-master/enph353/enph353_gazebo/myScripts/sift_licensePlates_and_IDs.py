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

# get height and width values
height, width = cameraImg.shape[0:2]
heightBlank, widthBlank = blankPlateImg.shape[0:2]

# threshold values
# thresholdColour_blue = [5, 5, 100]
thresholdColour_blue = [1, 0, 102]

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
print(heightThresholds)
# x direction
for x in range(width):
    imgColour = cameraImg[heightThresholds[1], x]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        left_x = x
        break

for x in range(width):
    imgColour = cameraImg[heightThresholds[1], width - x - 1]

    if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
       and imgColour[2] >= thresholdColour_blue[2]):
        right_x = width - x
        break

# y direction
up_y = heightThresholds[0]
down_y = heightThresholds[2]

# RIGHT_THRESHOLD = 10

# for y in range(height):
#     imgColour = cameraImg[y, right_x - RIGHT_THRESHOLD]

#     if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
#        and imgColour[2] >= thresholdColour_blue[2]):
#         up_y = y
#         break

# for y in range(height):
#     imgColour = cameraImg[height - y - 1, right_x - RIGHT_THRESHOLD]

#     if(imgColour[0] <= thresholdColour_blue[0] and imgColour[1] <= thresholdColour_blue[1]
#        and imgColour[2] >= thresholdColour_blue[2]):
#         down_y = height - y
#         break

print("left x: ", left_x, "\n right x: ", right_x, "\n up y: ", up_y, "\n down y: ", down_y)

# crop image
croppedImg = cameraImg[up_y:down_y, left_x:right_x, :]
print(croppedImg.shape[:])
print(cameraImg.shape[:])
print(blankPlateImg.shape[:])

resizedImg = cv2.resize(blankPlateImg, (int(0.25*widthBlank), int(0.25*heightBlank)), interpolation=cv2.INTER_AREA)
print(resizedImg.shape[:])
plt.imshow(croppedImg), plt.show()
# ==================================================================

# Initiate ORB detector
orb = cv2.ORB_create()

# Find keypoints and descriptors in images
keypointsFull, descriptorsFull = orb.detectAndCompute(croppedImg, None)
keypointsBlank, descriptorsBlank = orb.detectAndCompute(resizedImg, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptorsFull, descriptorsBlank)
matches = sorted(matches, key=lambda x: x.distance)

# outImg = cv2.drawMatches(resizedImg, keypointsBlank, croppedImg, keypointsFull, matches[:10], None, flags=2)

# plot the matches on the combined image
# plt.imshow(outImg), plt.show()
