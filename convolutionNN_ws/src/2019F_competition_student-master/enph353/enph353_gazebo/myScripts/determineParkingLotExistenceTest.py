#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import os
from PIL import Image

RELATIVE_PATH = 'rawImages/'
files = os.listdir(RELATIVE_PATH)
fileName = "lot_P6_OO66_13.jpg"

cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))

cameraImg = cv2.cvtColor(cameraImg, cv2.COLOR_RGB2HSV)
lowerBlue = np.array([110, 50, 50])
upperBlue = np.array([130, 255, 255])

blueMask = cv2.inRange(cameraImg, lowerBlue, upperBlue)
maskedCameraImg = cv2.bitwise_and(cameraImg, cameraImg, mask=blueMask)
maskedCameraImg = maskedCameraImg[300:, :, :]

imgSum = np.sum(maskedCameraImg)
threshold = [8000000, 40000000]

if imgSum > threshold[0] and imgSum < threshold[1]:
    print("parking lot")
    print(imgSum)
else:
    print("road")
    print(imgSum)

# plt.imshow(cameraImg), plt.show()
# plt.imshow(maskedCameraImg), plt.show()
# plt.imshow(roadImg), plt.show()
