#!/usr/bin/env python

import cv2
import numpy as np
import os
from PIL import Image
from imageCrop_for_CNN import imageCrop

print("second script")

def myfunction(num):
    print("hey", num)


RELATIVE_PATH = 'rawImages/'
files = os.listdir(RELATIVE_PATH)
fileName = "lot_P6_OO66_13.jpg"

cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))

print("before")
# imageCrop(cameraImg)
print("complete")
