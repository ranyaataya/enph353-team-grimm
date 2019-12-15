#!/usr/bin/env python

# Author: Ranya Ataya
# This script is just used as a place to test small random
# chunks of code. Each chunk had various purposes for testing

import cv2
import numpy as np
import os
from PIL import Image
from imageCrop_for_CNN import imageCrop

from keras.models import load_model
import keras.preprocessing.text

# import keras.models

# print("second script")

# def myfunction(num):
#     print("hey", num)

RELATIVE_PATH = 'letters_and_numbers/M/'
files = os.listdir(RELATIVE_PATH)
fileName = "M551.jpg"

cameraImg = np.array(Image.open(RELATIVE_PATH + fileName))
resizedImg = np.reshape(cameraImg, [1, 39, 36, 3])

# print("before")
# # imageCrop(cameraImg)
# print("complete")

# print("about to load model")
# model = load_model('ConvolutionModels/LPModel.h5')
# print("model loaded")

# predictions = model.predict(resizedImg)
# print(predictions)

# index = np.where(predictions == np.amax(predictions))
# index = int(index[1])
# # index = index[1:len(index) - 1]
# print(index)

tem = "Grimm"
pas = "mutli32"
n = "9njmk"
example = str(tem + ',' + pas + ',' +  n[0] + ',' +  n[1:])
print(example)
