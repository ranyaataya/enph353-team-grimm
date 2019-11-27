#!/usr/bin/env python
# Augments images to have different brightnesses

import numpy as np
from keras.preprocessing.image import ImageDataGenerator
import cv2
from PIL import Image
import os

READING_PATH = "letters_and_numbers/U/"
files = os.listdir(READING_PATH)
WRITING_PATH = "lettersAndNums_brightness/U/"

counter = 56
for i in range(1):
    fileName = "U3097.jpg"  # files[i]
    img = np.array(Image.open(READING_PATH + fileName))

    samples = np.expand_dims(img, 0)
    datagen = ImageDataGenerator(brightness_range=[0.2, 1.0])
    it = datagen.flow(samples, batch_size=1)

    character = fileName[0]
    print(character)

    for j in range(7):
        batch = it.next()
        image = batch[0].astype('uint8')

        img_BGR = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # define character
        cv2.imwrite(WRITING_PATH + character + str(counter) + ".jpg", img_BGR)  # imwrite assumes BGR
        counter = counter + 1
