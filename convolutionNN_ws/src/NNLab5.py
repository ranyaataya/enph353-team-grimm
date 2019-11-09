#!/usr/bin/env python

import math
import numpy as np
import re
import os
import string

from collections import Counter
from matplotlib import pyplot as plt
from PIL import Image

from ipywidgets import interact
import ipywidgets as ipywidgets

from keras import layers
from keras import models
from keras import optimizers

from keras.utils import plot_model
from keras import backend


def hotShotMaker():
    blankOneShot = np.zeros((36), dtype=int)
    hotShot = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z']
    oneShot = dict()
    for i in range(0, 36):
        blankOneShot[i] = 1
        oneShot[hotShot[i]] = np.copy(blankOneShot)
        blankOneShot[i] = 0
    return oneShot


VALIDATION_SPLIT = 0.2
hotShotDict = hotShotMaker()

"""
Load in images from folder,
create collection of images(np arrays) and answer key(letter in file name)
"""

PATH = "~/Desktop/NNLab5/TestSet/"
relPATH = "TestSet/"

files = os.listdir(relPATH)

images = []

for f in files[:]:
    name = string.lower(f[6])
    images.append([np.array(Image.open(relPATH + f)), hotShotDict[name]])

size = len(images)
origX = np.empty((size,), dtype=object)
Y = np.empty((size,), dtype=object)

for y in range(0, size):
    temp1 = images[y]
    Y[y] = np.asarray(temp1[1])
for x in range(0, size):
    temp1 = images[x]
    origX[x] = np.asarray(temp1[0])
normX = origX/255
imageShape = normX[0].shape

#Shuffle Datasets
all_dataset1 = np.vstack((normX, Y))
all_dataset2 = np.hstack((all_dataset1, all_dataset1))
all_dataset = np.hstack((all_dataset2, all_dataset2))
np.take(all_dataset, np.random.permutation(all_dataset.shape[1]), axis=1,
        out=all_dataset)
datasetT = np.transpose(all_dataset)
print(all_dataset.shape)
print(datasetT.shape)
X_dataset = np.array([data[0] for data in datasetT[:]])
Y_dataset = np.array([data[1] for data in datasetT[:]])
print(X_dataset.shape)
print(Y_dataset.shape)
"""
Use to confirm oneshot is correct
def displayImage(index):
    plt.imshow(origX[index])
    temp = Y[index]
    caption = ("y = " + str(np.where(temp == 1)))
    plt.text(0.5, 0.5, caption, color='orange',
             fontsize=20, horizontalalignment='left', verticalalignment='top')
    plt.show()


displayImage(5)
"""
"""
Build NN from Keras
Feed Arrays into Keras to train
"""


def reset_weights(model):
    session = backend.get_session()
    for layer in model.layers: 
        if hasattr(layer, 'kernel_initializer'):
            layer.kernel.initializer.run(session=session)


conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                             input_shape=imageShape))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Flatten())
conv_model.add(layers.Dropout(0.5))
conv_model.add(layers.Dense(512, activation='relu'))
conv_model.add(layers.Dense(36, activation='softmax'))
conv_model.summary()

LEARNING_RATE = 1e-4
conv_model.compile(loss='categorical_crossentropy',
                   optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                   metrics=['acc'])

reset_weights(conv_model)

history_conv = conv_model.fit(X_dataset, Y_dataset,
                              validation_split=VALIDATION_SPLIT,
                              #validation_data=(X_dataset[0:10],
                                               #Y_dataset[0:10]),
                              epochs=13,
                              batch_size=16)

plt.plot(history_conv.history['acc'])
plt.plot(history_conv.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy (%)')
plt.xlabel('epoch')
plt.legend(['train accuracy', 'val accuracy'], loc='upper left')
plt.show()
