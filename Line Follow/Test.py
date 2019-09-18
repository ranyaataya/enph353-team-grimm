import cv2
import numpy as np
import matplotlib.pyplot as plt

# Tutorial used
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html

# Load in video from file
capture = cv2.VideoCapture('raw_video_feed.mp4')

# Prepare to write output
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

while (capture.isOpened()):
    ret, frame = capture.read()

    # set to gray

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dimensions = gray.shape
    grayInverse = ~gray
    bw = cv2.threshold(grayInverse, 147, 255, cv2.THRESH_BINARY)[1]

    h, w = bw.shape[0:2]

   # print(bw.shape)
   # print("Width: {} Height {}".format(w, h))

    left_x = -34
    right_x = -34
    for x in range(w):
        if (bw[h - 5, x] > 0):
            left_x = x
            break
    # print("Left x: {}".format(left_x))

    for x in range(w):
        if (bw[h - 5, w-x-1] > 0):
            right_x = w-x
            break
    # print("Right x: {}".format(right_x - 1))

    circleCenterX = int((right_x+left_x)/2)

    # cv2.circle(frame, (right_x, h - 25), 15, (0, 0, 255), -1)
    cv2.circle(frame, (circleCenterX, h - 25), 15, (0, 255, 0), -1)
    # cv2.circle(frame, (left_x, h - 25), 15, (255, 0, 0), -1)
    # print('success')
    if ret is True:
        # grayInverse[0, 0] = 255
        # output fram
        out.write(gray)
        # show frame
        cv2.imshow('outline', frame)
        if cv2.waitKey(31) & 0xFF == ord('q'):
            break
    else:
        break
    # plt.imshow(frame)
    # plt.show()

# Release everything if job is finished
capture.release()
out.release()
# cv2.destroyAllWindows()
