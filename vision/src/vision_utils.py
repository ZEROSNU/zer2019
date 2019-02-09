import numpy as np
import cv2
import math

# Define values boundaries for color
lower_yellow = np.array([15,40,150],np.uint8)
upper_yellow = np.array([40,255,255],np.uint8)


def findColor(hsv_image, lower, upper):
    mask = cv2.inRange(hsv_image, lower, upper)
    return mask

# Calculates R-squared value of data
def calculate_rsquared(x, y, f):
    yhat = f(x)
    if (len(y) != 0):
        ybar = np.sum(y)/len(y)
        error = y - yhat

        rss = np.sum(error**2)
        tss = np.sum((y-ybar)**2)
        rsquared = 1-(rss/tss)
    else:
        rsquared = 0
    return rsquared
