import numpy as np
import cv2
import math

# Define values boundaries for color
lower_yellow = np.array([5,20,100],np.uint8)
upper_yellow = np.array([15,255,255],np.uint8)
    
lower_green = np.array([80-15,50,50],np.uint8)
upper_green = np.array([80+15,255,255],np.uint8)

lower_blue = np.array([95,150,150],np.uint8)
upper_blue = np.array([110,255,255],np.uint8)

lower_white = np.array([200,200,200],np.uint8)
upper_white = np.array([255,255,255],np.uint8)

#margin = np.ones(4) * 15

def get_hsv_color(event, x, y, flags, param):
    global hsv
    global img
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print("hsv:",hsv[y,x,:])
        print("rgb:",img[y,x,:])


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


if __name__ == "__main__":
    cam = cv2.VideoCapture(2)
    cv2.namedWindow('img')
    cv2.setMouseCallback('img',get_hsv_color)
    #img = cv2.imread('/home/kimsangmin/ZERO_VISION/bird/7/192.jpg')
    _, img = cam.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue = findColor(hsv, lower_blue, upper_blue)
    yellow = findColor(hsv, lower_yellow, upper_yellow)
    white = findColor(img, lower_white, upper_white)
    cv2.imshow('img', img)
    cv2.imshow('blue',blue)
    cv2.imshow('yellow',yellow)
    cv2.imshow('white', white)
    cv2.waitKey(100000)
