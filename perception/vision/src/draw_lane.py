#!/usr/bin/env python
import sys
import rospy
import cv2
import math
import numpy as numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_utils import *

bridge = CvBridge()

# Define Lane Coefficients Buffer
global coeff_buffer
coeff_buffer = []

'''
------------------------------------------------------------------------
BASIC SETTINGS
------------------------------------------------------------------------
'''
# Maximum offset pixels from previous lane polynomial
LANE_ROI_OFFSET = 100

# IMAGE & MAP SIZE (2019 Competition MAP SIZE : 200x200, 1px:3cm)
IMAGE_SIZE = 600
MAP_SIZE = 200

# Debug Mode
Z_DEBUG = False

def callback(data):
    img_input = bridge.imgmsg_to_cv2(data, 'bgr8')

    '''
    ------------------------------------------------------------------------
    IMAGE PROCESSING
    ------------------------------------------------------------------------
    '''
    # Blurring, Converts BGR -> HSV color space
    img = cv2.GaussianBlur(img_input, (5,5),0)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Masking color
    yellow_mask = findColor(hsv_img, lower_yellow, upper_yellow)
    green_mask = findColor(hsv_img, lower_green, upper_green)

    # Eliminating small unnecessary dots (morphologyEx)
    kernel = np.ones((5,5), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    
    #mask = yellow_mask
    mask = green_mask
    points_mask = np.where(mask>0)

    x_vals = points_mask[1]
    y_vals = points_mask[0]

    '''
    ------------------------------------------------------------------------
    GETTING LANE DATA (LINE FITTING - 2ND ORDER POLYNOMIAL COEFFICIENTS)
    ------------------------------------------------------------------------
    '''

    if(len(coeff_buffer)<3):
        # Previous coefficient data is not sufficient (less than 3)
        coeff = np.polyfit(x_vals, y_vals,2)
        coeff_buffer.append(coeff)
        coeff_left = coeff

    else:
        # Previous coefficient data is sufficient (more than 3)

        # Calculate coefficients using ROI ###START
        last_coeff = coeff_buffer[2]
        last_f = np.poly1d(last_coeff)

        # Target points inner ROI (Comparing with previous lane data)
        y_vals_roi = y_vals[abs(y_vals - last_f(x_vals))<LANE_ROI_OFFSET]
        x_vals_roi = x_vals[abs(y_vals - last_f(x_vals))<LANE_ROI_OFFSET]

        coeff = np.polyfit(x_vals_roi, y_vals_roi,2)

        x_vals = x_vals_roi
        y_vals = y_vals_roi

        # Using buffers for filtering
        # 1. Calculate rsquared for last 3 coefficients each
        # 2. Calculate weights of each coefficients using rsquared & softmax function
        prev_f1 = np.poly1d(coeff_buffer[1])
        prev_f2 = np.poly1d(coeff_buffer[2])
        current_f = np.poly1d(coeff)

        rsquared_prev1 = calculate_rsquared(x_vals, y_vals, prev_f1)
        rsquared_prev2 = calculate_rsquared(x_vals, y_vals, prev_f2)
        rsquared_current = calculate_rsquared(x_vals, y_vals, current_f)

        exp_sum = math.exp(rsquared_prev1) + math.exp(rsquared_prev2) + math.exp(rsquared_current)+10E-10
        weight_prev1 = math.exp(rsquared_prev1) / exp_sum
        weight_prev2 = math.exp(rsquared_prev2) / exp_sum
        weight_current = math.exp(rsquared_current) / exp_sum

        coeff_left = weight_prev1 * coeff_buffer[1] + weight_prev2 * coeff_buffer[2] + weight_current * coeff

        # Updating buffer
        coeff_buffer[0:-1] = coeff_buffer[1:3]
        coeff_buffer[2] = coeff_left

    t = np.arange(0,IMAGE_SIZE,1)
    f = np.poly1d(coeff_left)

    polypoints = np.zeros((IMAGE_SIZE,2))
    polypoints_left = np.zeros((IMAGE_SIZE,2))
    polypoints_left_ = np.zeros((IMAGE_SIZE,2))
    polypoints[:,0] = t
    polypoints[:,1] = f(t)

    '''
    ------------------------------------------------------------------------
    DRAW RIGHT LANE
    ------------------------------------------------------------------------
    '''

    LANE_WIDTH = 280


    #just shifting works well(?)
    '''
    slopes = 2 * coeff_left[0] * t + coeff_left[1]
    theta = np.arctan2((slopes), 1.0)
    polypoints_left[:,0] = t + LANE_WIDTH * np.cos(theta-np.pi/2)
    polypoints_left[:,1] = f(t) + LANE_WIDTH * np.sin(theta-np.pi/2)
    '''

    polypoints_left_[:,0] = t
    polypoints_left_[:,1] = f(t) - LANE_WIDTH

    coeff_right = np.copy(coeff_left)
    coeff_right[2] = coeff_right[2] - LANE_WIDTH
    '''
    ------------------------------------------------------------------------ CREATE NEW MASK FOR PUBLISH
    OUSIDE LANE = OCCUPIED, WHITE, 1
    INSIDE LANE = UNOCCUPIED, BLACK, 0
    ------------------------------------------------------------------------
    '''
    mask_left = np.arange(0, IMAGE_SIZE, 1)
    mask_right = np.arange(0, IMAGE_SIZE, 1)

    mask_left = coeff_left[0] * mask_left * mask_left + coeff_left[1] * mask_left + coeff_left[2]
    mask_left = np.zeros((IMAGE_SIZE,IMAGE_SIZE)) + mask_left

    mask_right = coeff_right[0] * mask_right * mask_right + coeff_right[1] * mask_right + coeff_right[2]
    mask_right = np.zeros((IMAGE_SIZE,IMAGE_SIZE)) + mask_right

    y_vals = np.arange(0,IMAGE_SIZE,1)
    y_vals = np.broadcast_to(y_vals, (IMAGE_SIZE,IMAGE_SIZE)).T

    masked_img = np.zeros((IMAGE_SIZE,IMAGE_SIZE), dtype='uint8')
    masked_img[mask_left<y_vals] = 255
    masked_img[mask_right>y_vals] = 255

    #Draw lines on lane
    cv2.polylines(img, np.int32([polypoints]), False, (255,0,0),2)
    #cv2.polylines(img, np.int32([polypoints_left]), False, (255,0,0),2)
    cv2.polylines(img, np.int32([polypoints_left_]), False, (0,255,0),2)


    #resize, rotate, and flip for output
    M_lane_map = cv2.getRotationMatrix2D((int(MAP_SIZE/2),int(MAP_SIZE/2)),180,1)
    M_raw_map = cv2.getRotationMatrix2D((IMAGE_SIZE/2,IMAGE_SIZE/2),90,1)

    masked_img = cv2.resize(masked_img,(MAP_SIZE,MAP_SIZE))
    masked_img = masked_img.T

    masked_img = cv2.warpAffine(masked_img,M_lane_map,(MAP_SIZE,MAP_SIZE))
    send_img_lane_map = bridge.cv2_to_imgmsg(masked_img, "mono8")
    pub_lane_map.publish(send_img_lane_map)

    send_img =  cv2.warpAffine(img,M_raw_map,(IMAGE_SIZE,IMAGE_SIZE))
    send_img_raw_map = bridge.cv2_to_imgmsg(send_img, "bgr8")
    pub_raw_map.publish(send_img_raw_map)

    if Z_DEBUG:
        cv2.imshow('image',green_mask)
        cv2.imshow('color_image',img)
        cv2.imshow('send_image',masked_img)
        cv2.waitKey(50)


if __name__ == '__main__':
    rospy.loginfo('Initiate draw_lane node')

    rospy.init_node('draw_lane', anonymous=True)

    bridge = CvBridge()
    pub_lane_map = rospy.Publisher('/lane_map', Image, queue_size=1)
    pub_raw_map = rospy.Publisher('/raw_map', Image, queue_size=1)

    rospy.Subscriber("raw_img", Image, callback)

    rospy.spin()
