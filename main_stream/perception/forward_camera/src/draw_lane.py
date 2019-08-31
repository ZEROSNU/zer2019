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
from core_msgs.msg import ActiveNode
from core_msgs.msg import CenterLine
from matplotlib import pyplot as plt
import pdb
from time import sleep # for debug 


# Define Lane Coefficients Buffer
global left_coeff_buffer
left_coeff_buffer = []

global right_coeff_buffer
right_coeff_buffer = []

'''
------------------------------------------------------------------------
BASIC SETTINGS
------------------------------------------------------------------------
'''
# Maximum offset pixels from previous lane polynomial
LANE_ROI_OFFSET = (-50,50)

# Maximum offset pixels from previous lane polynomial (consider turning direction)
LANE_ROI_LEFT = (-40,20) # respectively, lower bound and upper bound
LANE_ROI_RIGHT = (-20,40)

# Minimum pixel num for draw line
CANDIDATE_THRES = 4000

# pixel Thresholds
CENTER_LINE_THRES = 20000
CROSSWALK_THRES = 120
STOP_LINE_THRES = 300

# Loose lane ROI offset when first search
ROBUST_SEARCH = True

# IMAGE & MAP SIZE (2019 Competition MAP SIZE : 200x200, 1px:3cm)
IMAGE_SIZE = 600
MAP_SIZE = 200

# LANE_WIDTH
LANE_WIDTH = 340

# NO LANE DETECTED(COUNT UP EVERY FRAME)
NO_LANE_COUNT = 0

# Debug Mode
Z_DEBUG = False

# Global parameters

# Gaussian smoothing
kernel_size = 3

# Canny Edge Detector
low_threshold = 50
high_threshold = 150

# Hough Transform
rho = 2 # distance resolution in pixels of the Hough grid
theta = 1 * np.pi/180 # angular resolution in radians of the Hough grid
threshold = 15     # minimum number of votes (intersections in Hough grid cell)
min_line_length = 10 #minimum number of pixels making up a line
max_line_gap = 10    # maximum gap in pixels between connectable line segments


class LaneNode:

    def __init__(self):
        self.node_name = 'forward_camera'
        self.pub_lane_map = rospy.Publisher('/lane_data', Image, queue_size=1)
        self.pub_center_line = rospy.Publisher('/is_center', CenterLine, queue_size=1)
        self.sub_active_nodes = rospy.Subscriber('/active_nodes', ActiveNode, self.signalResponse)
        self.sub_raw_img = rospy.Subscriber('/forward_camera/raw_img', Image, self.callback)
        self.active = True
        self.bridge = CvBridge()
        self.cl = CenterLine()
        self.seq = 0

    def signalResponse(self, data):
        if 'zero_monitor' in data.active_nodes:
            if self.node_name in data.active_nodes:
                self.active = True
            else:
                self.active = False
        else:
            rospy.signal_shutdown('no monitor')
    
    def callback(self, data):
        img_input = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #img_output = annotate_image_array(img_input)
        img_output,_,is_center_left,is_center_right = draw_line_with_color(img_input)
        img_output = img_output[:,:IMAGE_SIZE]
        
        resized = cv2.resize(img_output, (MAP_SIZE,MAP_SIZE), interpolation=cv2.INTER_AREA)

        matrix = cv2.getRotationMatrix2D((MAP_SIZE/2,MAP_SIZE/2),90,1)
        dst = cv2.warpAffine(resized,matrix,(MAP_SIZE,MAP_SIZE))
        map_img = np.zeros((MAP_SIZE,MAP_SIZE),np.int8)
        map_img = dst[:,:,0]
        map_img[map_img > 255] = 255
        send_img_raw_map = self.bridge.cv2_to_imgmsg(map_img, encoding='mono8')
        
        
        self.cl.is_center_left = is_center_left
        self.cl.is_center_right = is_center_right

        if self.active:
            self.cl.header.stamp = rospy.Time.now()
            self.cl.header.seq = self.seq
            self.seq += 1
            self.cl.header.frame_id = 'center_line'
        
        
            self.pub_lane_map.publish(send_img_raw_map)
            self.pub_center_line.publish(self.cl)

        if Z_DEBUG:
            cv2.imshow('color_image',img_input)
            cv2.waitKey(1)


def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def draw_line_with_color(image_in):
    global NO_LANE_COUNT, CROSSWALK, ROBUST_SEARCH
    
    lane_color = [100,0,0]
    stop_line_color = [200,0,0]
    
    left_color = lane_color
    right_color = lane_color

    is_center_left = False
    is_center_right = False

    line_img = np.zeros((image_in.shape[0], image_in.shape[1], 3), dtype=np.uint8)  # 3-channel BGR image
    

    image, stop_lines = filter_colors(image_in)
    
    points = np.where(image>0)
    x_vals = points[1]
    y_vals = points[0]

    if len(x_vals) == 0:
        print("No line detected")
        ROBUST_SEARCH = True
        NO_LANE_COUNT += 1
        if NO_LANE_COUNT < 5:
            if len(left_coeff_buffer) < 3:
                coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            else:
                coeff_left = left_coeff_buffer[2]
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
        else:
            coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
    elif CROSSWALK:
        print("ENTER CROSSWALK!")
        ROBUST_SEARCH = True
        NO_LANE_COUNT += 1
        '''
        if NO_LANE_COUNT < 5:
            if len(left_coeff_buffer) < 3:
                coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            else:
                coeff_left = left_coeff_buffer[2]
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
        else:
            coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
        '''
        coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
        coeff_right = np.array([10e-10,0,int(IMAGE_SIZE/2 + LANE_WIDTH/2)])
        
    else:
        if len(left_coeff_buffer) < 3:
            last_left_coeff = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            last_left_f = np.poly1d(last_left_coeff)
            last_right_coeff = np.array(last_left_coeff)
            last_right_coeff[2] += LANE_WIDTH
            last_right_f = np.poly1d(last_right_coeff)
            if ROBUST_SEARCH:
                left_x_candidates = x_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_OFFSET[1])]
                left_y_candidates = y_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_OFFSET[1])]
                
                right_x_candidates = x_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_OFFSET[1])]
                right_y_candidates = y_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_OFFSET[1])]
            else:
                left_x_candidates = x_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_LEFT[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_LEFT[1])]
                left_y_candidates = y_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_LEFT[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_LEFT[1])]
                
                right_x_candidates = x_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_RIGHT[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_RIGHT[1])]
                right_y_candidates = y_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_RIGHT[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_RIGHT[1])]
            if len(left_x_candidates) > CANDIDATE_THRES:
                
                NO_LANE_COUNT = 0

                coeff_left = np.polyfit(left_x_candidates,left_y_candidates,2)
                if len(right_x_candidates) > CANDIDATE_THRES:
                    coeff_right = np.polyfit(right_x_candidates,right_y_candidates,2)
                else:
                    coeff_right = np.array(coeff_left)
                    coeff_right[2] += LANE_WIDTH
            elif len(right_x_candidates) > CANDIDATE_THRES:
                
                NO_LANE_COUNT = 0

                coeff_right = np.polyfit(right_x_candidates,right_y_candidates,2)
                coeff_left = np.array(coeff_right)
                coeff_left[2] -= LANE_WIDTH
            else:
                print("NO LANE!!")
                coeff_left = last_left_coeff
                coeff_right = last_right_coeff

            left_coeff_buffer.append(coeff_left)
            right_coeff_buffer.append(coeff_right)
        
        else:
            last_left_coeff = left_coeff_buffer[2]
            last_right_coeff = right_coeff_buffer[2]
            last_left_f = np.poly1d(last_left_coeff)
            last_right_f = np.poly1d(last_right_coeff)

            if ROBUST_SEARCH:
                left_x_candidates = x_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_OFFSET[1])]
                left_y_candidates = y_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_OFFSET[1])]
                
                right_x_candidates = x_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_OFFSET[1])]
                right_y_candidates = y_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_OFFSET[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_OFFSET[1])]
            else:
                left_x_candidates = x_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_LEFT[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_LEFT[1])]
                left_y_candidates = y_vals[(y_vals - last_left_f(x_vals) > LANE_ROI_LEFT[0]) & (y_vals - last_left_f(x_vals) < LANE_ROI_LEFT[1])]
                
                right_x_candidates = x_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_RIGHT[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_RIGHT[1])]
                right_y_candidates = y_vals[(y_vals - last_right_f(x_vals) > LANE_ROI_RIGHT[0]) & (y_vals - last_right_f(x_vals) < LANE_ROI_RIGHT[1])]
            
            if len(left_x_candidates) > CANDIDATE_THRES:
                
                NO_LANE_COUNT = 0

                coeff_left = np.polyfit(left_x_candidates,left_y_candidates,2)
                
                left_prev_f1 = np.poly1d(left_coeff_buffer[1])
                left_prev_f2 = np.poly1d(left_coeff_buffer[2])
                left_current_f = np.poly1d(coeff_left)

                rsquared_prev1 = calculate_rsquared(x_vals, y_vals, left_prev_f1)
                rsquared_prev2 = calculate_rsquared(x_vals, y_vals, left_prev_f2)
                rsquared_current = calculate_rsquared(x_vals, y_vals, left_current_f)

                exp_sum = math.exp(rsquared_prev1) + math.exp(rsquared_prev2) + math.exp(rsquared_current)+10E-10
                weight_prev1 = math.exp(rsquared_prev1) / exp_sum
                weight_prev2 = math.exp(rsquared_prev2) / exp_sum
                weight_current = math.exp(rsquared_current) / exp_sum

                coeff_left = weight_prev1 * left_coeff_buffer[1] + weight_prev2 * left_coeff_buffer[2] + weight_current * coeff_left
                if len(right_x_candidates) > CANDIDATE_THRES:
                
                    coeff_right = np.polyfit(right_x_candidates,right_y_candidates,2)

                    right_prev_f1 = np.poly1d(right_coeff_buffer[1])
                    right_prev_f2 = np.poly1d(right_coeff_buffer[2])
                    right_current_f = np.poly1d(coeff_right)

                    rsquared_prev1 = calculate_rsquared(x_vals, y_vals, right_prev_f1)
                    rsquared_prev2 = calculate_rsquared(x_vals, y_vals, right_prev_f2)
                    rsquared_current = calculate_rsquared(x_vals, y_vals, right_current_f)

                    exp_sum = math.exp(rsquared_prev1) + math.exp(rsquared_prev2) + math.exp(rsquared_current)+10E-10
                    weight_prev1 = math.exp(rsquared_prev1) / exp_sum
                    weight_prev2 = math.exp(rsquared_prev2) / exp_sum
                    weight_current = math.exp(rsquared_current) / exp_sum

                    coeff_right = weight_prev1 * right_coeff_buffer[1] + weight_prev2 * right_coeff_buffer[2] + weight_current * coeff_right
                else:
                    coeff_right = np.array(coeff_left)
                    coeff_right[2] += LANE_WIDTH
            
            elif len(right_x_candidates) > CANDIDATE_THRES:
                
                NO_LANE_COUNT = 0

                coeff_right = np.polyfit(right_x_candidates,right_y_candidates,2)

                right_prev_f1 = np.poly1d(right_coeff_buffer[1])
                right_prev_f2 = np.poly1d(right_coeff_buffer[2])
                right_current_f = np.poly1d(coeff_right)

                rsquared_prev1 = calculate_rsquared(x_vals, y_vals, right_prev_f1)
                rsquared_prev2 = calculate_rsquared(x_vals, y_vals, right_prev_f2)
                rsquared_current = calculate_rsquared(x_vals, y_vals, right_current_f)

                exp_sum = math.exp(rsquared_prev1) + math.exp(rsquared_prev2) + math.exp(rsquared_current)+10E-10
                weight_prev1 = math.exp(rsquared_prev1) / exp_sum
                weight_prev2 = math.exp(rsquared_prev2) / exp_sum
                weight_current = math.exp(rsquared_current) / exp_sum

                coeff_right = weight_prev1 * right_coeff_buffer[1] + weight_prev2 * right_coeff_buffer[2] + weight_current * coeff_right

                coeff_left = np.array(coeff_right)
                coeff_left[2] -= LANE_WIDTH
            
            else:
                print("NO LANE!")
                NO_LANE_COUNT += 1
                if NO_LANE_COUNT < 5:
                    if len(left_coeff_buffer) < 3:
                        coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
                    else:
                        coeff_left = left_coeff_buffer[2]
                    coeff_right = np.array(coeff_left)
                    coeff_right[2] += LANE_WIDTH
                else:
                    coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
                    coeff_right = np.array(coeff_left)
                    coeff_right[2] += LANE_WIDTH
            if (len(left_x_candidates) > CENTER_LINE_THRES) or np.sum(np.sum(image[left_y_candidates,left_x_candidates]) > 200) > 1000:
                is_center_left = True
            if (len(right_x_candidates) > CENTER_LINE_THRES) or np.sum(np.sum(image[right_y_candidates,right_x_candidates])> 200) > 1000:
                is_center_right = True

            # Updating buffer            
            left_coeff_buffer[:2] = left_coeff_buffer[1:]
            left_coeff_buffer[2] = coeff_left
            right_coeff_buffer[:2] = right_coeff_buffer[1:]
            right_coeff_buffer[2] = coeff_right

            ROBUST_SEARCH = False
            
    # Draw the lines on image
    polypoints_left = np.zeros((IMAGE_SIZE,2))
    polypoints_right = np.zeros((IMAGE_SIZE,2))
    polypoints_stop = np.zeros((IMAGE_SIZE,2))

    t = np.arange(0,IMAGE_SIZE,1)
    f = np.poly1d(coeff_left)
    polypoints_left[:,0] = t
    polypoints_left[:,1] = f(t)
    cv2.polylines(line_img,np.int32([polypoints_left]),False,left_color,5)

    f = np.poly1d(coeff_right)
    polypoints_right[:,0] = t
    polypoints_right[:,1] = f(t)

    cv2.polylines(line_img,np.int32([polypoints_right]),False,right_color,5)

    if len(stop_lines) > 5:
        polypoints_stop[:,1] = t
        polypoints_stop[:,0] = stop_lines[0]
    
        cv2.polylines(line_img,np.int32([polypoints_stop]),False,stop_line_color,5)

    annotated_image = weighted_img(line_img, image_in)
    return line_img, annotated_image, is_center_left, is_center_right
        

def draw_lines(img, lines, color=[255, 0, 0], thickness=5):
    global NO_LANE_COUNT, CROSSWALK

    left_color = color
    right_color = color

    if lines is None or len(lines) == 0:
        print("No line detected")
        NO_LANE_COUNT += 1
        if NO_LANE_COUNT < 5:
            if len(left_coeff_buffer) < 3:
                coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            else:
                coeff_left = left_coeff_buffer[2]
        else:
            coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
    elif CROSSWALK:
        print("ENTER CROSSWALK!")
        if NO_LANE_COUNT < 5:
            if len(left_coeff_buffer) < 3:
                coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            else:
                coeff_left = left_coeff_buffer[2]
        else:
            coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
    else:
        NO_LANE_COUNT = 0

        left_line_candidates = []
        right_line_candidates = []

        stop_line_candidates = []

        # Extract candidate with slope threshold
        lower_left_thres = -1
        upper_left_thres = 0.1
        
        lower_right_thres = -0.1
        upper_right_thres = 1

        lower_stop_thres = 10
        upper_stop_thres = 1000
        
        for line in lines:
            x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]
            
            # Calculate slope
            if x2 - x1 == 0:  # Stop line case
                slope = 999  
            else:
                slope = (y2 - y1) / (x2 - x1)
                
            # Filter lines based on slope
            if slope > lower_left_thres and slope < upper_left_thres:
                left_line_candidates.append(line)
            if slope > lower_right_thres and slope < upper_right_thres:
                right_line_candidates.append(line)
            if slope > lower_stop_thres and slope < upper_stop_thres:
                stop_line_candidates.append(line)

        right_lines = []
        left_lines = []
        stop_lines = []
        
        # find left lines with ROI
        if len(left_coeff_buffer) < 3:
            left_center_y = int(IMAGE_SIZE/2 - LANE_WIDTH/2)
            for line in left_line_candidates:
                x1, y1, x2, y2 = line[0]
                if y1 > left_center_y - LANE_ROI_OFFSET and y1 < left_center_y + LANE_ROI_OFFSET and y2 > left_center_y - LANE_ROI_OFFSET and y2 < left_center_y + LANE_ROI_OFFSET:
                    left_lines.append(line)
        else:
            last_left_coeff = left_coeff_buffer[2]
            last_left_f = np.poly1d(last_left_coeff)

            for line in left_line_candidates:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - last_left_f(x1)) >= LANE_ROI_OFFSET or abs(y2 - last_left_f(x2)) >= LANE_ROI_OFFSET:
                    continue
                left_lines.append(line)
        
        # find right lines with ROI
        if len(right_coeff_buffer) < 3:
            right_center_y = int(IMAGE_SIZE/2 + LANE_WIDTH/2)
            for line in right_line_candidates:
                x1, y1, x2, y2 = line[0]
                if y1 > right_center_y - LANE_ROI_OFFSET and y1 < right_center_y + LANE_ROI_OFFSET and y2 > right_center_y - LANE_ROI_OFFSET and y2 < right_center_y + LANE_ROI_OFFSET:
                    right_lines.append(line)
        else:
            last_right_coeff = right_coeff_buffer[2]
            last_right_f = np.poly1d(last_right_coeff)

            for line in right_line_candidates:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - last_right_f(x1)) >= LANE_ROI_OFFSET or abs(y2 - last_right_f(x2)) >= LANE_ROI_OFFSET:
                    continue
                right_lines.append(line)

        # find stop lines with width of line
        for line in stop_line_candidates:
            x1, y1, x2, y2 = line[0]
            if abs(y1-y2) > 200:
                stop_lines.append(line)

        # Run linear regression to find best fit line for right and left lane lines
        # Right lane lines
        right_lines_x = []
        right_lines_y = []
        
        for line in right_lines:
            x1, y1, x2, y2 = line[0]
            if abs(x2 - x1) > 300:
                right_color = [0,0,255]
            fill_lines(right_lines_x, right_lines_y, x1, x2, y1, y2)
            
        # Left lane lines
        left_lines_x = []
        left_lines_y = []
        
        for line in left_lines:
            x1, y1, x2, y2 = line[0]
            if abs(x2 - x1) > 300:
                left_color = [0,0,255]
            fill_lines(left_lines_x, left_lines_y, x1, x2, y1, y2)
        
        if left_lines == [] and right_lines == []:
            NO_LANE_COUNT += 1
            print("No line!")
            coeff_left = np.array([10e-10,0,int(IMAGE_SIZE/2 - LANE_WIDTH/2)])
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
        elif left_lines and right_lines == []:
            coeff_left = np.polyfit(left_lines_x, left_lines_y, 2)
            coeff_right = np.array(coeff_left)
            coeff_right[2] += LANE_WIDTH
        elif right_lines and left_lines == []:
            coeff_right = np.polyfit(right_lines_x, right_lines_y, 2)
            coeff_left = np.array(coeff_right)
            coeff_left[2] -= LANE_WIDTH
        else:
            coeff_left = np.polyfit(left_lines_x, left_lines_y, 2)
            coeff_right = np.polyfit(right_lines_x, right_lines_y, 2)
        
        # Stop line
        stop_lines_x = []
        stop_lines_y = []

        if stop_lines:
            for line in stop_lines:
                x1, y1, x2, y2 = line[0]
                fill_lines(stop_lines_x,stop_lines_y,x1,x2,y1,y2)

        # Draw the lines on image
        polypoints_left = np.zeros((IMAGE_SIZE,2))
        polypoints_right = np.zeros((IMAGE_SIZE,2))
        polypoints_stop = np.zeros((IMAGE_SIZE,2))

        t = np.arange(0,IMAGE_SIZE,1)
        f = np.poly1d(coeff_left)
        polypoints_left[:,0] = t
        polypoints_left[:,1] = f(t)
        cv2.polylines(img,np.int32([polypoints_left]),False,left_color,5)

        f = np.poly1d(coeff_right)
        polypoints_right[:,0] = t
        polypoints_right[:,1] = f(t)

        cv2.polylines(img,np.int32([polypoints_right]),False,right_color,5)

        if stop_lines:
            cv2.polylines(img,np.int32([polypoints_stop]),False,[0,0,255],5)
        
        if len(left_coeff_buffer) < 3:
            left_coeff_buffer.append(coeff_left)
        else:
            left_coeff_buffer[:2] = left_coeff_buffer[1:]
            left_coeff_buffer[2] = coeff_left
        if len(right_coeff_buffer) < 3:
            right_coeff_buffer.append(coeff_right)
        else:
            right_coeff_buffer[:2] = right_coeff_buffer[1:]
            right_coeff_buffer[2] = coeff_right
        
        

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)  # 3-channel RGB image
    draw_lines(line_img, lines)
    return line_img


def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    
    `initial_img` should be the image before any processing.
    
    The result image is computed as follows:
    
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, a, img, b, c)

def filter_colors(image):
    global CROSSWALK
    """
    Filter the image to include only yellow and white pixels
    """
    # Filter white pixels
    white_threshold = 200
    lower_white = np.array([white_threshold, white_threshold, white_threshold])
    upper_white = np.array([255, 255, 255])
    white_mask = cv2.inRange(image, lower_white, upper_white)
    
    # Filter yellow and blue pixels
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([5,20,100],np.uint8)
    upper_yellow = np.array([15,255,255],np.uint8)
    lower_blue = np.array([95,150,150],np.uint8)
    upper_blue = np.array([110,255,255],np.uint8)

    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Eliminating small unnecessary dots (morphologyEx)
    #kernel = np.ones((3,3), np.uint8)
    #yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    #white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    #blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    
    hsv_mask = yellow_mask + blue_mask

    white_image = cv2.bitwise_and(image, image, mask=white_mask)
    hsv_image = cv2.bitwise_and(image, image, mask=hsv_mask)

    # Combine the two above images
    image2 = cv2.addWeighted(white_image, 0.1, hsv_image, 1., 0.)
    
    if len (left_coeff_buffer) == 3:
        left_y = int(left_coeff_buffer[2][2])
        right_y = int(right_coeff_buffer[2][2])
        
        white_vals = sum(white_mask[left_y:right_y,:]>0)
        
        crosswalk_lines = np.where((white_vals > CROSSWALK_THRES) & (white_vals < STOP_LINE_THRES))
        stop_lines = np.where(white_vals >= STOP_LINE_THRES)
    
    else:
        left_y = int(IMAGE_SIZE/2 - LANE_WIDTH/2)
        right_y = int(IMAGE_SIZE/2 + LANE_WIDTH/2)

        white_vals = sum(white_mask[left_y:right_y,:]>0)
        
        crosswalk_lines = np.where((white_vals > CROSSWALK_THRES) & (white_vals < STOP_LINE_THRES))
        stop_lines = np.where(white_vals >= STOP_LINE_THRES)#pdb.set_trace()
    CROSSWALK = False
    if len(stop_lines[0]) > 5:
        print("STOPLINE!")
    else:
        if len(crosswalk_lines[0]) > 5:
            CROSSWALK = True
        else:
            CROSSWALK = False
    
    cv2.imshow('color_filtered', image2)
    image2[:,stop_lines[0]] = [0,0,0]
    image2[:,crosswalk_lines[0]] = [0,0,0]
    cv2.imshow('after',image2)

    return image2, stop_lines[0]

def annotate_image_array(image_in):
    """ Given an image Numpy array, return the annotated image as a Numpy array """
    # Only keep white and yellow pixels in the image, all other pixels become black
    image = filter_colors(image_in)
    
    # Read in and grayscale the image
    gray = grayscale(image)

    # Apply Gaussian smoothing
    blur_gray = gaussian_blur(gray, kernel_size)
    
    # Apply Canny Edge Detector
    edges = canny(blur_gray, low_threshold, high_threshold)

    # Run Hough on edge detected image
    line_image = hough_lines(edges, rho, theta, threshold, min_line_length, max_line_gap)
    #line_image = draw_line_with_color(binary_warped)
    # Draw lane lines on the original image
    annotated_image = weighted_img(line_image, image_in)
    return annotated_image
    
def fill_lines(line_x, line_y, x1, x2, y1, y2):
    if x1 > x2:
        return fill_lines(line_x, line_y, x2,x1,y1,y2)
    if y1 == y2:
        for x in range(x1,x2+1):
            line_x.append(x)
            line_y.append(y1)
    else:
        # (y-y1) = a(x-x1)
        for x in range(x1,x2+1):
            a = (y2 - y1) / (x2 - x1)
            y = np.round(a*(x-x1) + y1)
            line_x.append(x)
            line_y.append(y)


def main():
    rospy.init_node('forward_camera', anonymous=True)
    ln = LaneNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
    '''
    path = "/home/kimsangmin/ZERO_VISION/bird/3/"
    file_num = 0
    postfix = ".jpg"
    total_file_num = 600

    while True:
        full_path = path + str(file_num) + postfix
        if file_num > total_file_num:
            break
        file_num += 1
        img_input = cv2.imread(full_path)
        #img_output = annotate_image_array(img_input)
        img_output, annotate_image, left, right = draw_line_with_color(img_input)
        img_output = img_output[:,:IMAGE_SIZE]
        map_img = cv2.resize(img_output, (MAP_SIZE,MAP_SIZE), interpolation=cv2.INTER_AREA)

        matrix = cv2.getRotationMatrix2D((MAP_SIZE/2,MAP_SIZE/2),90,1)
        dst = cv2.warpAffine(map_img,matrix,(MAP_SIZE,MAP_SIZE))
        #img_output = filter_colors(img_input)
        cv2.imshow("img", annotate_image)
        print((left,right))
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
    '''
    #img_input = cv2.imread('./bird/4/166.jpg') #curve example
    #img_input = cv2.imread('./bird/4/187.jpg') #challenging curve example
    
    #img_output = annotate_image_array(img_input)
    #cv2.imshow('line', img_output)
    #cv2.waitKey(100000) 