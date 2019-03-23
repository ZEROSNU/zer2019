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

# LANE_WIDTH
LANE_WIDTH = 280

# Debug Mode
Z_DEBUG = True


class LaneDrawer():
    def __init__(self):
        # NO LANE DETECTED(COUNT UP EVERY FRAME)
        self.NO_LANE_COUNT = 0

        # Define Lane Coefficients Buffer
        self.coeff_buffer = []

    def SetImage(self, img):
        self.img = img

    def SetMask(self, upper_mask, lower_mask):
        self.upper_mask = upper_mask
        self.lower_mask = lower_mask
    
    def PreProcess(self): # Image processing
        # Blurring, Converts BGR -> HSV color space
        self.img =  img = cv2.GaussianBlur(self.img, (5,5),0)
        self.hsv_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        # Masking color
        self.mask = findColor(self.hsv_img, self.lower_mask, self.upper_mask)
        # Eliminating small unnecessary dots (morphologyEx)
        kernel = np.ones((5,5), np.uint8)
        self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)
        # Get valid lane points
        lane_points = np.where(self.mask>0)
        self.x_vals = lane_points[1]
        self.y_vals = lane_points[0]

    def GetLaneData(self): # Getting lane data(2nd order polynomial fitting)
        if(np.size(self.x_vals) == 0):
            #IF NO LANE DETECTED
            print("NO LANE!")
            self.NO_LANE_COUNT = self.NO_LANE_COUNT + 1
            if(self.NO_LANE_COUNT < 5):
                if(len(self.coeff_buffer)<3):
                    self.coeff_left = np.array([10e-10,0,int(IMAGE_SIZE*0.5 + LANE_WIDTH*0.5 )]) 
                else:
                    self.coeff_left = self.coeff_buffer[2]
            else:
                self.coeff_left = np.array([10e-10,0,int(IMAGE_SIZE*0.5 + LANE_WIDTH*0.5 )])
        else:
            #IF LANE DETECTED
            self.NO_LANE_COUNT = 0

            if(len(self.coeff_buffer)<3):
                # Previous coefficient data is not sufficient (less than 3)
                self.coeff_left = np.polyfit(self.x_vals, self.y_vals,2)
                self.coeff_buffer.append(self.coeff_left)
                
            else:
                # Previous coefficient data is sufficient (more than 3)

                # Calculate coefficients using ROI ###START
                last_coeff = self.coeff_buffer[2]
                last_f = np.poly1d(last_coeff)

                # Target points inner ROI (Comparing with previous lane data)
                y_vals_roi = self.y_vals[abs(self.y_vals - last_f(self.x_vals))<LANE_ROI_OFFSET]
                x_vals_roi = self.x_vals[abs(self.y_vals - last_f(self.x_vals))<LANE_ROI_OFFSET]
                

                coeff = np.polyfit(x_vals_roi, y_vals_roi,2)
                self.x_vals = x_vals_roi
                self.y_vals = y_vals_roi

                # Using buffers for filtering
                # 1. Calculate rsquared for last 3 coefficients each
                # 2. Calculate weights of each coefficients using rsquared & softmax function
                prev_f1 = np.poly1d(self.coeff_buffer[1])
                prev_f2 = np.poly1d(self.coeff_buffer[2])
                current_f = np.poly1d(coeff)

                rsquared_prev1 = calculate_rsquared(self.x_vals, self.y_vals, prev_f1)
                rsquared_prev2 = calculate_rsquared(self.x_vals, self.y_vals, prev_f2)
                rsquared_current = calculate_rsquared(self.x_vals, self.y_vals, current_f)

                exp_sum = math.exp(rsquared_prev1) + math.exp(rsquared_prev2) + math.exp(rsquared_current)+10E-10
                weight_prev1 = math.exp(rsquared_prev1) / exp_sum
                weight_prev2 = math.exp(rsquared_prev2) / exp_sum
                weight_current = math.exp(rsquared_current) / exp_sum

                self.coeff_left = weight_prev1 * self.coeff_buffer[1] + weight_prev2 * self.coeff_buffer[2] + weight_current * coeff

                # Updating buffer
                self.coeff_buffer[0:-1] = self.coeff_buffer[1:3]
                self.coeff_buffer[2] = self.coeff_left

    def DrawLane(self): # Draw Lane
        coeff_left = self.coeff_left
        
        # Right Lane
        t = np.arange(0,IMAGE_SIZE,1)
        f = np.poly1d(coeff_left)

        polypoints = np.zeros((IMAGE_SIZE,2))
        polypoints_left = np.zeros((IMAGE_SIZE,2))
        polypoints_left_ = np.zeros((IMAGE_SIZE,2))
        polypoints[:,0] = t
        polypoints[:,1] = f(t)

        # Left Lane
        polypoints_left_[:,0] = t
        polypoints_left_[:,1] = f(t) - LANE_WIDTH

        coeff_right = np.copy(coeff_left)
        coeff_right[2] = coeff_right[2] - LANE_WIDTH

        '''
        ------------------------------------------------------------------------
        CREATE NEW MASK FOR PUBLISH
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

        self.masked_img = np.zeros((IMAGE_SIZE,IMAGE_SIZE), dtype='uint8')
        self.masked_img[mask_left<y_vals] = 255
        self.masked_img[mask_right>y_vals] = 255

        #Draw lines on lane
        cv2.polylines(self.img, np.int32([polypoints]), False, (255,0,0),2)
        cv2.polylines(self.img, np.int32([polypoints_left_]), False, (0,255,0),2)
    
    def GetLaneMapMessage(self, bridge):
        #resize, rotate, and flip for output
        M_lane_map = cv2.getRotationMatrix2D((int(MAP_SIZE/2),int(MAP_SIZE/2)),180,1)

        masked_img = cv2.resize(self.masked_img,(MAP_SIZE,MAP_SIZE))
        masked_img = masked_img.T

        masked_img = cv2.warpAffine(masked_img,M_lane_map,(MAP_SIZE,MAP_SIZE))
        masked_img = cv2.flip(masked_img,1)
        send_img_lane_map = bridge.cv2_to_imgmsg(masked_img, "mono8")
        
        self.masked_img = masked_img

        return send_img_lane_map

    def GetRawMapMessage(self,bridge):

        #resize, rotate, and flip for output
        M_raw_map = cv2.getRotationMatrix2D((IMAGE_SIZE/2,IMAGE_SIZE/2),90,1)

        send_img =  cv2.warpAffine(self.img,M_raw_map,(IMAGE_SIZE,IMAGE_SIZE))
        send_img_raw_map = bridge.cv2_to_imgmsg(send_img, "bgr8")
        
        return send_img_raw_map

def callback(data):

    img_input = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    Drawer.SetImage(img_input)
    Drawer.SetMask(upper_yellow, lower_yellow) # yellow mask
    #Drawer.SetMask(upper_green, lower_green)  # green mask
    Drawer.PreProcess()
    Drawer.GetLaneData()
    Drawer.DrawLane()

    pub_lane_map.publish(Drawer.GetLaneMapMessage(bridge))
    pub_raw_map.publish(Drawer.GetRawMapMessage(bridge))

    if Z_DEBUG:
        cv2.imshow('image',Drawer.mask)
        cv2.imshow('color_image',Drawer.img)
        cv2.imshow('send_image',Drawer.masked_img)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.loginfo('Initiate draw_lane node')
    rospy.init_node('draw_lane', anonymous=True)

    

    bridge = CvBridge()
    pub_lane_map = rospy.Publisher('/lane_map', Image, queue_size=1)
    pub_raw_map = rospy.Publisher('/raw_map', Image, queue_size=1)

    rospy.Subscriber("raw_img", Image, callback)

    Drawer = LaneDrawer()

    rospy.spin()
