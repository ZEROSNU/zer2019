#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from core_msgs.msg import VehicleState
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

IMAGE_SIZE = 600
MAP_SIZE = 200
VIEW_SIZE = 450

img_lane_map = np.zeros((200,200))

def callback_vehicle_state(vs_data):
    
    print(vs_data.is_auto)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)from cv_bridge import CvBridge, CvBridgeError

def callback_lane_map(lane_map):
    global img_lane_map
    img_lane_map = bridge.imgmsg_to_cv2(lane_map, 'mono8')



def callback_raw_map(raw_map):
    global img_raw_map
    img_raw_map = bridge.imgmsg_to_cv2(raw_map, 'bgr8')


if __name__ == '__main__':
    rospy.loginfo('Initiate monitoring node')
    rospy.init_node('monitor_node', anonymous=True)

    rospy.Subscriber("vehicle_state", VehicleState, callback_vehicle_state)
    rospy.Subscriber("/lane_map", Image, callback_lane_map)
    rospy.Subscriber("/raw_map", Image, callback_raw_map)

    bridge = CvBridge()

    rate = rospy.Rate(20) #20Hz

    img_lane_map = np.zeros((MAP_SIZE, MAP_SIZE),np.uint8)
    img_raw_map = np.zeros((IMAGE_SIZE,IMAGE_SIZE,3),np.uint8)

    while not rospy.is_shutdown():
        
        monitor_img = np.zeros((VIEW_SIZE+300,VIEW_SIZE*2+150,3),np.uint8)

        #Lane image proecssing
        lane_map_view = np.flip(img_lane_map,1)
        lane_map_view = cv2.resize(lane_map_view,(VIEW_SIZE,VIEW_SIZE))
        
        #Raw image processing
        raw_map_view = cv2.resize(img_raw_map,(VIEW_SIZE,VIEW_SIZE))

        monitor_img[50:50+VIEW_SIZE,50:50+VIEW_SIZE] = raw_map_view
        monitor_img[50:50+VIEW_SIZE,100+VIEW_SIZE:100+VIEW_SIZE*2,2] = lane_map_view
        
        cv2.imshow("Monitor", monitor_img)
        cv2.waitKey(50)
        rate.sleep()