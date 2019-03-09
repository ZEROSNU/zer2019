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

'''
------------------------------------------------------------------------
SET DEFAULT VALUES
------------------------------------------------------------------------
'''
img_obstacle_map = np.zeros((200,200))
is_auto = False
estop = False
gear = 0   # 0: drive gear 1: neutral 2: rear
brake = 0  # 0~200
speed = 0  # m/s
steer = 0 # degree
encoder = 0
alive = 0

def callback_vehicle_state(vs_data):
    global is_auto, estop, gear, brake, speed, steer, encoder, alive
    is_auto = vs_data.is_auto
    estop = vs_data.estop
    gear = vs_data.gear
    brake = vs_data.brake
    speed = vs_data.speed
    steer = vs_data.steer
    encoder = vs_data.encoder
    alive = vs_data.alive
    
def callback_obstacle_map(obstacle_map):
    global img_obstacle_map
    img_obstacle_map = bridge.imgmsg_to_cv2(obstacle_map, 'mono8')



def callback_raw_map(raw_map):
    global img_raw_map
    img_raw_map = bridge.imgmsg_to_cv2(raw_map, 'bgr8')


if __name__ == '__main__':
    rospy.loginfo('Initiate monitoring node')
    rospy.init_node('monitor_node', anonymous=True)

    rospy.Subscriber("vehicle_state", VehicleState, callback_vehicle_state)
    rospy.Subscriber("/obstacle_map", Image, callback_obstacle_map)
    rospy.Subscriber("/raw_map", Image, callback_raw_map)

    bridge = CvBridge()

    rate = rospy.Rate(20) #20Hz

    img_obstacle_map = np.zeros((MAP_SIZE, MAP_SIZE),np.uint8)
    img_raw_map = np.zeros((IMAGE_SIZE,IMAGE_SIZE,3),np.uint8)

    while not rospy.is_shutdown():
        
        monitor_img = np.zeros((VIEW_SIZE+300,VIEW_SIZE*2+150,3),np.uint8)

        #Lane image proecssing
        #obstacle_map_view = np.flip(img_obstacle_map,1)
        obstacle_map_view = img_obstacle_map
        obstacle_map_view = cv2.resize(obstacle_map_view,(VIEW_SIZE,VIEW_SIZE))
        
        #Raw image processing
        raw_map_view = cv2.resize(img_raw_map,(VIEW_SIZE,VIEW_SIZE))

        monitor_img[50:50+VIEW_SIZE,50:50+VIEW_SIZE] = raw_map_view
        monitor_img[50:50+VIEW_SIZE,100+VIEW_SIZE:100+VIEW_SIZE*2,2] = obstacle_map_view
        
        #Generate Vehicle State text - is_auto
        output = "Mode : "
        if(is_auto):
            output = output + "Auto"
        else:
            output = output + "Manual"
        cv2.putText(monitor_img, output, (50,100+VIEW_SIZE), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        #Generate Vehicle State text - gear
        output = "Gear : "
        if(gear==0):
            output = output + "Forward"
        elif(gear==1):
            output = output + "Neutral"
        elif(gear==2):
            output = output + "Baccward"
        cv2.putText(monitor_img, output, (200,100+VIEW_SIZE), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        
        #Generate Vehicle State text - brake
        output = "Brake : " + str(brake)
        cv2.putText(monitor_img, output, (400,100+VIEW_SIZE), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        
        #Generate Vehicle State text - speed
        speed = round(speed, 1)
        output = "Speed : " + str(speed) + "m/s"
        cv2.putText(monitor_img, output, (550,100+VIEW_SIZE), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        
        #Generate Vehicle State text - steer
        steer = int(steer)
        output = "Steer : " + '{:3d}'.format(steer)
        cv2.putText(monitor_img, output, (750,100+VIEW_SIZE), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("Monitor", monitor_img)
        cv2.waitKey(50)
        rate.sleep()