#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
import numpy.ma as m
import math
import sys
import time

from core_msgs_019.msg import Control
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16

 
class model_estimatier:
        def __init__ (self) :
                # Control constants
                self.MAX_speed = 5.5                 # [m/s]
                self.MIN_speed = 0                   # [m/s]
                self.MAX_steer = 2000/float(71)      # [degree]
                self.MIN_steer = -2000/float(71)     # [degree]
                self.MAX_brake = 150
                self.MIN_brake = 1

                # Control states
                self.NO_CONTROL = 0
                self.EMERGENCY_BRAKE = 1
                self.NORMAL_TRACKING = 2
                self.ESTIMATE_TRACKING = 3

                # Comunication constants
                self.pub_rate = 1.0            #[Hz] Temporary(Actually 20)

                # Platform constants
        
                # Control variable
                
                # Input manage
                

        # Define functions
        #TODO

        # Define write functions
        def write_task(self, data):
                #TODO

        def write_tracker_control(self, data):
                #TODO

        def wirte_cur_state(self, data):
                #TODO

        
        # Main control loop
        def main_control_loop(self):
                '''
                1) Take current task(/Task) and control from tracker(/Tracker_control)
                2) Take current platform state(/Current_state)
                3) Estimate proper platform speed, gear and brake control value
                4) Publish /Estimatier_control
                '''
                #TODO
                return self.control


main_ME = model_estimatier()

# Some other functions
#TODO


# Define callbac functions for Subscriber
def callbac_update_task(data):
        main_ME.write_task(data)

def callbac_update_tracker_control(data):
        main_ME.write_tracker_control(data)

def callbac_update_state(data):
        main_track.wirte_cur_state(data)

# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Model_estimatier', anonymous=True)

        #Subscribe
        rospy.Subscriber('Task', Int16, callbac_update_task)
        rospy.Subscriber('Tracker_control', Control, callbac_update_tracker_control)
        rospy.Subscriber('Current_state', Control, callbac_update_state)

        #Publish
        cont_pub = rospy.Publisher('/Estimatier_control', Control, queue_size=10)
        rate = rospy.Rate(main_ME.pub_rate)
        while not rospy.is_shutdown() :
                cont_pub.publish(main_ME.main_control_loop())
                rate.sleep()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case
