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
from std_msgs.msg import Float32

'''
This node publish fake vehicle state and corresponding curvature.
Use to test tracker node.
'''

class Fake_serial_com:
        def __init__(self) :
                self.pub_rate = 30
                self.vehicle_state = Control()
                self.curvature = Float32()
                
                self.latest_new_curvature = Float32()
                self.update_curvature = False

                self.vehicle_state.is_auto = True
                self.vehicle_state.estop = False
                self.vehicle_state.gear = 1
                self.vehicle_state.brake = 1
                self.vehicle_state.speed = 4.
                self.vehicle_state.steer = 30.

        def write_latest_new_curvature(self, data):
                self.latest_new_curvature = data
                self.update_curvature = True

        '''
        def Set_vehicle_state(self):
                if self.update_control == True:
                        self.vehicle_state = copy.deepcopy(self.latest_ideal_control)
                else:
                        pass
                self.update_velocity_level = False
                
                return self.vehicle_state
        '''

        def  Set_curvature(self):
                if self.update_curvature == True:
                                self.curvature = copy.deepcopy(self.latest_new_curvature)
                else:
                        pass
                self.update_curvature = False
                        
                return self.curvature

fake_serial_com = Fake_serial_com()

def callbac_update_new_curvature(data):
        fake_serial_com.write_latest_new_curvature(data)


# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Fake_serial_communicator', anonymous=True)

        #Subscribe
        rospy.Subscriber('new_curvature',Float32, callbac_update_new_curvature)
        
        #Publish
        vehicle_state_pub = rospy.Publisher('/vehicle_state', Control, queue_size=10)
        curvature_pub = rospy.Publisher('/current_curvature', Float32, queue_size=10)
        rate = rospy.Rate(fake_serial_com.pub_rate)
        while not rospy.is_shutdown() :
                vehicle_state_pub.publish(fake_serial_com.vehicle_state)
                curvature_pub.publish(fake_serial_com.Set_curvature())
                rate.sleep()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case