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


class Fake_serial_com:
        def __init__(self) :
                self.pub_rate = 30
                self.vehicle_state = Control()
                
                self.latest_ideal_control = Control()
                self.update_control = False

                self.vehicle_state.is_auto = True
                self.vehicle_state.estop = False
                self.vehicle_state.gear = 0
                self.vehicle_state.brake = 150
                self.vehicle_state.speed = 0.
                self.vehicle_state.steer = 0.

        def write_latest_ideal_control(self, data):
                self.latest_ideal_control = data
                self.update_control = True

        def Set_vehicle_state(self):
                if self.update_control == True:
                        self.vehicle_state = copy.deepcopy(self.latest_ideal_control)
                else:
                        pass
                self.update_velocity_level = False
                
                return self.vehicle_state


fake_serial_com = Fake_serial_com()

def callbac_update_ideal_control(data):
        fake_serial_com.write_latest_ideal_control(data)


# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Fake_serial_communicator', anonymous=True)

        #Subscribe
        rospy.Subscriber('ideal_control',Control, callbac_update_ideal_control)
        
        #Publish
        vehicle_state_pub = rospy.Publisher('/vehicle_state', Control, queue_size=10)
        rate = rospy.Rate(fake_serial_com.pub_rate)
        while not rospy.is_shutdown() :
                vehicle_state_pub.publish(fake_serial_com.Set_vehicle_state())
                rate.sleep()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case