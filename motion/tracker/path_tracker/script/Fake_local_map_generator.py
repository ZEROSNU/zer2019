#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
import numpy.ma as m
import math
import sys
import time

from std_msgs.msg import Int8
from std_msgs.msg import Float32

class Fake_map_gen:
        def __init__(self) :
                self.pub_rate = 30

                self.MAX_speed = 5.5
                self.MIN_speed = 0.

                self.gear_level = Int8()
                self.velocity_level = Float32()

        def Set_gear_level(self):
                self.gear_level = 1 #forward mode

                return self.gear_level

        def Set_velocity_level(self):
                self.velocity_level = 4 #m/s

                return self.velocity_level

fake_map_gen = Fake_map_gen()

# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Fake_local_map_gen', anonymous=True)
        
        #Publish
        velocity_pub = rospy.Publisher('/velocity_level', Float32, queue_size=10)
        gear_pub = rospy.Publisher('/gear_level', Int8, queue_size=10)
        rate = rospy.Rate(fake_map_gen.pub_rate)
        while not rospy.is_shutdown() :
                velocity_pub.publish(fake_map_gen.Set_velocity_level())
                gear_pub.publish(fake_map_gen.Set_gear_level())
                rate.sleep()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case