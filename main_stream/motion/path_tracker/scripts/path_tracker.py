#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
import numpy.ma as m
import math
import sys
import time
import tf_conversions

from core_msgs.msg import Control
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from core_msgs.msg import VelocityLevel
from core_msgs.msg import ActiveNode
from core_msgs.msg import Curvature
from core_msgs.msg import VehicleState
from core_msgs.msg import MotionState

 
class tracker:
        def __init__ (self) :
                # Control states
                self.NO_CONTROL = 0
                self.EMERGENCY_BRAKE = 1
                self.NORMAL_TRACKING = 2
                self.ESTIMATE_TRACKING = 3
        
                # Control variable
                self.look_ahead_distance = 30
                self.look_ahead_point= PoseStamped()

                # Input manage
                self.vehicle_state = VehicleState()

                self.latest_generated_path = Path()
                self.current_path = Path()
                self.update_path = False
                self.first_path = True

                self.latest_velocity_level = VelocityLevel()
                self.velocity_level = VelocityLevel()
                self.update_velocity_level = False

                self.latest_motion_state = MotionState()
                self.motion_state = MotionState()
                self.update_motion_state = False
                self.motion_state_buff = []
                self.motion_state_buff_size = 20

                self.curvature = Curvature()
                self.curvature.gear = 1
                self.curvature_write = 0
                self.curvature_count = 0
                self.curvature_buff = []
                self.curvature_time_buff = []
                self.curvature_buff_size = 200

                self.temp_control_mode = 0

                # Variable for parking
                self.is_PARKING = False
                self.init_parking_path = Path()

        # Define functions
        def Path_update(self):
                try:
                        self.current_path = copy.deepcopy(self.latest_generated_path)
                        self.first_path = True
                        return self.NORMAL_TRACKING
                except:
                        print("Path_update failed.")
                        return self.EMERGENCY_BRAKE

        def Path_parsing(self):
                try:
                        diff = []
                       for pose in self.current_path.poses:
                               pose.pose.position.x
                       return  
                except:
                        print("Path parsing failed.")

        def Path_estimate(self, initialize_time):       # TODO: Consider motion_state_buff[-1], it was parking or not?
                try:
                        passed_time = initialize_time - self.curvature_time_buff[-1]
                        
                        # Estimate current vehicle state (position and orientation)
                        theta = self.vehicle_state.speed * self.curvature.curvature * passed_time
                        shift_y = math.sin(theta)/self.curvature.curvature
                        if self.vehicle_state.steer >= 0.:
                                shift_x = (1. - math.cos(theta))/self.curvature.curvature
                                rad_angle = math.pi/2. - theta
                        else:
                                shitf_x = (math.cos(theta) - 1.)/self.curvature.curvature
                                rad_angle = math.pi/2. + theta

                        #calculate quaternion rotate
                        c = math.cos(rad_angle/2.)
                        s = math.sin(rad_angle/2.)
                        q = np.array([[c, 0., 0., s]])
                        q_inv = np.array([[c],
                                         [0.],
                                         [0.],
                                         [s]])

                        # Update map
                        for i in self.current_path.poses:
                                #update position
                                i.pose.position.x = i.pose.position.x - shift_x
                                i.pose.position.y = i.pose.position.y - shift_y
                                #update orientation   (Check is needed !!!!!)
 
                                v = np.array([[i.pose.position.w],
                                             [i.pose.position.x],
                                             [i.pose.position.y],
                                             [i.pose.position.z]])
                                qv = np.dot(q,v)
                                v = qv * q_inv
                                i.pose.orientation.x = v[0][1]
                                i.pose.orientation.y = v[0][2]
                                i.pose.orientation.z = v[0][3]
                                i.pose.orientation.w = v[0][0]

                        return self.ESTIMATE_TRACKING
                        
                except:
                        print("Path_estimate failed.")
                        return self.EMERGENCY_BRAKE

         
        def Set_look_ahead_point(self, temp_control_mode):
                #try:
                        # TODO
                        list_len = len(self.current_path.poses)
                        if list_len <= 0:
                                return self.EMERGENCY_BRAKE
                        if list_len > self.look_ahead_distance:
                                self.look_ahead_point = copy.deepcopy(self.current_path.poses[self.look_ahead_distance])
                        else:
                                self.look_ahead_point = copy.deepcopy(self.current_path.poses[-1])
                                
                        return temp_control_mode
                #except:
                #        print("Set_look_ahead_point failed.")
                #        return self.EMERGENCY_BRAKE

        def Deicide_curvature(self, temp_control_mode): # TODO: Consider current motion state -> Parking or not?
                try:
                        if (self.look_ahead_point.pose.position.x == 0) and (self.look_ahead_point.pose.position.y == 0):
                                self.curvature.curvature = 0.
                        else:   
                                self.curvature.curvature = 2*(self.look_ahead_point.pose.position.x)/((self.look_ahead_point.pose.position.x**2 + self.look_ahead_point.pose.position.y**2))
                        return temp_control_mode

                except:
                        print("Deicide_steering_angle failed.")
                        return self.EMERGENCY_BRAKE
                       
                        
        def Set_look_ahead_distance(self):
                #TODO adaptive look_ahead_distance
                self.look_ahead_distance = 20
                
        # Define write functions
        def write_latest_path(self, data):
                self.latest_generated_path = data
                self.update_path = True

        def wirte_vehicle_state(self, data):
                self.vehicle_state = data

        def write_velocity_level(self, data):
                self.latest_velocity_level = data
                self.update_velocity_level = True

        def write_motion_state(self, data):
                self.latest_motion_state = data
                self.update_motion_state = True

        # Main control loop
        def main_control_loop(self):
                '''
                1. Initialization
                '''
                # Initialize curvature
                if self.first_path == True:
                        if len(self.curvature_buff) <= 0:
                                self.curvature = Curvature()
                        else:
                                self.curvature = copy.deepcopy(self.curvature_buff[-1])
                else:
                        self.curvature = Curvature()
                        self.curvature.curvature = 1.

                # Update velocity level
                if self.update_velocity_level == True:
                        self.velocity_level = copy.deepcopy(self.latest_velocity_level)
                else:
                        pass
                self.update_velocity_level = False
 
                # Initialize temp_control_mode
                temp_control_mode = self.NO_CONTROL

                # Initialize motion state
                if self.update_motion_state == True:
                        self.motion_state = copy.deepcopy(self.latest_motion_state)
                else:
                        pass
                self.update_motion_state = False

                # Decide is parking state
                if self.motion_state == 'PARKING':
                        self.is_PARKING = True
                else:
                        self.is_PARKING = False

                # Initialize time
                if self.first_path == True:
                    initialize_time = rospy.get_rostime().to_sec()
                    #initialize_time = self.curvature_time_buff[-1]
                else:
                    initialize_time = rospy.get_rostime().to_sec()

                '''
                2. Path update
                '''
                if self.update_path == True:
                        temp_control_mode = self.Path_update()
                        if self.is_PARKING == True:
                                temp_control_mode = self.Path_parsing()
                else:
                        if self.first_path == True:
                                temp_control_mode = self.Path_estimate(initialize_time)
                        else:
                                temp_control_mode = self.EMERGENCY_BRAKE
                self.update_path = False

                '''
                3. Deciding new curvature
                '''
                if temp_control_mode == self.EMERGENCY_BRAKE:
                        pass
                else:
                        temp_control_mode = self.Set_look_ahead_point(temp_control_mode)
                        temp_control_mode = self.Deicide_curvature(temp_control_mode)

                '''
                4. Save and return new curvature
                '''
                if self.first_path == True:
                    if len(self.curvature_buff) >= self.curvature_buff_size:
                            self.curvature_buff[0:-1]=self.curvature_buff[1:]
                            self.curvature_buff[-1] = self.curvature
                            self.curvature_time_buff[0:-1] = self.curvature_time_buff[1:]
                            self.curvature_time_buff[-1] = rospy.get_rostime().to_sec()
                            self.motion_state_buff[0:-1] = self.motion_state_buff[1:]
                            self.motion_state_buff[-1] = self.motion_state
                    else:
                            self.curvature_buff.append(self.curvature)
                            self.curvature_time_buff = np.append(self.curvature_time_buff, rospy.get_time())
                            self.motion_state_buff.append(self.motion_state)
                    self.curvature_count += 1
                
                print("One main tracking cycle is compeleted.")
                return self.curvature


main_track = tracker()

# Some operation functions
def rotation_transform(x, y, theta):
        new_x = math.cos(theta) * x - math.sin(theta) * y
        new_y = math.sin(theta) * x + math.cos(theta) * y
        return new_x, new_y

def distance_two_point(A, B):
        return math.sqrt((A.x - B.x)**2 + (A.y - B.y)**2)


# Define callbac functions for Subscriber
def callbac_update_path(data):
        main_track.write_latest_path(data)
        return 0

def callbac_update_vehicle_state(data):
        main_track.wirte_vehicle_state(data)
        return 

def callbac_update_velocity_level(data):
        main_track.write_velocity_level(data)
        return 0

def callbac_update_motion_state(data):
        main_track.write_motion_state(data)
        return 0

# Define fuction for Sub and Pub
def mainloop() :
        '''
        code for activate and deactivate the node
        '''
        nodename = 'path_tracker'
        mainloop.active = True
        def signalResponse(data) :
                mainloop.active
                if 'zero_monitor' in data.active_nodes :
                        if nodename in data.active_nodes :
                                mainloop.active = True
                        else :
                                mainloop.active = False
                else :
                        rospy.signal_shutdown('no monitor')
        rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)

        rospy.Subscriber('/path', Path, callbac_update_path)
        rospy.Subscriber('/vehicle_state', VehicleState, callbac_update_vehicle_state)
        rospy.Subscriber('/velocity_level', VelocityLevel, callbac_update_velocity_level)
        rospy.Subscriber('/motion_state', MotionState, callbac_update_motion_state)

        curvature_pub = rospy.Publisher('/curvature', Curvature, queue_size=10)

        rospy.init_node(nodename, anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
                if mainloop.active :
                        curvature_pub.publish(main_track.main_control_loop())
                rate.sleep()
 
if __name__ == '__main__':
        try:
                mainloop()
        except rospy.ROSInterruptException:
                pass
