#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
import numpy.ma as m
import math
import sys
import time

from core_msgs.msg import Control
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from core_msgs.msg import VelocityLevel
from core_msgs.msg import ActiveNode
from core_msgs.msg import Curvature
from core_msgs.msg import VehicleState

 
class tracker:
        def __init__ (self) :
                # Control states
                self.NO_CONTROL = 0
                self.EMERGENCY_BRAKE = 1
                self.NORMAL_TRACKING = 2
                self.ESTIMATE_TRACKING = 3
        
                # Control variable
                self.look_ahead_distance = 20
                self.look_ahead_point= PoseStamped()

                # Input manage
                self.vehicle_state = VehicleState()

                self.latest_generated_path = Path()
                self.current_path = Path()
                self.update_path = False
                self.path_count = 0

                self.latest_velocity_level = VelocityLevel()
                self.velocity_level = VelocityLevel()
                self.update_velocity_level = False

                self.curvature = Curvature()
                self.curvature_write = 0
                self.curvature_count = 0
                self.curvature_buff = []
                self.curvature_time_buff = []
                self.curvature_buff_size = 200

                self.temp_control_mode = 0
            

        # Define functions
        def Path_update(self):
                try:
                        self.current_path = copy.deepcopy(self.latest_generated_path)
                        self.path_count = self.path_count + 1
                        return self.NORMAL_TRACKING
                except:
                        print("Path_update failed.")
                        return self.EMERGENCY_BRAKE

        def Path_estimate(self, initialize_time):
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
                        q = np.array([[math.cos(rad_angle/2.), 0., 0., math.sin(rad_angle/2.)]])
                        q_inv = np.array([[math.cos(rad_angle/2.)],
                                         [0.],
                                         [0.],
                                         [-math.sin(rad_angle/2.)]])

                        # Update map
                        for i in self.current_path.poses:
                                #update position
                                i.pose.position.x = i.pose.position.x - shift_x
                                i.pose.position.y = i.pose.position.y - shift_y
                                #update orientation   (Check is needed !!!!!)
                                v = np.array([[0],
                                             [i.pose.position.x],
                                             [i.pose.position.y],
                                             [i.pose.position.z]])
                                qv = np.dot(q,v)
                                v = qv * q_inv
                                i.pose.orientation.x = v[1][0]
                                i.pose.orientation.y = v[2][0]
                                i.pose.orientation.z = v[3][0]
                                i.pose.orientation.w = v[0][0]

                        return self.ESTIMATE_TRACKING
                        
                except:
                        print("Path_estimate failed.")
                        return self.EMERGENCY_BRAKE

         
        def Set_look_ahead_point(self, temp_control_mode):
                try:
                        # TODO
                        self.look_ahead_point = copy.deepcopy(self.current_path.poses[self.look_ahead_distance])
                        return temp_control_mode
                except:
                        print("Set_look_ahead_point failed.")
                        return self.EMERGENCY_BRAKE

        def Deicide_curvature(self, temp_control_mode):
                try:
                        if (self.look_ahead_point.pose.position.x == 0) and (self.look_ahead_point.pose.position.y == 0):
                                self.curvature.curvature = 0
                        else:
                                if self.look_ahead_point.pose.position.x >= 0.:
                                        self.curvature.curvature = 2*self.look_ahead_point.pose.position.x/(self.look_ahead_point.pose.position.x**2 + self.look_ahead_point.pose.position.y**2)
                                else:
                                        self.curvature.curvature = -2*self.look_ahead_point.pose.position.x/(self.look_ahead_point.pose.position.x**2 + self.look_ahead_point.pose.position.y**2)
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

        # Main control loop
        def main_control_loop(self):
                '''
                1. Initialization
                '''
                if self.path_count <= 0:
                    self.curvature = Curvature()
                else:
                    self.curvature = copy.deepcopy(self.curvature_buff[-1])

                # Update velocity level
                if self.update_velocity_level == True:
                        self.velocity_level = copy.deepcopy(self.latest_velocity_level)
                else:
                        pass
                self.update_velocity_level = False
 
                temp_control_mode = self.NO_CONTROL

                if self.path_count <= 0:
                    initialize_time = rospy.get_rostime().to_sec()
                else:
                    initialize_time = self.curvature_time_buff[-1]

                '''
                2. Path update
                '''
                if self.update_path == True:
                        temp_control_mode = self.Path_update()
                else:
                        if self.path_count <= 0:
                                temp_control_mode = self.EMERGENCY_BRAKE
                        else:
                                temp_control_mode = self.Path_estimate(initialize_time)
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
                if self.path_count > 0:
                    if len(self.curvature_buff) >= self.curvature_buff_size:
                            self.curvature_buff[0:-1]=self.curvature_buff[1:]
                            self.curvature_buff[-1] = self.curvature
                            self.curvature_time_buff[0:-1] = self.curvature_time_buff[1:]
                            self.curvature_time_buff[-1] = rospy.get_rostime().to_sec()
                    else:
                            self.curvature_buff.append(self.curvature)
                            self.curvature_time_buff = np.append(self.curvature_time_buff, rospy.get_time())
                    self.curvature_count += 1
                
                print("One main tracking cycle is compeleted.")
                return self.curvature


main_track = tracker()

# Some operation functions
def rotaiton_transform(x, y, theta):
        new_x = math.cos(theta) * x - math.sin(theta) * y
        new_y = math.sin(theta) * x + math.cos(theta) * y
        return new_x, new_y

def distance_two_point(A, B):
        return math.sqrt((A.x - B.x)**2 + (A.y - B.y)**2)


# Define callbac functions for Subscriber
def callbac_update_path(data):
        main_track.write_latest_path(data)
        print("got path")
        return 0

def callbac_update_vehicle_state(data):
        main_track.wirte_vehicle_state(data)
        print("got vehicle state")
        return 0

def callbac_update_velocity_level(data):
        main_track.write_velocity_level(data)
        print ("got velocity level")
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
