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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
from std_msgs.msg import Int8

 
class tracker:
        def __init__ (self) :
                # Control constants
                self.MAX_speed = 5.5                 # [m/s]
                self.MIN_speed = 0.                   # [m/s]
                #self.MAX_steer = 2000/float(71)      # [degree]
                #self.MIN_steer = -2000/float(71)     # [degree]
                self.MAX_steer = 180.
                self.MIN_steer = -180.
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
                self.wheelbase = 1.02           #[m] distance between two wheel axis
                self.cm2rear = 0.51             #[m] distance between center of mass and rear axis
        
                # Control variable
                self.control = Control()
                self.control_mode = self.NO_CONTROL
                self.control_write = 0
                self.control_count = 0
                self.control_buff = []
                self.control_time_buff = []
                self.control_buff_size = 200
                self.look_ahead_distance = 20
                self.look_ahead_point= PoseStamped()

                # Input manage
                self.vehicle_state = Control()

                self.latest_generated_path = Path()
                self.current_path = Path()
                self.update_path = False

                self.latest_velocity_level = Int16()
                self.velocity_level = Int16()
                self.update_velocity_level = False

                self.latest_gear_level = Int8()
                self.gear_level = Int8()
                self.update_gear_level = False

        # Define functions
        def Path_update(self):
                try:
                        self.current_path = copy.deepcopy(self.latest_generated_path)
                        return self.NORMAL_TRACKING
                except:
                        print("Path_update failed.")
                        return self.EMERGENCY_BRAKE

        def Path_estimate(self, initialize_time):
                try:
                        curvature = self.Steer_to_curvature(self.vehicle_state.steer)
                        passed_time = initialize_time - self.control_time_buff[-1]
                        
                        # Estimate current vehicle state (position and orientation)
                        theta = self.vehicle_state.speed * curvature * passed_time
                        shift_y = math.sin(theta)/curvature
                        if self.vehicle_state.steer >= 0.:
                                shift_x = (1. - math.cos(theta))/curvature
                                rad_angle = math.pi/2. - theta
                        else:
                                shitf_x = (math.cos(theta) - 1.)/curvature
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

        def Deicide_steering_angle(self, temp_control_mode):
                try:
                        #TODO
                        curvature = 1.
                        if (self.look_ahead_point.pose.position.x == 0) and (self.look_ahead_point.pose.position.y == 0):
                                return self.EMERGENCY_BRAKE
                        else:
                                if self.look_ahead_point.pose.position.x >= 0.:
                                        curvature = 2*self.look_ahead_point.pose.position.x/(self.look_ahead_point.pose.position.x**2 + self.look_ahead_point.pose.position.y**2)
                                else:
                                        curvature = -2*self.look_ahead_point.pose.position.x/(self.look_ahead_point.pose.position.x**2 + self.look_ahead_point.pose.position.y**2)
                                self.control.steer = self.Curvature_to_steer(curvature)
                        return temp_control_mode

                        
                except:
                        print("Deicide_steering_angle failed.")
                        return self.EMERGENCY_BRAKE
                       
                        

        def Control_post_processing(self, temp_control_mode):
                # self.control.control_mode = temp_control_mode
                if temp_control_mode == self.NO_CONTROL:        # Human control mode
                        self.control.is_auto = False
                        self.control.estop = False
                elif temp_control_mode == self.EMERGENCY_BRAKE: # Estop mode
                        self.control.is_auto = True
                        self.control.estop = True
                        self.control.gear = 0
                        self.control.speed = self.MIN_speed
                        self.control.steer = 0.
                        self.control.brake = self.MAX_brake
                else:                                           # Nomal mode
                        self.control.is_auto = True
                        self.control.estop = False
                        if self.control.steer > self.MAX_steer:
                                self.control.steer = self.MAX_steer
                        elif self.control.steer < self.MIN_steer:
                                self.control.steer = self.MIN_steer
                        else:
                                pass
        
        def Steer_to_curvature(self, steer):
                # We need some expriments
                curvature = 1.1              
                return curvature

        def Curvature_to_steer(self, curvature):
                # We need some expriments
                steer = 1.1
                return steer

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

        def write_gear_level(self, data):
                self.latest_gear_level = data
                self.update_gear_level = True
        
        # Main control loop
        def main_control_loop(self):
                '''
                1. Initialization
                '''

                '''
                if self.update_vehicle_state == True:
                        self.control = copy.deepcopy(self.vehicle_state)
                else:
                        self.control = copy.deepcopy(self.control_buff[-1])
                self.update_vehicle_state = False
                '''
                self.control = Control()

                # Update velocity level
                if self.update_velocity_level == True:
                        self.velocity_level = copy.deepcopy(self.latest_velocity_level)
                else:
                        pass
                self.update_velocity_level = False

                # Update gear level
                if self.update_gear_level == True:
                        self.gear_level = copy.deepcopy(self.latest_gear_level)
                else:
                        pass
                self.update_gear_level = False
                
                # Initialize contorl variables
                self.control.is_auto = False
                self.control.estop = False
                self.control.gear = self.gear_level
                self.control.speed = self.velocity_level
                self.control.steer = 0 
                self.control.brake = self.MAX_brake

                temp_control_mode = self.NO_CONTROL
                initialize_time = self.vehicle_state.header.stamp.to_sec()

                '''
                2. Checking E-stop and Path update
                        1) Check E-stop condition. If E-stop  all control variable would be controlled for E-stop (Q: What is E-stop condition?)
                        2) If it is okay, then path should be updated.
                                2-1) Update path from planner
                                2-2) Or estimate new path by transforming latest path
                '''
                if self.update_path == True:
                        temp_control_mode = self.Path_update()
                else:
                        if len(self.control_buff) <= 0:
                                temp_control_mode = self.EMERGENCY_BRAKE
                        else:
                                temp_control_mode = self.Path_estimate(initialize_time)
                self.update_path = False

                '''
                3. Deciding steering angle
                        1) By path and control mode, we calculate appropriate steering angle.
                '''
                if temp_control_mode == self.EMERGENCY_BRAKE:
                        pass
                else:
                        temp_control_mode = self.Set_look_ahead_point(temp_control_mode)
                        temp_control_mode = self.Deicide_steering_angle(temp_control_mode)

                '''
                4. Post process of main_control_loop
                        1) Save new control variable values at the buffer with current time.
                        2) Return new control variable.
                '''
                self.Control_post_processing(temp_control_mode)

                if len(self.control_buff) >= self.control_buff_size:
                        self.control_buff[0:-1]=self.control_buff[1:]
                        self.control_buff[-1] = self.control
                        self.control_time_buff[0:-1] = self.control_time_buff[1:]
                        self.control_time_buff[-1] = rospy.get_rostime().to_sec()
                else:
                        self.control_buff.append(self.control)
                        self.control_time_buff = np.append(self.control_time_buff, rospy.get_time())
                self.control_count += 1
                
                print("One main tracking cycle is compeleted.")
                return self.control


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

def callbac_update_vehicle_state(data):
        main_track.wirte_vehicle_state(data)

def callbac_update_velocity_level(data):
        main_track.write_velocity_level(data)

def callbac_update_gear_level(data):
        main_track.write_gear_level(data)


# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Path_tracker', anonymous=True)

        #Subscribe
        rospy.Subscriber('planned_path', Path, callbac_update_path)
        rospy.Subscriber('vehicle_state', Control, callbac_update_vehicle_state)
        rospy.Subscriber('velocity_level', Int16, callbac_update_velocity_level)
        rospy.Subscriber('gear_level', Int8, callbac_update_gear_level)
        
        #Publish
        cont_pub = rospy.Publisher('/ideal_control', Control, queue_size=10)
        temp_pub = rospy.Publisher('/current_path', Path, queue_size=10)
        rate = rospy.Rate(main_track.pub_rate)
        while not rospy.is_shutdown() :
                cont_pub.publish(main_track.main_control_loop())
                temp_pub.publish(main_track.current_path)
                rate.sleep()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case
