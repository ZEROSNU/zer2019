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

min_dis_square = float("inf")
 
class tracker: # Fill it!  

        def __init__ (self) :
                # CONSTANT for Control varialbe range ---------------------------------------------------#
                self.MAX_speed = 5.5                 # [m/s]
                self.MIN_speed = 0                   # [m/s]
                self.MAX_steer = 2000/float(71)      # [degree]
                self.MIN_steer = -2000/float(71)     # [degree]
                self.MAX_brake = 150
                self.MIN_brake = 1

                self.NO_CONTROL = 0
                self.EMERGENCY_BRAKE = 1
                self.NORMAL_S = 2 # Normal path following mode

                #----------------------------------------------------------------------------------------#

                # Comunication rate
                self.pub_rate = 1.0        # [Hz]      #IMPORTANT!!! Main control rate
                self.access_wait_rate = 500 # [Hz]

                # Map varialbe
                self.latest_map = OccupancyGrid() # Last updated map
                self.cur_map = OccupancyGrid()  # Curent map of car
                self.upsign_map = False             # if updated map is different, then 1

                # Hardware constant
                self.L_wheelbase = 1.02     # [m] distance between two wheel axis
                self.L_rear = 0.51          # [m]  distance between cm and rear wheel

                # Control variable
                self.control = Control()    # My order to the car
                self.control_mode = self.NO_CONTROL
                self.writing_state = 0
                self.control_count = 0

                self.control_buff = []
                self.control_time_buff = []
                self.control_buff_size = 600            #TODO: Tune this!
                self.look_ahead = 20                    #TODO: Tune this! now we choose 20th point as look_ahead_point
                self.regular_speed = 2                  #TODO[m/s]: Tune this!
                self.k_gain = 0.28

                self.look_ahead_point = PoseStamped

                # Input variable
                self.cur_state = Control()      # Current state of car
                self.upsign_state = False           # if updated state is different, then 1

                self.latest_path = Path()         # Last updated path
                self.cur_path = Path()          # Current path of car
                self.upsign_path = False            # if updated path is different, then 1

        def S_path_update(self, temp_control_mode):
                try:
                        self.cur_path = copy.deepcopy(self.latest_path)

                        return temp_control_mode
                except:
                        print("S_path_update error ocurr.")
                        return self.EMERGENCY_BRAKE


        def S_path_estimate(self, cur_time, temp_control_mode):
                try:
                        past_time = self.control_time_buff[-1]
                        past_control = self.control_buff[-1]
                        delta_t = cur_time - past_time  # [sec]
                        min_point_seq = 0
                        if past_control.steer != 0:
                                past_radius = self.L_wheelbase / math.tan(abs(past_control.steer) * math.pi / 180)   # [m]
                                delta_theta = (past_control.speed * delta_t) / past_radius      # [radian]
                                delta_dis = 2 * past_radius * math.sin(delta_theta / 2)         # [m]
                                delta_pos_x = delta_dis * math.cos((math.pi - delta_theta) / 2) # [m]
                                delta_pos_y = delta_dis * math.sin((math.pi - delta_theta) / 2) # [m]
                                
                                global min_dis_square
                                min_dis_square = float("inf")
                                if past_control.steer > 0:
                                        for i in self.cur_path.poses:
                                                x_moved = i.pose.position.x - delta_pos_x
                                                y_moved = i.pose.position.y - delta_pos_y
                                                i.pose.position.x, i.pose.position.y = rotaiton_transform(x_moved, y_moved, delta_theta)
                                                if (i.pose.position.x**2 + i.pose.position.y**2) < min_dis_square:
                                                        min_point_seq = i.header.seq
                                                i.header.stamp = rospy.get_rostime().to_sec()
                                else:
                                        for i in self.cur_path.poses:
                                                x_moved = i.pose.position.x + delta_pos_x
                                                y_moved = i.pose.position.y - delta_pos_y
                                                i.pose.position.x, i.pose.position.y = rotaiton_transform(x_moved, y_moved, -delta_theta)
                                                if (i.pose.position.x**2 + i.pose.position.y**2) < min_dis_square:
                                                        min_point_seq = i.header.seq
                                                i.header.stamp = rospy.get_rostime().to_sec()
                        else:           # Past steer angle was 0
                                delta_pos_y = delta_t * past_control.speed
                                for i in self.cur_path.poses:
                                        i.pose.position.y = i.pose.position.y - delta_pos_y
                                        if (i.pose.position.x**2 + i.pose.position.y**2) < min_dis_square:
                                                min_point_seq = i.header.seq
                                        i.header.stamp = rospy.get_rostime().to_sec()
                        
                        for i in self.cur_path.poses:
                                i.header.seq = i.header.seq - min_point_seq

                        return temp_control_mode
                except:
                        print("S_path_estimate error occur.")
                        return self.EMERGENCY_BRAKE

        def Set_look_ahead_point(self):
                try:    #Must be len(self.cur_path.poses) > 0
                        if self.cur_path.poses[-1].header.seq < self.look_ahead:
                                return self.cur_path.poses[-1]
                        else:
                                for i in self.cur_path.poses:
                                        if i.header.seq == self.look_ahead:
                                                return i
                except:
                        print("Path_Upload failed.")
                        return self.EMERGENCY_BRAKE

        def Deicide_steering_angle(self, look_ahead_point, temp_control_mode):
                try:
                        look_ahead_dis = math.sqrt(look_ahead_point.pose.position.x**2 + look_ahead_point.pose.position.y**2)
                        sin_alpha = look_ahead_point.pose.positon.x / look_ahead_dis
                        self.control.steer = math.atan((2 * self.L_wheelbase * sin_alpha )/(self.k_gain * self.regular_speed))
                        return temp_control_mode
                        
                except:
                        print("Deicide_steering_angle error ocurr.")
                        return self.EMERGENCY_BRAKE
                        

        def Control_post_processing(self, temp_control_mode):
                if temp_control_mode == self.EMERGENCY_BRAKE:
                        self.control.speed = self.MIN_speed
                        self.control.steer = 0
                        self.control.brake = self.MAX_brake
                else:
                        if self.control.speed > self.MAX_speed:
                                self.control.speed = self.MAX_speed
                        elif self.control.speed < self.MIN_speed:
                                self.control.speed = self.MIN_speed
                        else:
                                pass
                        
                        if self.control.steer > self.MAX_steer:
                                self.control.steer = self.MAX_steer
                        elif self.control.steer < self.MIN_steer:
                                self.control.steer = self.MIN_steer
                        else:
                                pass

                        if self.control.brake > self.MAX_brake:
                                self.control.brake = self.MAX_brake
                        elif self.control.brake < self.MIN_brake:
                                self.control.brake = self.MIN_brake
                        else:
                                pass

        # Define write functions
        def write_latest_map(self, data):
                self.latest_map = data
                self.upsign_map = True

        def write_latest_path(self, data):
                self.latest_path = copy.deepcopy(data)
                self.upsign_path = True

        def wirte_cur_state(self, data):
                self.cur_state = data

                
        def main_control_loop(self):
                '''
                1. Initialization
                '''
                self.control = Control()
                temp_control_mode = self.NO_CONTROL
                self.control.gear = 0 #0: front 1: neutral 2: rear
                self.control.speed = 0
                self.control.steer = 0 
                self.control.brake = 1
                #TODO: change this to server client later
                cur_time = self.cur_state.header.stamp.to_sec()

                '''
                2. Checking E-stop and Path update
                        1) Check E-stop condition. If E-stop  all control variable would be controlled for E-stop (Q: What is E-stop condition?)
                        2) If it is okay, then path should be updated.
                                2-1) Update path from planner
                                2-2) Or estimate new path by transforming latest path
                '''
                # Update or estimate path, if it fail, then return EMERGEMCY_BRAKE
                if self.upsign_path == True:    #New path is ready
                        temp_control_mode = self.S_path_update(temp_control_mode)
                else:
                        if len(self.control_buff) <= 0:
                                temp_control_mode = self.EMERGENCY_BRAKE
                        else:
                                temp_control_mode = self.S_path_estimate(cur_time, temp_control_mode)
                self.upsign_path = False

                '''
                3. Deciding the control mode
                        1) Check mission and change temp_control_mode as appropriate control_mode
                        2) Change some variables which are related to control_mode (ex) gear, etc
                
                if temp_control_mode == EMERGENCY_BRAKE:
                        pass
                else:   #TODO
                        pass
                '''

                '''
                4. Deciding steering angle
                        1) By path and control mode, we calculate appropriate steering angle.
                '''
                if temp_control_mode == self.EMERGENCY_BRAKE:
                        pass
                else:
                        self.look_ahead_point = self.Set_look_ahead_point()
                        temp_control_mode =  self.Deicide_steering_angle(self.look_ahead_point, temp_control_mode)

                '''
                5. Deciding speed
                        1) By path, control mode and steering angle, we calculate appropriate vehicle speed.
                '''
                
                if temp_control_mode == self.EMERGENCY_BRAKE:
                        pass
                else:
                        self.control.speed = self.regular_speed
                
                
                '''
                6. Post process of main_control_loop
                        1) Save new control variable values at the buffer with current time.
                        2) Return new control variable.
                '''
                self.Control_post_processing(temp_control_mode)

                #Save control and time to buffer
                if len(self.control_buff) >= self.control_buff_size:
                        self.control_buff[0:-1]=self.control_buff[1:]
                        self.control_buff[-1] = self.control
                        self.control_time_buff[0:-1] = self.control_time_buff[1:]
                        self.control_time_buff[-1] = rospy.get_rostime().to_sec()
                else:
                        self.control_buff.append(self.control)
                        self.control_time_buff = np.append(self.control_time_buff, rospy.get_time())
                self.control_count += 1

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
def callbac_update_path(data):  # Fill it!
        main_track.write_latest_path(data)

def callbac_update_map(data):  # Fill it!
        main_track.write_latest_map(data)

def callbac_update_state(data):  # Fill it!
        main_track.wirte_cur_state(data)

# Define fuction for Sub and Pub
def main() :
        rospy.init_node('Path_tracker', anonymous=True)

        #Subscribe
        rospy.Subscriber('planned_path', Path, callbac_update_path)
        rospy.Subscriber('map', OccupancyGrid, callbac_update_map)
        rospy.Subscriber('Current_state', Control, callbac_update_state)

        #Publish
        cont_pub = rospy.Publisher('/Control', Control, queue_size=10)
        path_pub = rospy.Publisher('/Tracking_path', Path, queue_size=10)
        path_pub_2 = rospy.Publisher('/Tracking_path_2', Path, queue_size=10)

        rate = rospy.Rate(main_track.pub_rate)
        while not rospy.is_shutdown() :
                cont_pub.publish(main_track.main_control_loop())
                path_pub.publish(main_track.cur_path)
                path_pub_2.publish(main_track.latest_path)
                # main_track.show_cPoint()
                rate.sleep()



if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass #TODO: maybe E-stop for this case
