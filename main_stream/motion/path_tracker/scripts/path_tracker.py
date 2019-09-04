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
import tf

from core_msgs.msg import Control
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
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
                self.look_ahead_distance = 25
                self.look_ahead_point= PoseStamped()

                # Input manage
                self.vehicle_state = VehicleState()

                self.latest_generated_path = Path()
                self.current_path = Path()
                self.update_path = False

                self.latest_velocity_level = VelocityLevel()
                self.velocity_level = VelocityLevel()
                self.update_velocity_level = False

                self.latest_motion_state = MotionState()
                self.motion_state = MotionState()
                self.update_motion_state = False
                self.motion_state_buff = [self.motion_state]
                self.motion_state_buff_size = 20

                self.curvature = Curvature()
                self.curvature.gear = 0
                self.curvature_buff = [self.curvature]
                self.curvature_time_buff = [rospy.get_rostime().to_sec()]
                self.curvature_buff_size = 200

                self.latest_vehicle_pose = Pose()
                self.update_vehicle_pose = False
                self.vehicle_pose = Pose()

                self.temp_control_mode = 0

                # Variable for parking
                self.is_PARKING = False
                self.init_parking_path = Path()

        # Define functions
        def Path_update(self):
                try:
                        self.current_path = copy.deepcopy(self.latest_generated_path)
                        return self.NORMAL_TRACKING
                except:
                        print("Path_update failed.")
                        return self.EMERGENCY_BRAKE
        #Check! NEED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        def Path_parsing(self):
                try:    
                        last_x = 0.
                        last_y = 0.
                        idx = 0
                        for pose in self.current_path.poses:
                               # yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                                # base of yaw? Check needed
                                x_ori = math.sin(yaw)
                                y_ori = math.cos(yaw)

                                diff_x = pose.pose.position.x - last_x
                                diff_y = pose.pose.position.x - last_y

                                if (x_ori * diff_x + y_ori * diff_y) < 0:
                                        if idx == 0:    # rear
                                                self.curvature.gear = 2
                                                return self.NORMAL_TRACKING
                                        self.curvature.gear = 0         # yet, forward
                                        self.current_path.poses = self.current_path.poses[:idx]
                                        break
                                idx = idx + 1
                        return self.NORMAL_TRACKING
                except:
                        print("Path parsing failed.")
                        return self.EMERGENCY_BRAKE
        
        def Global2Local(Self):
                try:
                        vehicle_yaw = 2 * np.arctan2(self.vehicle_pose.orientation.z, self.vehicle_pose.orientation.w)
                        vehicle_q = tf.transformations.quaternion_from_euler(0,0,0)
                        self.vehicle_pose.orientation.x = vehicle_q[0]
                        self.vehicle_pose.orientation.y = vehicle_q[1]
                        self.vehicle_pose.orientation.z = vehicle_q[2]
                        self.vehicle_pose.orientation.w = vehicle_q[3]
                        vehicle_x = self.vehicle_pose.position.x
                        self.vehicle_pose.position.x = 0
                        vehicle_y = self.vehicle_pose.position.y
                        self.vehicle_pose.position.y = 0
                        for pose in self.current_path.poses:
                                yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                                yaw = yaw - vehicle_yaw
                                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                                pose.pose.orientation.x = quaternion[0]
                                pose.pose.orientation.y = quaternion[1]
                                pose.pose.orientation.z = quaternion[2]
                                pose.pose.orientation.w = quaternion[3]
                                pose.pose.position.x = pose.pose.position.x - vehicle_x
                                pose.pose.position.y = pose.pose.position.y - vehicle_y
                except:
                        print("Converting global coordinate to local coordinate is failed.")
                        return self.EMERGENCY_BRAKE

        # DEBUG NEEDED!!!!!!!!!!!!!
        def Path_estimate(self, initialize_time):       # TODO: Consider motion_state_buff[-1], it was parking or not?
                #try:
                passed_time = initialize_time - self.curvature_time_buff[-1]
                
                # Estimate current vehicle state (position and orientation)
                theta = self.vehicle_state.speed * self.curvature.curvature * passed_time
                if self.curvature.curvature == 0.:
                        shift_y = self.vehicle_state.speed * passed_time
                        shift_x = 0.
                        shift_yaw = math.pi/2.
                else:
                        shift_y = math.sin(theta)/(self.curvature.curvature)
                        if self.vehicle_state.steer >= 0.:
                                shift_x = (1. - math.cos(theta))/self.curvature.curvature
                                shift_yaw = math.pi/2. - theta
                        else:
                                shift_x = (math.cos(theta) - 1.)/self.curvature.curvature
                                shift_yaw = math.pi/2. + theta


                for pose in self.current_path.poses:
                        yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                        yaw = yaw - shift_yaw
                        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                        pose.pose.orientation.x = quaternion[0]
                        pose.pose.orientation.y = quaternion[1]
                        pose.pose.orientation.z = quaternion[2]
                        pose.pose.orientation.w = quaternion[3]
                        pose.pose.position.x = pose.pose.position.x - shift_x
                        pose.pose.position.y = pose.pose.position.y - shift_y

                return self.ESTIMATE_TRACKING
                        
                #except:
                #        print("Path_estimate failed.")
                #        return self.EMERGENCY_BRAKE

         
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
                       
        '''        
        def Set_look_ahead_distance(self):
                #TODO adaptive look_ahead_distance
                self.look_ahead_distance = 20
        '''       
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

        def wirte_vehicle_pose(self, data):
                self.latest_vehicle_pose = data
                self.update_vehicle_pose = True

        # Define callbac functions for Subscriber
        def callbac_update_path(self, data):
                self.write_latest_path(data)
                return 0

        def callbac_update_vehicle_state(self,data):
                self.wirte_vehicle_state(data)
                return 

        def callbac_update_velocity_level(self,data):
                self.write_velocity_level(data)
                return 0

        def callbac_update_motion_state(self,data):
                self.write_motion_state(data)
                return 0

        def callbac_update_pose(self, data):
                self.wirte_vehicle_pose(data)
                return 0


        # Main control loop
        def main_control_loop(self):
                '''
                1. Initialization
                '''
                # Initialize curvature
                self.curvature = copy.deepcopy(self.curvature_buff[-1])

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
                        if self.update_vehicle_pose == True:
                                self.vehicle_pose = copy.deepcopy(self.latest_vehicle_pose)
                                #self.correct_vehicle_pose = True
                        self.update_vehicle_pose = False
                else:
                        self.is_PARKING = False

      
                initialize_time = self.curvature_time_buff[-1]

                '''
                2. Path update
                '''
                if self.update_path == True:
                        temp_control_mode = self.Path_update()
                        if self.is_PARKING == True:
                                temp_control_mode = self.Global2Local()
                                temp_control_mode = self.Path_parsing()
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
                # Fail Policy
                if temp_control_mode ==  self.EMERGENCY_BRAKE:
                        self.curvature = copy.deepcopy(self.curvature_buff[-1])

                '''
                4. Save and return new curvature
                '''
                if len(self.curvature_buff) >= self.curvature_buff_size:
                        self.curvature_buff[0:-1]=self.curvature_buff[1:]
                        self.curvature_buff[-1] = self.curvature
                        self.curvature_time_buff[0:-1] = self.curvature_time_buff[1:]
                        self.curvature_time_buff[-1] = rospy.get_rostime().to_sec()
                        self.motion_state_buff[0:-1] = self.motion_state_buff[1:]
                        self.motion_state_buff[-1] = self.motion_state
                else:
                        self.curvature_buff.append(self.curvature)
                        self.curvature_time_buff = np.append(self.curvature_time_buff, rospy.get_rostime().to_sec())
                        self.motion_state_buff.append(self.motion_state)
                
                print("One main tracking cycle is compeleted.")
                return self.curvature




# Some operation functions
def rotation_transform(x, y, theta):
        new_x = math.cos(theta) * x - math.sin(theta) * y
        new_y = math.sin(theta) * x + math.cos(theta) * y
        return new_x, new_y

def distance_two_point(A, B):
        return math.sqrt((A.x - B.x)**2 + (A.y - B.y)**2)



# Define fuction for Sub and Pub
def mainloop() :
        '''
        code for activate and deactivate the node
        '''
        nodename = 'path_tracker'
        mainloop.active = True
        rospy.init_node(nodename, anonymous=True)
        main_track = tracker()
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

        rospy.Subscriber('/path', Path, main_track.callbac_update_path)
        rospy.Subscriber('/vehicle_state', VehicleState, main_track.callbac_update_vehicle_state)
        rospy.Subscriber('/velocity_level', VelocityLevel, main_track.callbac_update_velocity_level)
        rospy.Subscriber('/motion_state', MotionState, main_track.callbac_update_motion_state)
        rospy.Subscriber('/vehicle_pose', Pose, main_track.callbac_update_pose)

        curvature_pub = rospy.Publisher('/curvature', Curvature, queue_size=10)


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
