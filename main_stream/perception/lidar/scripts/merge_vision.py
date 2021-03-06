#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from core_msgs.msg import ActiveNode
from core_msgs.msg import MotionState
from core_msgs.msg import CenterLine
from core_msgs.msg import VelocityLevel
from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import PoseStamped
import tf_conversions
import tf2_ros
import tf
from tf2_msgs.msg import TFMessage
import cv2
isactive = True
motion = None
is_left_cen = True
is_right_cen = True
IMG_SIZE = 200

LANE_PIXEL_VALUE = 100
STOPLINE_PIXEL_VALUE = 200
OBSTACLE_BUFFER = [49,23] # buff obstacles as much as circumscribed circle of the car 
LANE_BUFFER = [1,23]
ROTATION = [3*np.pi/4, np.pi/2, np.pi/4]

STOP_TIME = 3 # sec
CHANGE_LINE_TIME = 3 # sec
MISSION_RATE = 10

DRIVING_VELOCITY = 2.
HIGH_VELOCITY = 3.
LOW_VELOCITY = 1.

class ImageMerger():
    def __init__(self):
        self.lane_map = None
        self.lidar_map = None
        self.merged_map = None
        self.left_cor = [0, 0] #upper bound of left lane
        self.right_cor = [0, 0] #upper bound if right lane
        self.mid_cor = [0, 0] #medium of two coors
        self.turn_count = 0
        self.count = 0
        self.pose = PoseStamped()
        self.velocity = VelocityLevel()

    def set_lane_map(self, img):
        self.lane_map = ros_numpy.numpify(img)
        
    def set_lidar_map(self, img):
        self.lidar_map = ros_numpy.numpify(img)
        
    def merge(self):
        #self.lane_map += self.lidar_map
        #self.lane_map /= 2

        obst_kernel = np.ones((OBSTACLE_BUFFER[0],OBSTACLE_BUFFER[1]),np.uint8)
        self.lidar_map = cv2.dilate(self.lidar_map, obst_kernel, iterations=1)
        
        self.merged_map = self.lidar_map | self.lane_map
        

    def find_lane_cor(self):
        global is_left_cen, is_right_cen
        '''
        left_array = self.lane_map[1:, 0]
        mid_array = self.lane_map[1, :]
        right_array = self.lane_map[1:, IMG_SIZE - 1]

        left_center_array = self.lane_map[:,int(IMG_SIZE/4)]
        mid_center_array = self.lane_map[IMG_SIZE/2. :]
        right_center_array = self.lane_map[:,int(IMG_SIZE/4*3)]
        
        left_cor = np.where(left_array == 0)[0]
        mid_cor = np.where(mid_array == 0)[0]
        right_cor = np.where(right_array == 0)[0]

        left_center_cor = np.where(left_center_array == 0)[0]
        mid_center_cor = np.where(mid_center_array == 0)[0]
        right_center_cor = np.where(right_center_array == 0)[0]

        left_num = len(left_cor)
        middle_num = len(mid_cor)
        right_num = len(right_cor)
        '''
        
        if not isinstance(self.merged_map, type(None)):
            for i in range(IMG_SIZE):
                if len(np.where(self.merged_map[i,:] == 0)[0]) < 10:
                    lane_mask = self.merged_map == 100
                    if not is_left_cen and not is_right_cen:
                        pass
                    elif not is_left_cen:
                        lane_mask[:,IMG_SIZE/3*2:] = False
                    elif not is_right_cen:
                        lane_mask[:,:IMG_SIZE/3] = False
                    else:
                        pass
                    self.merged_map[lane_mask] = 0


        goalpose_y = 50
        if isinstance(self.merged_map, type(None)):
            mid_center_array = self.lane_map[goalpose_y, :]
        else:
            mid_center_array = self.merged_map[goalpose_y, :]
        mid_center_cor = np.where(mid_center_array == 0)[0]
        available_region = [[]]
        first_cor = mid_center_cor[0]
        idx = 0
        for cor in mid_center_cor[1:]:
            if cor - first_cor == 1:
                available_region[idx].append(cor)
            else:
                available_region.append([])
                idx += 1
                available_region[idx].append(cor)
                
            first_cor = cor
        max_len = 0
        actual_region = []
        for region in available_region:
            
            if len(region) > max_len:
                actual_region = region
                max_len = len(region)
        
        '''
        if left_num == 0 and right_num == 0 and middle_num != 0:
            self.left_cor = [IMG_SIZE/2, mid_center_cor[0]]
            self.right_cor = [IMG_SIZE/2, mid_center_cor[-1]]
        elif left_num != 0 and middle_num != 0:
            self.left_cor = [left_center_cor[-1], mid_center_cor[0]]
            self.right_cor = [left_center_cor[0], mid_center_cor[-1]]
        elif left_num != 0 and right_num == 0 and middle_num == 0:
            self.left_cor = [left_center_cor[-1], mid_center_cor[0]]
            self.right_cor = [left_center_cor[0], mid_center_cor[-1]]
        elif middle_num != 0 and right_num != 0:
            self.left_cor = [right_center_cor[0], mid_center_cor[0]]
            self.right_cor = [right_center_cor[-1], mid_center_cor[-1]]
        elif left_num == 0 and right_num != 0 and middle_num == 0:
            self.left_cor = [right_center_cor[0], mid_center_cor]
            self.right_cor = [right_center_cor[-1], IMG_SIZE - 1]
        else:
            self.left_cor = [0, IMG_SIZE /2]
            self.right_cor = [0, IMG_SIZE /2]
        self.mid_cor = np.add(self.left_cor, self.right_cor) / 2
        '''
        self.left_cor = (goalpose_y,actual_region[0])
        self.right_cor = (goalpose_y, actual_region[-1])
        self.mid_cor = np.add(self.left_cor, self.right_cor) / 2
        
        if self.mid_cor[0] < 0 or self.mid_cor[0] >= IMG_SIZE or self.mid_cor[1] < 0 or self.mid_cor[1] >=IMG_SIZE:
            self.mid_cor[0] = goalpose_y
            self.mid_cor[1] = IMG_SIZE/2
        
    def set_goal(self):
        global motion
        print(motion)
        #self.velocity.velocity_level = DRIVING_VELOCITY
        if motion == "LEFT_MOTION":
            if is_left_cen:
                velocity= LOW_VELOCITY
                #self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255
                self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
                self.pose.pose.position.y = (IMG_SIZE - self.mid_cor[0]) / 100.0 * 3
                theta = ROTATION[0]

            else:
                velocity = LOW_VELOCITY
                #self.lane_map[0][0] = 255
                self.lane_map[self.lane_map == LANE_PIXEL_VALUE] = 0
                if self.turn_count <= CHANGE_LINE_TIME * MISSION_RATE:
                    self.pose.pose.position.x = -1
                    self.pose.pose.position.y = 3
                    self.turn_count += 1
                else:
                    velocity = DRIVING_VELOCITY
                    self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
                    self.pose.pose.position.y = (IMG_SIZE - self.mid_cor[0]) / 100.0 * 3
                theta = ROTATION[0]
                
        elif motion == "RIGHT_MOTION":
            if is_right_cen:
                velocity = LOW_VELOCITY
                self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
                self.pose.pose.position.y = (IMG_SIZE - self.mid_cor[0]) / 100.0 * 3
                theta = ROTATION[2]
            
            else:
                velocity = LOW_VELOCITY
                self.lane_map[self.lane_map == LANE_PIXEL_VALUE] = 0
                self.pose.pose.position.x = 1
                self.pose.pose.position.y = 3
                theta = ROTATION[2]
            
                
        elif motion == "FORWARD_MOTION":
            velocity = HIGH_VELOCITY
            #self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255
            self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
            self.pose.pose.position.y = (IMG_SIZE - self.mid_cor[0]) / 100.0 * 3
            theta = ROTATION[1]
                    
        elif motion == "HALT":
            velocity = LOW_VELOCITY
            #self.lane_map[min(self.mid_cor[0] + 50, IMG_SIZE - 1)][self.mid_cor[1]] =  255
            if STOPLINE_PIXEL_VALUE in self.lane_map[IMG_SIZE/2:,:]:
                stop_line_y = np.where(self.lane_map[IMG_SIZE/2:,:]==STOPLINE_PIXEL_VALUE)[0][0]
            else:
                stop_line_y = -50
            if stop_line_y != -50:
                velocity = 0
            self.pose.pose.position.y = 4.5
            self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
            theta = ROTATION[1]
        
        else:
            velocity = LOW_VELOCITY
            self.pose.pose.position.x = (self.mid_cor[1] - IMG_SIZE/2) / 100.0 * 3
            self.pose.pose.position.y = (IMG_SIZE - self.mid_cor[0]) / 100.0 * 3
            theta = ROTATION[1]
    
        if motion != "HALT":
            self.lane_map[self.lane_map==STOPLINE_PIXEL_VALUE] = 0
        if motion != "LEFT_MOTION":
            self.turn_count = 0
            
        qframe = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)   
        self.pose.pose.orientation.x = qframe[0]
        self.pose.pose.orientation.y = qframe[1] 
        self.pose.pose.orientation.z = qframe[2]
        self.pose.pose.orientation.w = qframe[3]
        if not isinstance(motion, type(None)):
            if "SLOW" in motion:
                velocity = LOW_VELOCITY
        print(velocity)
        return velocity
        

def lane_callback(img):
    merger.set_lane_map(img)

def lidar_callback(img):
    merger.set_lidar_map(img)

def mission_callback(data):
    global motion
    motion = data.motion_state

def is_center_callback(data):
    global is_left_cen, is_right_cen
    is_left_cen = data.is_center_left
    is_right_cen = data.is_center_right

if __name__ == "__main__":
    '''
    code for activate and deactivate the node
    '''
    isactive = True
    nodename = 'map_merger'
    def signalResponse(data) :
        global isactive
        if 'zero_monitor' in data.active_nodes :
            if nodename in data.active_nodes :
                isactive = True
            else :
                isactive = False
        else :
            rospy.signal_shutdown('no monitor')
    rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)
    '''
    ...
    '''
    merger = ImageMerger()

    rospy.init_node(nodename, anonymous=True)
    rospy.Subscriber('/lane_data', Image, lane_callback)
    rospy.Subscriber('/occupancy_map', Image, lidar_callback)
    rospy.Subscriber('/motion_state', MotionState, mission_callback) #mission master
    rospy.Subscriber('/is_center', CenterLine, is_center_callback) #left and right is centerline or not
    
    #__import__('pdb').set_trace()
    obstacle_map_pub = rospy.Publisher('/raw_local_map', Image, queue_size=1)
    local_map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size = 1)
    goal_pose_pub = rospy.Publisher('/goal_pose', PoseStamped, queue_size = 1)
    velocity_level_pub = rospy.Publisher('/velocity_level', VelocityLevel, queue_size = 1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not isinstance(merger.lane_map, type(None)) and not isinstance(merger.lidar_map, type(None)) and isactive:
            occupancy = OccupancyGrid()
            

            merger.velocity.velocity_level = merger.set_goal()
            merger.merge()
            merger.find_lane_cor()
            merged_img = ros_numpy.msgify(Image, merger.merged_map, encoding='mono8')
            merged_arr = ros_numpy.numpify(merged_img)

            if isactive:
                obstacle_map_pub.publish(merged_img)
            
            occupancy.data = list(np.flipud(merged_arr).ravel().astype(np.int8))
            
            occupancy.header.stamp = rospy.Time.now()
            occupancy.header.frame_id = "car_frame"
            occupancy.info.resolution = 0.03
            occupancy.info.width = merged_arr.shape[0]
            occupancy.info.height = merged_arr.shape[1]
            occupancy.info.origin.position.x = -occupancy.info.width / 2 * occupancy.info.resolution
            occupancy.info.origin.position.y = 0
            occupancy.info.origin.position.z = 0
            occupancy.info.origin.orientation.z = 0
            occupancy.info.origin.orientation.w = 1
            if isactive:
                local_map_pub.publish(occupancy)

            merger.pose.header.stamp = rospy.Time.now()
            merger.pose.header.frame_id = "car_frame"
            if isactive:
                goal_pose_pub.publish(merger.pose)

            merger.velocity.header.stamp = rospy.Time.now()
            merger.velocity.header.frame_id = "car_frame"
            if isactive:
                velocity_level_pub.publish(merger.velocity)
        
        elif not isinstance(merger.merged_map, type(None)):
            merged_img = ros_numpy.msgify(Image, merger.merged_map, encoding='mono8')
            merged_arr = ros_numpy.numpify(merged_img)
            if isactive:    
                obstacle_map_pub.publish(merged_img)
            occupancy = OccupancyGrid()
            occupancy.data = list(np.flipud(merged_arr).ravel().astype(np.int8))
            occupancy.header.stamp = rospy.Time.now()
            occupancy.header.frame_id = "car_frame"
            occupancy.info.resolution = 0.03
            occupancy.info.width = merged_arr.shape[0]
            occupancy.info.height = merged_arr.shape[1]
            occupancy.info.origin.position.x = -occupancy.info.width / 2 * occupancy.info.resolution
            occupancy.info.origin.position.y = 0
            occupancy.info.origin.position.z = 0
            occupancy.info.origin.orientation.z = 0
            occupancy.info.origin.orientation.w = 1
            if isactive:
                obstacle_map_pub.publish(merged_img)
            if isactive:
                local_map_pub.publish(occupancy)

            pose = PoseStamped()
            pose.header.frame_id = "car_frame"
            if isactive:
                goal_pose_pub.publish(pose)

            velocity = VelocityLevel()
            velocity.header.frame_id = "car_frame"
            if isactive:
                velocity_level_pub.publish(velocity)
        



        rate.sleep()
    #rospy.spin()
    #except TypeError:
    #    rospy.loginfo("None type is in either lane_map or lidar_map")
