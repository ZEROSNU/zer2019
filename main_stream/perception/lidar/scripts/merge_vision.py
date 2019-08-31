#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from core_msgs.msg import ActiveNode
from core_msgs.msg import MotionState
from core_msgs.msg import CenterLine
import cv2
isactive = True
motion = None
is_left_cen = True
is_right_cen = True
IMG_SIZE = 200
class ImageMerger():
    def __init__(self):
        self.lane_map = None 
        self.lidar_map = None
        self.left_cor = [0, 0] #upper bound of left lane
        self.right_cor = [0, 0] #upper bound if right lane
        self.mid_cor = [0, 0] #medium of two coors
    def set_lane_map(self, img):
        self.lane_map = ros_numpy.numpify(img)
    def set_lidar_map(self, img):
        self.lidar_map = ros_numpy.numpify(img)
        
    def merge(self):
        #self.lane_map += self.lidar_map
        #self.lane_map /= 2
        self.lane_map |= self.lidar_map
        return ros_numpy.msgify(Image, self.lane_map, encoding='mono8')

    def find_lane_cor(self):
        left_array = self.lane_map[1:, 0]
        mid_array = self.lane_map[0, :]
        right_array = self.lane_map[1:, IMG_SIZE - 1]

        left_num = np.count_nonzero(left_array)
        middle_num = np.count_nonzero(mid_array)
        right_num = np.count_nonzero(right_array)

        left_cor = np.nonzero(left_array)
        mid_cor = np.nonzero(mid_array)
        right_cor = np.nonzero(right_array)
        if left_num == 0 and right_num == 0 and middle_num != 0:
            self.left_cor = [0, mid_cor[0][0]]
            self.right_cor = [0, mid_cor[0][-1]]
        elif left_num != 0 and middle_num != 0:
            self.left_cor = [left_cor[0][-1], 0]
            self.right_cor = [0, mid_cor[0][-1]]
        elif left_num != 0 and right_num == 0 and middle_num == 0:
            self.left_cor = [left_cor[0][-1], 0]
            self.right_cor = [left_cor[0][0], 0]
        elif middle_num != 0 and right_num != 0:
            self.left_cor = [0, mid_cor[0][0]]
            self.right_cor = [0, right_cor[0][-1]]
        elif left_num == 0 and right_num != 0 and middle_num == 0:
            self.left_cor = [right_cor[0][0], IMG_SIZE - 1]
            self.right_cor = [right_cor[0][-1], IMG_SIZE - 1]
        self.mid_cor = np.add(self.left_cor, self.right_cor) / 2

    def set_goal(self):
        if motion == "LEFT_MOTION":
            if is_left_cen:
                self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255
            else:
                self.lane_map[0][0] = 255
        elif motion == "RIGHT_MOTION":
            if is_right_cen:
                self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255
            else:
                self.lane_map[0][IMG_SIZE - 1] = 255
        elif motion == "FORWARD_MOTION":
            self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255
        elif motion == "HALT":
            print self.mid_cor
            self.lane_map[self.mid_cor[0] + 50][self.mid_cor[1]] =  255
        elif motion == "FORWARD_MOTION_SLOW":
            self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255  
        else:
            self.lane_map[self.mid_cor[0]][self.mid_cor[1]] =  255

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
    global isactive
    '''
    code for activate and deactivate the node
    '''
    isactive = True
    nodename = 'map_merger'
    def signalResponse(data) :
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
    rospy.Subscriber('/lane_map', Image, lane_callback)
    rospy.Subscriber('/occupancy_map', Image, lidar_callback)
    rospy.Subscriber('/motion_state', MotionState, mission_callback) #mission master
    rospy.Subscriber('/is_center', CenterLine, is_center_callback) #left and right is centerline or not
       
    obstacle_map_pub = rospy.Publisher('/raw_local_map', Image, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not isinstance(merger.lane_map, type(None)) and not isinstance(merger.lidar_map, type(None)) and isactive:
            merger.find_lane_cor()
            merger.set_goal()
            obstacle_map_pub.publish(merger.merge())
        rate.sleep()
    #rospy.spin()
    #except TypeError:
    #    rospy.loginfo("None type is in either lane_map or lidar_map")
