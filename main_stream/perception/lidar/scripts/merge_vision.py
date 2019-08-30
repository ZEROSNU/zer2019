#!/usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from core_msgs.msg import ActiveNode
import cv2
isactive = True

class ImageMerger():
    def __init__(self):
        self.lane_map = None
        self.lidar_map = None

    def set_lane_map(self, img):
        self.lane_map = ros_numpy.numpify(img)
    def set_lidar_map(self, img):
        self.lidar_map = ros_numpy.numpify(img)

    def merge(self):
        #self.lane_map += self.lidar_map
        #self.lane_map /= 2
        self.lane_map |= self.lidar_map
        return ros_numpy.msgify(Image, self.lane_map, encoding='mono8')

def lane_callback(img):
    merger.set_lane_map(img)

def lidar_callback(img):
    merger.set_lidar_map(img)


if __name__ == "__main__":
    global isactive
    '''
    code for activate and deactivate the node
    '''
    isactive = True
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
    rospy.init_node('map_merger', anonymous=True)
    rospy.Subscriber('/lane_data', Image, lane_callback)
    rospy.Subscriber('/occupancy_map', Image, lidar_callback)
    obstacle_map_pub = rospy.Publisher('/raw_local_map', Image, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not isinstance(merger.lane_map, type(None)) and not isinstance(merger.lidar_map, type(None)) and isactive:
            obstacle_map_pub.publish(merger.merge())
        rate.sleep()
    #rospy.spin()
    #except TypeError:
    #    rospy.loginfo("None type is in either lane_map or lidar_map")
