#!/usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import Image


class ImageMerger():
    '''
    Combines lane map and lidar map
    '''
    def __init__(self):
        self.lane_map = None
        self.lidar_map = None

    def set_lane_map(self, img):
        self.lane_map = ros_numpy.numpify(img)

    def set_lidar_map(self, img):
        self.lidar_map = ros_numpy.numpify(img)

    def merge(self):
        self.lane_map += self.lidar_map
        self.lane_map /= 2
        self.lane_map = None
        self.lidar_map = None
        return ros_numpy.msgify(Image, self.lane_map, encoding='mono8')


def lane_callback(img):
    merger.set_lane_map(img)


def lidar_callback(img):
    merger.set_lidar_map(img)


if __name__ == "__main__":
    merger = ImageMerger()
    rospy.init_node('map_merger', anonymous=True)
    rospy.Subscriber('/lane_map', Image, lane_callback)
    rospy.Subscriber('/lidar_map', Image, lidar_callback)
    obstacle_map_pub = rospy.Publisher('/obstacle_map', Image, queue_size=1)
    try:
        obstacle_map_pub.publish(merger.merge())
    except TypeError:
        rospy.loginfo("None type is in either lane_map or lidar_map")
    rospy.spin()
