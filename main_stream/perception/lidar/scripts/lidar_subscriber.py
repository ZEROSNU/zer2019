#!/usr/bin/env python

import rospy
import ros_numpy
import time
import math
import numpy as np
from core_msgs.msg import ActiveNode
from sensor_msgs.msg import LaserScan, Image
import cv2
DEBUG = False
XY_RES = 0.03  # x-y grid resolution [m]
YAW_RES = 0.00872664619237  # yaw angle resolution [rad] = 0.05 degree
MIN_X = -3.0
MIN_Y = 0.0
MAX_X = 3.0
MAX_Y = 6.0
LEN_X = int(round((MAX_X - MIN_X) / XY_RES))
LEN_Y = int(round((MAX_Y - MIN_Y) / XY_RES))

isactive = True

def callback(data):
    global isactive
    #t1 = time.time()

    r = list(data.ranges)
    r = r[90:-90]         # len(r)=361

    pmap = generate_ray_casting_grid_map(r)
    pmap[:LEN_X - 30,:] = pmap[30:,:]
    pmap[LEN_X - 30:,:] = 0

    img_msg = ros_numpy.msgify(Image, pmap, encoding='mono8')
    if isactive:
        lidar_map_pub.publish(img_msg)

    # show calculation time
    #t2 = time.time()
    #rospy.loginfo("Time taken to calculate single image: %f" % (t2 - t1))


class PrecastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle


def precasting():            # initiating mapping process. so we run this func once. Not everytime when we read data.

    precast = [[] for _ in range(int(round(math.pi / YAW_RES)) + 1)]
    
    for ix in range(LEN_X):

        for iy in range(LEN_Y):

            px = ix * XY_RES + MIN_X
            py = iy * XY_RES + MIN_Y

            d = math.sqrt(px**2 + py**2)
            angle = atan_zero_to_twopi(py, px)
            angle_id = int(math.floor(angle / YAW_RES))
            pc = PrecastDB()  # too much time
            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle
            precast[angle_id].append(pc)

    return precast


def generate_ray_casting_grid_map(r):

    #:param r: list, lidar data, size = 361
    #:return: 2D numpy array, containing the obstacles
    for i in range(len(r)):
        if abs(r[i] * math.cos(i * YAW_RES)) > 2.97 \
                or abs(r[i] * math.sin(i * YAW_RES)) > 6 \
                or abs(r[i]) < 0.03:
            r[i] = 0       # dismiss strange data
    
    ox = [r[i] * math.cos(i * YAW_RES) for i in range(len(r))]
    oy = [r[i] * math.sin(i * YAW_RES) for i in range(len(r))]
    pmap = np.zeros(shape=(LEN_Y, LEN_X), dtype=np.uint8)

    for (x, y) in zip(ox, oy):

        d = math.sqrt(x**2 + y**2)
        if d == 0:
            continue  # if d==0 just dismiss
        angle = atan_zero_to_twopi(y, x)
        angle_id = int(math.floor(angle / YAW_RES))
        grid_list = precast[angle_id]

        ix = int(round((x - MIN_X) / XY_RES))   
        iy = int(round((y - MIN_Y) / XY_RES))
        # TODO: fix so that y = 200 does not occur
        #for grid in grid_list:
            #if grid.d > d:
                #pmap[min(int(LEN_Y - grid.iy), 199)][min(int(grid.ix), 199)] = 255        # unknown area    
        pmap[min(int(LEN_Y - iy), LEN_Y - 1)][min(int(ix), LEN_X - 1)] = 127  # obstacles #added min operation for handling indexerror
        #except IndexError:
         #   rospy.loginfo("Error with (ix, iy) = (%s, %s)" % (ix, iy))
        #kernel_dil = np.ones((2, 2), np.uint8)
        #kernel_erode = np.ones((3, 3), np.uint8)
        #pmap = cv2.dilate(pmap, kernel_dil, iterations=1)
        #pmap = cv2.erode(pmap, kernel_erode, iterations=1)
    return pmap

precast = precasting()

if __name__ == "__main__":
    global isactive
    '''
    code for activate and deactivate the node
    '''
    nodename = 'lidar_subscriber'
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
    rospy.init_node(nodename, anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    lidar_map_pub = rospy.Publisher('/occupancy_map', Image, queue_size=1)
    rospy.spin()
