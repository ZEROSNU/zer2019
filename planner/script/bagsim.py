#!/usr/bin/env python

import rospy
import numpy as np
import tf_conversions
import tf2_ros
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid

class bagsim :
    def __init__ (self):
        self.seq = -1
        self.access = False
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.simpletf = TFMessage()
        self.simpletf.transforms.append(TransformStamped())
        self.simpletf.transforms[0].header.frame_id = "/map"
        self.simpletf.transforms[0].child_frame_id = "/car_frame"
        self.simpletf.transforms[0].transform.rotation.x = q[0]
        self.simpletf.transforms[0].transform.rotation.y = q[1]
        self.simpletf.transforms[0].transform.rotation.z = q[2]
        self.simpletf.transforms[0].transform.rotation.w = q[3]

        self.simplemap = OccupancyGrid()
        self.simplemap.header.frame_id = "/car_frame"
        self.simplemap.info.origin.orientation.x = q[0]
        self.simplemap.info.origin.orientation.y = q[1]
        self.simplemap.info.origin.orientation.z = q[2]
        self.simplemap.info.origin.orientation.w = q[3]


        qinit = tf_conversions.transformations.quaternion_from_euler(0, 0, np.pi/2)
        self.simpleinitial = PoseWithCovarianceStamped()
        self.simpleinitial.header.frame_id = "/car_frame"
        self.simpleinitial.pose.pose.position.y = 4
        self.simpleinitial.pose.pose.orientation.x = qinit[0]
        self.simpleinitial.pose.pose.orientation.y = qinit[1]
        self.simpleinitial.pose.pose.orientation.z = qinit[2]
        self.simpleinitial.pose.pose.orientation.w = qinit[3]

        qgoal = tf_conversions.transformations.quaternion_from_euler(0, 0, np.pi/4)
        self.simplegoal = PoseStamped()
        self.simplegoal.header.frame_id = "/car_frame"
        self.simplegoal.pose.position.y = 16
        self.simplegoal.pose.orientation.x = qgoal[0]
        self.simplegoal.pose.orientation.y = qgoal[1]
        self.simplegoal.pose.orientation.z = qgoal[2]
        self.simplegoal.pose.orientation.w = qgoal[3]


    def mapCB(self, data) :
        while(self.access) :
            continue
        self.access = True
        self.simplemap.header.stamp = rospy.Time.now()
        self.simplemap.header.seq = self.seq
        self.simplemap.info.map_load_time = rospy.Time.now()

        self.simpletf.transforms[0].header.stamp = rospy.Time.now()
        self.simpletf.transforms[0].header.seq = self.seq

        self.simpleinitial.header.stamp = rospy.Time.now()
        self.simpleinitial.header.seq = self.seq

        self.simplegoal.header.stamp = rospy.Time.now()
        self.simplegoal.header.seq = self.seq


        self.simplemap.header.frame_id = data.header.frame_id
        self.simplemap.info.resolution = data.info.resolution
        self.simplemap.info.width = data.info.width
        self.simplemap.info.height = data.info.height
        self.simplemap.info.origin.position.x = -data.info.width/2*data.info.resolution
        self.simpletf.transforms[0].child_frame_id = data.header.frame_id

        self.simplemap.data = np.flip(np.resize(data.data, (data.info.height, data.info.width)), 0).flatten()
        self.seq = self.seq + 1
        self.access = False

myplayer = bagsim()

def mapCB(occ_map) :
    print "get_map"
    myplayer.mapCB(occ_map)

def main():
    rospy.init_node('fakemap', anonymous=False)
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
    pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_goal = rospy.Publisher('/goalpose', PoseStamped, queue_size=10)
    rospy.Subscriber("/occ_map", OccupancyGrid, mapCB)
    seq = myplayer.seq
    while not rospy.is_shutdown():
        if seq < myplayer.seq :
            while(myplayer.access) :
                continue
            myplayer.access = True
            pub_map.publish(myplayer.simplemap)
            pub_tf.publish(myplayer.simpletf)
            pub_init.publish(myplayer.simpleinitial)
            pub_goal.publish(myplayer.simplegoal)
            seq = seq + 1
            print "publshing data"
            myplayer.access = False



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass