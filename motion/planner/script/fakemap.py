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

class fakemap :
    def __init__ (self):
        self.seq = -1
        self.initseq = -1
        self.goalseq = -1
        qframe = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.simpletf = TFMessage()
        self.simpletf.transforms.append(TransformStamped())
        self.simpletf.transforms[0].header.frame_id = "map"
        self.simpletf.transforms[0].child_frame_id = "car_frame"
        self.simpletf.transforms[0].transform.rotation.x = qframe[0]
        self.simpletf.transforms[0].transform.rotation.y = qframe[1]
        self.simpletf.transforms[0].transform.rotation.z = qframe[2]
        self.simpletf.transforms[0].transform.rotation.w = qframe[3]
        self.simpletf.transforms[0].transform.translation.x = 0
        self.simpletf.transforms[0].transform.translation.y = 3

        qmap = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        self.simplemap = OccupancyGrid()
        self.simplemap.header.frame_id = "car_frame"
        self.simplemap.info.resolution = 0.03
        self.simplemap.info.width = 200
        self.simplemap.info.height = 200
        self.simplemap.info.origin.position.x = -3
        self.simplemap.info.origin.orientation.x = qmap[0]
        self.simplemap.info.origin.orientation.y = qmap[1]
        self.simplemap.info.origin.orientation.z = qmap[2]
        self.simplemap.info.origin.orientation.w = qmap[3]
        self.simplemap.data = self.makemap(self.simplemap.info.height, self.simplemap.info.width)
    def makemap(self, height, width) :
        zrs = np.zeros([height, width])
        for i in range(height) :
            zrs[i][-1] = 1
        zrs[height/2 : -1]=1
        print zrs
        return zrs.flatten()

    def getgoal(self, data) :
        self.goalseq = data.header.seq
        print "get goal #" + str(self.goalseq)
        print data.pose.position
    def getinit(self, data) :
        self.initseq = data.header.seq
        print "get init #" + str(self.initseq)
        print data.pose.pose.position
    def checkpub(self) :
        if self.seq < self.initseq and self.seq < self.goalseq :
            if self.initseq < self.goalseq :
                self.seq = self.initseq
            else :
                self.seq = self.goalseq
            self.simplemap.header.stamp = rospy.Time.now()
            self.simplemap.header.seq = self.seq
            self.simplemap.info.map_load_time = rospy.Time.now()

            self.simpletf.transforms[0].header.stamp = rospy.Time.now()
            self.simpletf.transforms[0].header.seq = self.seq
            return True
        else :
            return False

myfm = fakemap()

def goalCB(data) :
    myfm.getgoal(data)

def initCB(data) :
    myfm.getinit(data)



def main():
    rospy.init_node('fakemap', anonymous=False)
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=10)
    pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initCB)
    rospy.Subscriber("/goalpose", PoseStamped, goalCB)
    while not rospy.is_shutdown() :
        if myfm.checkpub() :
            pub_tf.publish(myfm.simpletf)
            pub_map.publish(myfm.simplemap)
            print "tf and map published"
    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass