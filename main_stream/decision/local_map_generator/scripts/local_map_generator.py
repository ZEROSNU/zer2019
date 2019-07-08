#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import tf_conversions
from sensor_msgs.msg import Image
from core_msgs.msg import MotionState
from core_msgs.msg import VelocityLevel
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

def mainloop():
    rospy.Subscriber("/motion_state", MotionState, mcb)
    rospy.Subscriber("/raw_local_map", Image, rlmcb)
    pubgoal = rospy.Publisher('/goal_pose', PoseStamped, queue_size = 10)
    pubmap = rospy.Publisher('/local_map', OccupancyGrid, queue_size = 10)
    pubtf = rospy.Publisher('/tf_information', TFMessage, queue_size = 10)
    pubvel = rospy.Publisher('/velocity_level', VelocityLevel, queue_size = 10)
    rospy.init_node('local_map_generator', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    goal = PoseStamped()
    goal.header.frame_id = "car_frame"

    vellv = VelocityLevel()
    vellv.header.frame_id = "car_frame"

    qmap = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    simplemap = OccupancyGrid()
    simplemap.header.frame_id = "car_frame"
    simplemap.info.resolution = 0.03 #0.5
    simplemap.info.width = 200 #100
    simplemap.info.height = 200 #100
    simplemap.info.origin.position.x = -3 #-25
    simplemap.info.origin.position.y = -3 #-25
    simplemap.info.origin.orientation.x = qmap[0]
    simplemap.info.origin.orientation.y = qmap[1]
    simplemap.info.origin.orientation.z = qmap[2]
    simplemap.info.origin.orientation.w = qmap[3]
    simplemap.data = makemap(simplemap.info.height, simplemap.info.width)


    qframe = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    simpletf = TFMessage()
    simpletf.transforms.append(TransformStamped())
    simpletf.transforms[0].header.frame_id = "global_ref"
    simpletf.transforms[0].child_frame_id = "car_frame"
    simpletf.transforms[0].transform.rotation.x = qframe[0]
    simpletf.transforms[0].transform.rotation.y = qframe[1]
    simpletf.transforms[0].transform.rotation.z = qframe[2]
    simpletf.transforms[0].transform.rotation.w = qframe[3]
    simpletf.transforms[0].transform.translation.x = 0
    simpletf.transforms[0].transform.translation.y = 0
    i=0
    while not rospy.is_shutdown():
        simplemap.header.stamp = rospy.Time.now()
        simplemap.header.seq = i
        goal.header.stamp = rospy.Time.now()
        goal.header.seq = i
        vellv.header.stamp = rospy.Time.now()
        vellv.header.seq = i
        i = i+1
        pubgoal.publish(goal)
        pubmap.publish(simplemap)
        pubtf.publish(simpletf)
        pubvel.publish(vellv)
        rate.sleep()

def mcb(data) :
    print ("got motion state")
    return 0
    
def rlmcb(data) :
    print ("got raw local map state")
    return 0

def makemap(height, width) :
    zrs = np.zeros([height, width])
    for i in range(height) :
        zrs[i][-1] = 1
    zrs[height/2 : -1]=1
    for i in range(height/4) :
        for j in range(height/4) :
            zrs[i+height/8][j+height*3/8] = 1
    print zrs
    return zrs.flatten()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass