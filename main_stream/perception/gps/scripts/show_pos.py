#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
from core_msgs.msg import Location
from geometry_msgs.msg import PoseStamped

pose = PoseStamped()
first = [ 126.773048401, 37.2390632629]
def mainloop():
    pub = rospy.Publisher('/gps_pose', PoseStamped, queue_size=10)
    rospy.init_node('gps2map', anonymous=True)
    rospy.Subscriber('/Location_msg', Location, cb)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(pose)
        drawmap(pose)
        rate.sleep()

def drawmap(pose) :
    
    dir = rospy.get_param('/dir')
    file_name = 'secondpath.jpg'
    img = cv2.imread(dir+file_name)
    cv2.circle( img, (int(pose.pose.position.x) + 256, int(pose.pose.position.y)), 3, (255,0,0), -1 )
    
    cv2.imshow('path',img)
    cv2.waitKey(1)

def cb(data) :
    global first
    global path
    global pose
    dx = 0.00001
    dy = 0.00001
    pose.pose.position.x = (data.Long - first[0])/dx
    pose.pose.position.y = (data.Lat - first[1])/dy
    pose.pose.orientation.w = 1

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass