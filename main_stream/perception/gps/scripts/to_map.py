#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import Location
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = Path()
isinit = False
first = []
def mainloop():
    global path
    path.poses = []
    path.header.frame_id = 'car_frame'
    pub = rospy.Publisher('gpspath', Path, queue_size=10)
    rospy.init_node('gps2map', anonymous=True)
    rospy.Subscriber('/Location_msg', Location, cb)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(path)
        rate.sleep()

def cb(data) :
    global isinit
    global first
    global path
    dx = 0.0001
    dy = 0.0001
    if not isinit :
        isinit = True
        first = [data.Long, data.Lat]
    pose = PoseStamped()
    pose.pose.position.x = (data.Long - first[0])/dx
    pose.pose.position.y = (data.Lat - first[1])/dy
    pose.pose.orientation.w = 1
    path.poses.append(pose)

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass