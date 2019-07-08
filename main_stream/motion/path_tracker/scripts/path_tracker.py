#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import VelocityLevel
from nav_msgs.msg import Path
from core_msgs.msg import Control

def mainloop():
    rospy.Subscriber("/velocity_level", VelocityLevel, vcb)
    rospy.Subscriber("/path", Path, pcb)
    pub = rospy.Publisher('/ideal_control', Control, queue_size = 10)
    rospy.init_node('path_tracker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    control = Control()
    control.header.frame_id = 'car_frame'
    i=0
    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.header.seq = i
        i = i+1
        pub.publish(control)
        rate.sleep()

def vcb(data) :
    print ("got velocity level")
    return 0
    
def pcb(data) :
    print ("got path")
    return 0



if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass