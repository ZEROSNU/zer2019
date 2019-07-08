#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState
from core_msgs.msg import LightState
from core_msgs.msg import Task
from core_msgs.msg import MotionState

def mainloop():
    rospy.Subscriber("/mission_state", MissionState, mscb)
    rospy.Subscriber("/light_state", LightState, lscb)
    rospy.Subscriber("/task", Task, tcb)
    pub = rospy.Publisher('/motion_state', MotionState, queue_size = 10)
    rospy.init_node('mission_master', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    motion = MotionState()
    motion.header.frame_id = 'gps'
    i=0
    while not rospy.is_shutdown():
        motion.header.stamp = rospy.Time.now()
        motion.header.seq = i
        i = i+1
        pub.publish(motion)
        rate.sleep()

def mscb(data) :
    print ("got mission state")
    return 0
    
def lscb(data) :
    print ("got light state")
    return 0

def tcb(data) :
    print ("got task")
    return 0


if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass