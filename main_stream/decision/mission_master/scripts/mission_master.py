#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState
from core_msgs.msg import LightState
from core_msgs.msg import Task
from core_msgs.msg import MotionState
from core_msgs.msg import ActiveNode

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'mission_master'
    mainloop.active = True
    def signalResponse(data) :
        mainloop.active
        if 'zero_monitor' in data.active_nodes :
            if nodename in data.active_nodes :
                mainloop.active = True
            else :
                mainloop.active = False
        else :
            rospy.signal_shutdown('no monitor')
    rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)
    '''
    ...
    '''
    rospy.Subscriber("/mission_state", MissionState, mscb)
    rospy.Subscriber("/light_state", LightState, lscb)
    rospy.Subscriber("/task", Task, tcb)
    pub = rospy.Publisher('/motion_state', MotionState, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    motion = MotionState()
    motion.header.frame_id = 'gps'
    i=0
    while not rospy.is_shutdown():
        motion.header.stamp = rospy.Time.now()
        motion.header.seq = i
        i = i+1
        if mainloop.active :
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