#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import VelocityLevel
from nav_msgs.msg import Path
from core_msgs.msg import Curvature
from core_msgs.msg import ActiveNode #add

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'path_tracker'
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
    rospy.Subscriber("/velocity_level", VelocityLevel, vcb)
    rospy.Subscriber("/path", Path, pcb)
    pub = rospy.Publisher('/curvature', Curvature, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    curv = Curvature()
    curv.header.frame_id = 'car_frame'
    i=0
    while not rospy.is_shutdown():
        curv.header.stamp = rospy.Time.now()
        curv.header.seq = i
        i = i+1
        if mainloop.active :
            pub.publish(curv)
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