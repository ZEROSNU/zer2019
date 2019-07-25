#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState
from core_msgs.msg import ActiveNode #add

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'gps'
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
    pub = rospy.Publisher('/mission_state', MissionState, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    mission = MissionState()
    i=0
    while not rospy.is_shutdown():
        mission.header.stamp = rospy.Time.now()
        mission.header.seq = i
        i = i+1
        mission.header.frame_id = 'gps'
        if mainloop.active :
            pub.publish(mission)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass