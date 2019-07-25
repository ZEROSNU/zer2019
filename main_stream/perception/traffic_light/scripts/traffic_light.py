#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import LightState
from core_msgs.msg import ActiveNode

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'traffic_light'
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
    pub = rospy.Publisher('/light_state', LightState, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    light = LightState()
    i=0
    while not rospy.is_shutdown():
        light.header.stamp = rospy.Time.now()
        light.header.seq = i
        i = i+1
        light.header.frame_id = 'traffic_light'
        if mainloop.active :
            pub.publish(light)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass