#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import ImuSpeed
from core_msgs.msg import ActiveNode #add

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'imu_encoder'
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
    pub = rospy.Publisher('/imu_speed', ImuSpeed, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10)
    imu = ImuSpeed()
    i=0
    while not rospy.is_shutdown():
        imu.header.stamp = rospy.Time.now()
        imu.header.seq = i
        i = i+1
        imu.header.frame_id = 'imu'
        if mainloop.active: #add
            pub.publish(imu)
        rate.sleep()


if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass