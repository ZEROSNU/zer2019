#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import ImuSpeed
from core_msgs.msg import Control
from core_msgs.msg import ActiveNode #add

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'model_estimator'
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
    rospy.Subscriber("/imu_speed", ImuSpeed, icb)
    rospy.Subscriber("/ideal_control", Control, ccb)
    pub = rospy.Publisher('/calibrated_control', Control, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    control = Control()
    control.header.frame_id = 'car_frame'
    i=0
    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.header.seq = i
        i = i+1
        if mainloop.active:
            pub.publish(control)
        rate.sleep()

def icb(data) :
    print ("got imuspeed")
    return 0
    
def ccb(data) :
    print ("got control")
    return 0



if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass