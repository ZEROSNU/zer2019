#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from core_msgs.msg import ImuSpeed
from core_msgs.msg import Curvature
from core_msgs.msg import VelocityLevel
from core_msgs.msg import Control
from core_msgs.msg import ActiveNode #add

steer = 0
velocity = 0

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
    rospy.Subscriber("/curvature", Curvature, ccb)
    rospy.Subscriber("/velocity_level", VelocityLevel, vcb)
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
        control.speed = velocity
        control.steer = steer
        if mainloop.active:
            pub.publish(control)
        rate.sleep()
def vcb(data) :
    velocity = data.velocity_level


def icb(data) :
    print ("got imuspeed")
    return 0
    
def ccb(data) :
    steer = np.arctan(1.3*data.curvature)



if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass