#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import ImuSpeed
from core_msgs.msg import Control

def mainloop():
    rospy.Subscriber("/imu_speed", ImuSpeed, icb)
    rospy.Subscriber("/ideal_control", Control, ccb)
    pub = rospy.Publisher('/calibrated_control', Control, queue_size = 10)
    rospy.init_node('model_estimator', anonymous=True)
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