#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import ImuSpeed

def mainloop():
    pub = rospy.Publisher('/imu_speed', ImuSpeed, queue_size = 10)
    rospy.init_node('imu_encoder', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    imu = ImuSpeed()
    i=0
    while not rospy.is_shutdown():
        imu.header.stamp = rospy.Time.now()
        imu.header.seq = i
        i = i+1
        imu.header.frame_id = 'imu'
        pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass