#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import LightState

def mainloop():
    pub = rospy.Publisher('/light_state', LightState, queue_size = 10)
    rospy.init_node('traffic_light', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    light = LightState()
    i=0
    while not rospy.is_shutdown():
        light.header.stamp = rospy.Time.now()
        light.header.seq = i
        i = i+1
        light.header.frame_id = 'traffic_light'
        pub.publish(light)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass