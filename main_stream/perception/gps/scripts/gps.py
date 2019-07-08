#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState

def mainloop():
    pub = rospy.Publisher('/mission_state', MissionState, queue_size = 10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    mission = MissionState()
    i=0
    while not rospy.is_shutdown():
        mission.header.stamp = rospy.Time.now()
        mission.header.seq = i
        i = i+1
        mission.header.frame_id = 'gps'
        pub.publish(mission)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass