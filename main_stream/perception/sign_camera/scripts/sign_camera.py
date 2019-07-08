#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import Task

def mainloop():
    pub = rospy.Publisher('/task', Task, queue_size = 10)
    rospy.init_node('sign_camera', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    task = Task()
    i=0
    while not rospy.is_shutdown():
        task.header.stamp = rospy.Time.now()
        task.header.seq = i
        i = i+1
        task.header.frame_id = 'sign_camera'
        pub.publish(task)
        rate.sleep()

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass