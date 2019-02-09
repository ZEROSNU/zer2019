#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def talker():
    pub = rospy.Publisher('zero_img', Image, queue_size=10)
    rospy.init_node('image_sender', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    img = cv2.imread('/home/ksg/zero/camera/1.png')
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')


    while not rospy.is_shutdown():
        pub.publish(img_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        
        talker()
    except rospy.ROSInterruptException:
        pass