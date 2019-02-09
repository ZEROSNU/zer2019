#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def send_image():
    pub = rospy.Publisher('raw_img', Image, queue_size=10)
    rospy.init_node('cam_node', anonymous=True)
    rate = rospy.Rate(10) # N hz

    while not rospy.is_shutdown():
        #Dataset Path
        global count
        path = '/home/ksg/zero/lane/data/lane1/'
        ext = '.jpg'
        num_of_files = 2000
        if count < num_of_files:
            full_path = path + str(count) + ext
            count = count + 1
        else:
            full_path = path + str(count) + ext
            count = 1000

        img = cv2.imread(full_path)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')

        pub.publish(img_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo('Initiate fake_cam node')
        count = 1000
        send_image()
    except rospy.ROSInterruptException:
        pass