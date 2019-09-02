#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from core_msgs.msg import ActiveNode


class FakeCam:
    def __init__(self):
        self.node_name = 'fake_cam'
        self.pub_raw_image = rospy.Publisher('/forward_camera/raw_img', Image, queue_size=1)
        self.active = True
        self.count = 0
    '''
    def signalResponse(self, data):
        if 'zero_monitor' in data.active_nodes:
            if self.node_name in data.active_nodes:
                self.active = True
            else:
                self.active = False
        else:
            rospy.signal_shutdown('no monitor')
    '''
    def send_image(self):
        rate = rospy.Rate(20)
        path = rospy.get_param('/data_path') + 'bird/'
        #path = '/home/kimsangmin/ZERO_VISION/bird/3/'
        ext = '.jpg'
        num_of_files = 600
        if self.count < num_of_files:
            full_path = path + str(self.count) + ext
            self.count += 1
        else:
            full_path = path + str(self.count) + ext
            self.count = 0
        img = cv2.imread(full_path)
        bridge = CvBridge()
        img_msg = bridge.cv2_to_imgmsg(img, 'bgr8')

        self.pub_raw_image.publish(img_msg)
        rate.sleep()

def main():
    rospy.init_node('fake_cam', anonymous=True)
    fc = FakeCam()

    while not rospy.is_shutdown():
        try:
            fc.send_image()
        except rospy.ROSInterruptException:
            pass
    

if __name__ == '__main__':
    main()
