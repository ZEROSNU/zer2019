#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def imagePublisher():
    traffic_pub = rospy.Publisher('/traffic_image', Image, queue_size=1)
    rospy.init_node('cam', anonymous=True)
    rate=rospy.Rate(30)#30hz
    bridge = CvBridge()

    traffic_count = 0

    while not rospy.is_shutdown():
        _, img_front = cam_front.read() # captures image
        
        traffic_msg = bridge.cv2_to_imgmsg(img_front,'bgr8')
        '''
        if pub_mod == "warp":
            left_msg = bridge.cv2_to_imgmsg(wrp_left, 'bgr8')
            front_msg = bridge.cv2_to_imgmsg(wrp_front, 'bgr8')
            right_msg = bridge.cv2_to_imgmsg(wrp_right, 'bgr8')
        elif pub_mod == "raw":
            left_msg = bridge.cv2_to_imgmsg(img_left, 'bgr8')
            front_msg = bridge.cv2_to_imgmsg(img_front, 'bgr8')
            right_msg = bridge.cv2_to_imgmsg(img_right, 'bgr8')
        '''
        # PUBLISH traffic msg as 10Hz
        if traffic_count <= 0:
            traffic_pub.publish(traffic_msg)
            traffic_count = 2
        else:
            traffic_count -= 1
if __name__ == '__main__':
    try:
        # Node to obtain call camera data. Separate I/O pipeline
        rospy.loginfo('Init Cameras...')
        while True:
            cam_front = cv2.VideoCapture(1)


            _front, img_front = cam_front.read() 
            #if (ret1 and ret2 and ret3):
            if(_front):
                print("All cameras connected!")
                break
            else:
                print("Connection error! retrying...")
                cam_front.release()

    
        imagePublisher()
    except rospy.ROSInterruptException:
        pass