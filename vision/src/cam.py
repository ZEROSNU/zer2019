#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os

H_front = np.array([[-2.21600611e-01, -4.02884371e-01,  1.05711505e+03],
 [ 1.31734847e+00,  2.48484602e+00, -3.46819684e+01],
 [-2.82815151e-04,  7.94547169e-03,  1.00000000e+00]])

H_left = np.array([[ 1.43015382e-01, -2.11501611e-01,  8.13747195e+02],
 [ 1.23578247e+00,  2.37317155e+00, -2.27820899e+02],
 [-8.49898466e-05,  7.68283611e-03,  1.00000000e+00]])

def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (600, 600))
    return im_out


def find_mask(image):
    black_range = np.array([0,0,0])
    im_mask = (cv2.inRange(image, black_range, black_range)).astype('bool')
    im_mask_inv = (1-im_mask).astype('bool')
    im_mask_inv = np.dstack((im_mask_inv, im_mask_inv, im_mask_inv))
    im_mask= np.dstack((im_mask, im_mask, im_mask))
    return im_mask_inv, im_mask

def imagePublisher():
    lane_pub = rospy.Publisher('raw_img', Image, queue_size=1)
    traffic_pub = rospy.Publisher('traffic_image', Image, queue_size=1)
    rospy.init_node('cam_node', anonymous=True)
    rate=rospy.Rate(20)#20hz
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, img_front = cam_front.read() # captures image
        _, img_left = cam_left.read()
        
        wrp_front = warp_image(img_front, H_front)
        wrp_left = warp_image(img_left, H_left)

        non_black_area, black_area = find_mask(wrp_front)
        front_masked = np.multiply(wrp_front, non_black_area).astype('uint8')
        left_masked = np.multiply(wrp_left, black_area).astype('uint8')
        merged =  front_masked + left_masked

        merged_msg = bridge.cv2_to_imgmsg(merged,'bgr8')

        # PUBLISH
        lane_pub.publish(merged_msg)

        cv2.imshow("result", merged)
        if cv2.waitKey(1)==27:
            break
    cv2.destroyAllWindows()

    cam_front.release()
    cam_left.release()


if __name__ == '__main__':
    try:
        # Node to obtain call camera data. Separate I/O pipeline
        rospy.loginfo('Init Cameras...')
        while True:
            cam_front = cv2.VideoCapture(0)
            cam_left = cv2.VideoCapture(1)
            
            cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

            cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

            '''
            cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam_right.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
            traffic_cam = cv2.VideoCapture(5)
            traffic_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            traffic_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            traffic_cam.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
            '''
            _front, img_front = cam_front.read() # captures image
            _left, img_left = cam_left.read()

            #if (ret1 and ret2 and ret3):
            if(_front and _left):
                print("All cameras connected!")
                break
            else:
                print("Connection error! retrying...")
                cam_front.release()
                cam_left.release()
    
        imagePublisher()
    except rospy.ROSInterruptException:
        pass