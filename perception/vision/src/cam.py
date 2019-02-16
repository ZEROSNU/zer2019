#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os

H_front = np.array([[ 3.10164638e-02, -2.71691454e-01,  5.90104491e+02],
 [ 8.87632169e-01,  2.18687196e+00,  1.20801218e+01],
 [ 1.04434533e-05,  7.26795454e-03,  1.00000000e+00]]
)

H_left = np.array([[ 4.73842015e-01, -4.65504938e-01,  3.96031931e+02],
 [ 8.77530407e-01,  2.06530134e+00, -3.14076391e+02],
 [ 3.09604396e-04,  6.04023151e-03,  1.00000000e+00]]
)

H_right = np.array([[ 4.73842015e-01, -4.65504938e-01,  3.96031931e+02],
 [ 8.77530407e-01,  2.06530134e+00, -3.14076391e+02],
 [ 3.09604396e-04,  6.04023151e-03,  1.00000000e+00]]
)

Z_DEBUG = True

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
    rate=rospy.Rate(30)#30hz
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
        if Z_DEBUG:
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
            cam_front = cv2.VideoCapture(2)
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
