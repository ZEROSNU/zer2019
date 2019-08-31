#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os
'''
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
'''

pub_mod = "warp" # "raw"

H_front = np.array([[ 7.24463464e-03, -3.07926267e-01,  7.93327596e+02],
 [ 1.18294789e+00,  2.68545668e+00, -8.01974499e+01],
 [ 2.00098970e-04,  9.09630505e-03,  1.00000000e+00]]


)

H_left = np.array([[ 1.51829177e+00 ,-3.56086666e-02 , 1.02628015e+03],
 [ 2.65189371e+00,  7.76363177e+00, -1.39756446e+03],
 [ 1.33740329e-03,  2.56790131e-02,  1.00000000e+00]]
)

H_right = np.array([[-3.46869877e-01, -2.34105692e-01,  5.37533684e+02],
 [ 8.78637251e-01 , 1.09143533e+00,  3.45392159e+02],
 [-5.79562630e-05 , 5.01844520e-03 , 1.00000000e+00]]

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
    #left_pub = rospy.Publisher('left_img', Image, queue_size=1)
    #front_pub = rospy.Publisher('front_img', Image, queue_size=1)
    #right_pub = rospy.Publisher('right_img', Image, queue_size=1)
    
    traffic_pub = rospy.Publisher('/traffic_image', Image, queue_size=1)
    rospy.init_node('cam', anonymous=True)
    rate=rospy.Rate(30)#30hz
    bridge = CvBridge()

    traffic_count = 0

    while not rospy.is_shutdown():
        _, img_front = cam_front.read() # captures image
        _, img_left = cam_left.read()
        _, img_right = cam_right.read()
        _, img_wide = cam_wide.read()
        
        wrp_front = warp_image(img_front, H_front)
        wrp_left = warp_image(img_left, H_left)
        wrp_right = warp_image(img_right, H_right)

        non_black_area, black_area = find_mask(wrp_front)
        front_masked = np.multiply(wrp_front, non_black_area).astype('uint8')
        left_masked = np.multiply(wrp_left, black_area).astype('uint8')
        right_masked = np.multiply(wrp_right, black_area).astype('uint8')
        
        merged =  front_masked + right_masked + left_masked

        merged_msg = bridge.cv2_to_imgmsg(merged,'bgr8')
        
        traffic_msg = bridge.cv2_to_imgmsg(img_wide,'bgr8')
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
        # PUBLISH merged msg as 30Hz
        lane_pub.publish(merged_msg)

        # PUBLISH traffic msg as 10Hz
        if traffic_count <= 0:
            traffic_pub.publish(traffic_msg)
            traffic_count = 2
        else:
            traffic_count -= 1
            
        #left_pub.publish(left_msg)
        #front_msg = bridge.cv2_to_imgmsg(img_front, 'bgr8')
        #front_pub.publish(front_msg)
        #right_pub.publish(right_msg)

        if Z_DEBUG:
            cv2.imshow("result", merged)
            cv2.imshow("wide", img_wide)
            #cv2.imshow('result', front_masked)
            if cv2.waitKey(1)==27:
                break
    cv2.destroyAllWindows()

    cam_front.release()
    cam_left.release()
    cam_right.release()
    cam_wide.release()


if __name__ == '__main__':
    FRAME = 10
    try:
        # Node to obtain call camera data. Separate I/O pipeline
        rospy.loginfo('Init Cameras...')
        while True:
            cam_front = cv2.VideoCapture(2)
            cam_left = cv2.VideoCapture(1)
            cam_right = cv2.VideoCapture(4)
            cam_wide = cv2.VideoCapture(3)
            cam_front.set(cv2.CAP_PROP_FPS, FRAME)
            cam_left.set(cv2.CAP_PROP_FPS, FRAME)
            cam_right.set(cv2.CAP_PROP_FPS, FRAME)
            cam_wide.set(cv2.CAP_PROP_FPS, FRAME)
            cv2.CAP_PROP_FPS

            '''
            cam_front.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            cam_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam_front.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))

            
            cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, 864)
            cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam_left.set(cv2.CAP_PROP_FOURCC, int(0x47504A4D))
            

            
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
            _right, img_right = cam_right.read()

            _wide, img_wide = cam_wide.read()

            #if (ret1 and ret2 and ret3):
            if(_front and _right and _left and _wide):
                print("All cameras connected!")
                break
            else:
                print("Connection error! retrying...")
                cam_front.release()
                cam_left.release()
                cam_right.release()
                cam_wide.release()
    
        imagePublisher()
    except rospy.ROSInterruptException:
        pass
