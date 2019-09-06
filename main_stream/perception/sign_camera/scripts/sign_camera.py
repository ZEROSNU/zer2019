#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState
from core_msgs.msg import ActiveNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

task = MissionState()

def mainloop():
    global task
    '''
    code for activate and deactivate the node
    '''
    nodename = 'sign_camera'
    mainloop.active = True
    def signalResponse(data) :
        mainloop.active
        if 'zero_monitor' in data.active_nodes :
            if nodename in data.active_nodes :
                mainloop.active = True
            else :
                mainloop.active = False
        else :
            rospy.signal_shutdown('no monitor')
    rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)
    rospy.Subscriber('/traffic_image', Image, callback)
    '''
    ...
    '''
    pub = rospy.Publisher('/task', MissionState, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(3) # 10hz
    i=0
    while not rospy.is_shutdown():
        task.header.stamp = rospy.Time.now()
        task.header.seq = i
        i = i+1
        task.header.frame_id = 'sign_camera'
        if mainloop.active :
            pub.publish(task)
        rate.sleep()

def callback(data):
    global task
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(data, 'bgr8')
    '''
    rgb = np.zeros(bgr.shape)
    rgb[:,:,0] = bgr[:,:,2]
    rgb[:,:,1] = bgr[:,:,1]
    rgb[:,:,2] = bgr[:,:,0]
    '''
    sign_path = '/home/snuzero/sign_classification/output/sign.txt'
    sign_img_path = '/home/snuzero/catkin_ws/src/zer2019/core/zero_monitor/data/sign/test.jpg'
    
    cv2.imwrite(sign_img_path,bgr)
    try:
        with open(sign_path,'r') as f:
            state = f.read()
        if state == "no_sign":
            task.mission_state = "DRIVING_SECTION"
        elif state == "forward":
            task.mission_state = "DRIVING_SECTION"
        elif state == "turn_left":
            task.mission_state = "INTERSECTION_LEFT"
        elif state == "turn_right":
            task.mission_state = "INTERSECTION_RIGHT"
        elif state == "slow":
            task.mission_state = "SCHOOL_ZONE"
        elif state == "crosswalk":
            task.mission_state = "CROSSWALK"
        elif state == "static_obstacle":
            task.mission_state = "OBSTACLE_STATIC"
        elif state == "dynamic_obstacle":
            task.mission_state = "OBSTACLE_SUDDEN"
        elif state == "parking":
            task.mission_state = "DRIVING_SECTION"
    except:
        pass
        
        

if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass