#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from core_msgs.msg import LightState
from core_msgs.msg import ActiveNode
from sensor_msgs.msg import Image

count = 0 # count for late update 30Hz -> 10Hz

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'traffic_light'
    mainloop.active = True

    light = LightState()

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
    pub = rospy.Publisher('/light_state', LightState, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    i=0
    while not rospy.is_shutdown():
        light.header.stamp = rospy.Time.now()
        light.header.seq = i
        i = i+1
        light.header.frame_id = 'traffic_light'
        if mainloop.active :
            pub.publish(light)
        rate.sleep()

def callback(data):
    global light
    
    bgr = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    
    rgb = np.zeros(bgr.shape)
    rgb[:,:,0] = bgr[:,:,2]
    rgb[:,:,1] = bgr[:,:,1]
    rgb[:,:,2] = bgr[:,:,0]

    tl_path = '/home/kimsangmin/ZERO_VISION/PyTorch-YOLOv3/output/tl.txt'
    tl_img_path = '/home/kimsangmin/catkin_ws/src/zer2019/core/zero_monitor/data/traffic/test.jpg'
    
    cv2.imwrite(tl_img_path,rgb)

    with open(tl_path,'r') as f:
        state = f.read()
    if state == 'no_signal':
        light.light_found = False
        light.red = False
        light.yellow = False
        light.green = False
        light.left = False
    elif state == 'red':
        light.light_found = True
        light.red = True
        light.yellow = False
        light.green = False
        light.left = False
    elif state == 'yellow':
        light.light_found = True
        light.red = False
        light.yellow = True
        light.green = False
        light.left = False
    elif state == 'green':
        light.light_found = True
        light.red = False
        light.yellow = False
        light.green = True
        light.left = False
    elif state == 'red_left':
        light.light_found = True
        light.red = True
        light.yellow = False
        light.green = False
        light.left = True
    elif state == 'green_left':
        light.light_found = True
        light.red = False
        light.yellow = False
        light.green = True
        light.left = True
    

    
    
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass