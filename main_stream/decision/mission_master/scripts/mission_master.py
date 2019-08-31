#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import MissionState
from core_msgs.msg import LightState
from core_msgs.msg import MotionState
from core_msgs.msg import ActiveNode
from std_msgs.msg import String
import roslaunch
missionstate=MissionState()
lightstate=LightState()
taskstate=MissionState()
slam_mask = True
def mainloop():
    global slam_mask
    '''
    code for activate and deactivate the node
    '''
    nodename = 'mission_master'
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
    '''
    ...
    '''
    rospy.Subscriber("/mission_state", MissionState, mscb)
    rospy.Subscriber("/light_state", LightState, lscb)
    rospy.Subscriber("/task", MissionState, tcb)
    pub = rospy.Publisher('/motion_state', MotionState, queue_size = 10)
    #pub_reset_hector = rospy.Publisher('/syscommand', String, queue_size = 10) #syscommand
    rospy.init_node(nodename, anonymous=True)
    uuid=roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    slam_mask=True
    slam_launch=roslaunch.parent.ROSLaunchParent(uuid, ["home/snuzero/catkin_ws/src/zer2019/main_stream/perception/hector_slam/hector_slam_launch/tutorial.launch"])
    rate = rospy.Rate(10) # 10hz
    #sysco = 'reset' #syscommand string
    motion = MotionState()
    motion.header.frame_id = 'gps'
    i=0
    while not rospy.is_shutdown():
        motion.header.stamp = rospy.Time.now()
        motion.header.seq = i
        if taskstate.mission_state!="PARKING" or missionstate.mission_state!="PARKING":
            if slam_mask==False:
                slam_mask=True
                slam_launch.shutdown()
        if taskstate.mission_state=="NO_SIGN":
            if missionstate.mission_state=="DRIVING_SECTION":
                motion.motion_state="FORWARD_MOTION"
        elif taskstate.mission_state=="INTERSECTION_STRAIGHT":
            if missionstate.mission_state=="INTERSECTION_STRAIGHT":
                if lightstate.light_found:
                    if lightstate.red and not lightstate.yellow and not lightstate.green:
                        motion.motion_state="HALT"
                    elif not lightstate.red and lightstate.yellow and not lightstate.green:
                        motion.motion_state="HALT"
                    else:
                        motion.motion_state="FORWARD_MOTION"
                else:
                    motion.motion_state="FORWARD_MOTION"
        elif taskstate.mission_state=="INTERSECTION_LEFT" and missionstate.mission_state=="INTERSECTION_LEFT":
            if lightstate.light_found:
                if not lightstate.left:
                    motion.motion_state="HALT"
                else: 
                    motion.motion_state="LEFT_MOTION"
            else: 
                motion.motion_state="LEFT_MOTION"
        elif taskstate.mission_state=="INTERSECTION_RIGHT" and missionstate.mission_state=="INTERSECTION_RIGHT":
            if lightstate.light_found:
                motion.motion_state="RIGHT_MOTION"
            else:
                motion.motion_state="RIGHT_MOTION"
        elif taskstate.mission_state=="OBSTACLE_STATIC" and missionstate.mission_state=="OBSTACLE_STATIC":
            motion.motion_state="FORWARD_MOTION_SLOW"
        elif taskstate.mission_state=="OBSTACLE_DYNAMIC" and missionstate.mission_state=="DYNAMIC":
            motion.motion_state="FORWARD_MOTION_SLOW"
        elif taskstate.mission_state=="CROSSWALK" and missionstate.mission_state=="CROSSWALK":
            motion.motion_state="HALT"
        elif taskstate.mission_state=="SCHOOL_ZONE" and missionstate.mission_state=="SCHOOL_ZONE":
            motion.motion_state="FORWARD_MOTION_SLOW"
        elif taskstate.mission_state=="SPEED_BUST" and missionstate.mission_state=="SPEED_BUST":
            motion.motion_state="FORWARD_MOTION_SLOW"
        elif taskstate.mission_state=="PARKING" and missionstate.mission_state=="PARKING":
            motion.motion_state="PARKING"
            if slam_mask==True:
                slam_mask=False
                slam_launch.start()
        else:
            motion.motion_state="HALT"               

        i = i+1
        if mainloop.active :
            pub.publish(motion)
        rate.sleep()

def mscb(data) :
    print ("got mission state - <"+ data.mission_state+">\n")
    missionstate.mission_state=data.mission_state
    return 0
    
def lscb(data) :
    print ("got light state - <%d,[%d,%d,%d,%d]>\n " % (data.light_found, data.red, data.yellow, data.left,data.green))
    lightstate.light_found=data.light_found
    lightstate.red=data.red
    lightstate.yellow=data.yellow
    lightstate.left=data.left
    lightstate.green=data.green
    return 0
def tcb(data) :
    print ("got task - <"+ data.mission_state+">\n")
    taskstate.mission_state=data.mission_state
    return 0
if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass
