#!/usr/bin/env python
# license removed for brevity
import rospy
from core_msgs.msg import ImuSpeed
from core_msgs.msg import Control
from core_msgs.msg import ActiveNode
from core_msgs.msg import Curvature
from core_msgs.msg import VehicleState
from core_msgs.msg import MotionState

class model_estimator:
    # Initialize class variables
    def __init__(self):

        self.temp_control_mode = 'NO_CONTROL'

        # Output variables
        self.control = Control()

        # Manage for subscribe input
        self.latest_imuspeed = ImuSpeed()
        self.imuspeed = ImuSpeed()
        self.imuspeed_buff = []
        self.imuspeed_buff_size = 200
        self.imuspeed_time_buff = []
        self.update_imuspeed = False

        self.latest_curvature = Curvature()
        self.curvature = Curvature()
        self.curvature_buff = []
        self.curvature_buff_size = 200
        self.curvature_time_buff = []
        self.update_curvature = False

        self.latest_vehicle_state = VehicleState()
        self.vehicle_state = VehicleState()
        self.vehicle_state_buff = []
        self.vehicle_state_buff_size = 200
        self.vehicle_state_time_buff = []
        self.update_vehicle_state = False

        self.latest_motion_state = MotionState()
        self.motion_state = MotionState()
        self.update_motion_state = False
        self.motion_state_buff = []
        self.motion_state_buff_size = 200
        self.motion_state_time_buff = []

        self.latest_velocity_level = VelocityLevel()
        self.velocity_level = VelocityLevel()
        self.update_velocity_level = False
        self.velocity_level_buff = []
        self.velocity_level_buff_size = 200
        self.velocity_level_time_buff = []
        
      
    # Define functions
    def Set_temp_control_mode(self):
        #TODO
        temp_control_mode = 'NO_CONTROL'
        return temp_control_mode

    def Set_control_mode(self):
        #TODO
        control_mode = 'NO_CONTROL'
        return control_mode

    def Curvature2Steer(self):
        #TODO
        steer = 0.
        return steer

    def Set_gear(self):
        #TODO
        gear = 0
        return gear

    def Set_speed_and_brake(self):
        #TODO
        speed = 0.
        brake = 0
        return speed, brake

    def Set_isauto_and_estop(self):
        #TODO
        is_auto = False
        estop = False
        return is_auto, estop


    # Define write functions
    def write_imuspeed(self, data):
        self.latest_imuspeed = data
        self.update_imuspeed = True
    
    def write_curvature(self, data):
        self.latest_curvature = data
        self.update_curvature = True

    def write_vehicle_state(self, data):
        self.latest_vehicle_state = data
        self.update_vehicle_state = True

    def write_motion_state(self, data):
        self.latest_motion_state = data
        self.update_motion_state = True

    def write_velocity_level(self, data):
        self.latest_velocity_level = data
        self.update_velocity_level = True


    # Define main estimate loop
    def main_estimate_loop(self):
        '''
        1. Initialization
        '''
        # Update imuspeed
        if self.update_imuspeed == True:
            self.imuspeed = copy.deepcopy(self.latest_imuspeed)
        else:
            pass
        self.update_imuspeed = False

        # Update curvature
        if self.update_curvature == True:
            self.curvature = copy.deepcopy(self.latest_curvature)
        else:
            pass
        self.update_curvature = False

        # Update vehicle state
        if self.update_vehicle_state == True:
            self.vehicle_state = copy.deepcopy(self.latest_vehicle_state)
        else:
            pass
        self.update_vehicle_state = False

        # Update velocity level
        if self.update_motion_state == True:
            self.motion_state = copy.deepcopy(self.latest_motion_state)
        else:
            pass
        self.update_motion_state = False

        # Update velocity level
        if self.update_velocity_level == True:
            self.velocity_level = copy.deepcopy(self.latest_velocity_level)
        else:
            pass
        self.update_velocity_level = False

        # Initialize control
        self.control = Control()

        '''
        2. Temporary control mode decision
        '''
        self.temp_control_mode = self.Set_temp_control_mode()

        '''
        3. Curvature to steer angle with compensation
        ''' 
        self.control.steer = self.Curvature2Steer()

        '''
        4. Set gear state by current motion state
        '''
        self.control.gear = self.SetGear()

        '''
        5. Set proper velocity and brake level by velocity level and vehicle state
        '''
        self.control.speed, self.control.brake = self.Set_speed_and_brake()

        '''
        6. Finial decision of control mode
        '''
        self.control.control_mode = self.Set_control_mode()

        '''
        7. Final decision of auto mode and e-stop
        '''
        self.control.is_auto, self.control.estop = self.Set_isauto_and_estop()

        '''
        8. Post processing of control variables(Range of brake, speed, steer/ etc...)
        '''

        '''
        9. Save and return calibrated_control
        '''

        return self.control


# Declare instance of model_estimator class
main_model_estimator = model_estimator()

# Define Callbac functions and some functions
def callbac_update_imuspeed(data):
    main_model_estimator.write_imuspeed(data)
    print ("got imuspeed")
    return 0
    
def callbac_update_curvature(data):
    main_model_estimator.write_curvature(data)
    print ("got curvature")
    return 0

def callbac_update_vehicle_state(data):
    main_model_estimator.write_vehicle_state(data)
    print("got vehicle_state")
    return 0

def callbac_update_motion_state(data):
    main_track.write_motion_state(data)
    print ("got motion state")
    return 0

def callbac_update_velocity_level(data):
    main_track.write_velocity_level(data)
    print ("got velocity level")
    return 0

def mainloop():
    '''
    code for activate and deactivate the node
    '''
    nodename = 'model_estimator'
    mainloop.active = True
    def signalResponse(data):
        mainloop.active
        if 'zero_monitor' in data.active_nodes:
            if nodename in data.active_nodes:
                mainloop.active = True
            else:
                mainloop.active = False
        else:
            rospy.signal_shutdown('no monitor')
    rospy.Subscriber('/active_nodes', ActiveNode, signalResponse)
    '''
    ...
    '''
    rospy.Subscriber("/motion_state", MotionState, callbac_update_motion_state)
    rospy.Subscriber("/vehicle_state", VehicleState, callbac_update_vehicle_state)
    rospy.Subscriber("/imu_speed", ImuSpeed, callbac_update_imuspeed)
    rospy.Subscriber("/curvature", Curvature, callbac_update_curvature)
    rospy.Subscriber('/velocity_level', VelocityLevel, callbac_update_velocity_level)

    pub = rospy.Publisher('/calibrated_control', Control, queue_size = 10)
    rospy.init_node(nodename, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    control = Control()
    control.header.frame_id = 'car_frame'
    i=0
    while not rospy.is_shutdown():
        control.header.stamp = rospy.Time.now()
        control.header.seq = i
        i = i+1
        if mainloop.active:
            pub.publish(main_model_estimator.main_estimate_loop())
        rate.sleep()


if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException:
        pass