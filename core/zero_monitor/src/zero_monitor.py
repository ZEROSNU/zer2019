#!/usr/bin/env python

import pyforms
from pyforms.basewidget import BaseWidget
from pyforms.controls import ControlText
from pyforms.controls import ControlButton
from pyforms.controls import ControlImage
from pyforms.controls import ControlCheckBox
from pyforms.controls import ControlNumber
from pyforms.controls import ControlSlider
from pyforms.controls import ControlCombo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import threading
import cv2
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from core_msgs.msg import MissionState
from core_msgs.msg import LightState
from core_msgs.msg import MotionState
from core_msgs.msg import ImuSpeed
from core_msgs.msg import Control
from core_msgs.msg import VehicleState
from core_msgs.msg import VelocityLevel
from core_msgs.msg import ActiveNode
from core_msgs.msg import Curvature
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class Node(BaseWidget) :
    names = {}
    activeNodes = ActiveNode()
    activeNodes.active_nodes = []
    activeNodes.header.frame_id = 'zero_monitor'
    signalPublisher = rospy.Publisher('/active_nodes', ActiveNode, queue_size = 10)
    def __init__(self, name) :
        super(Node,self).__init__()
        self.name = name
        Node.names[name] = self
        self.pubs = {}
        self.subs = {}
        self.button = ControlButton(name, checkable = True)
        self.button.value = self.__buttonAction

        self._deactivate = ControlButton('deactivate ' + self.name, checkable = True)
        self._deactivate.value = self.__deactivate
        self.formset = [('_deactivate'), {'publish' : [], 'subscribe' : []}]
        Node.activeNodes.active_nodes.append(self.name)
        
    def __buttonAction(self) :
        if self.button.checked == True :
            self.show()
        else :
            self.close()

    def __deactivate(self) :
        if self._deactivate.checked == True :
            Node.activeNodes.active_nodes.remove(self.name)
        else :
            Node.activeNodes.active_nodes.append(self.name)
        Node.pubSignal()

    @classmethod
    def pubSignal(cls) :
        cls.activeNodes.header.stamp = rospy.Time.now()
        cls.signalPublisher.publish(cls.activeNodes)


    def setPub(self, topic) :
        if not (topic in Topic.names.keys()) :
            print 'no topic name' + topic
            return
        self.pubs[topic] = Topic.names[topic]
        Topic.names[topic].setPub(self.name)

        self.formset[1]['publish'].append('_' + topic[1:])
        exec('self._' + topic[1:] + '= ControlButton("' + topic + '")')
        exec('self._' + topic[1:] + '.value = Topic.names["' + topic + '"].button.click')
    def setSub(self, topic) :
        if not (topic in Topic.names.keys()) :
            print 'no topic name' + topic
            return
        self.subs[topic] = Topic.names[topic]
        Topic.names[topic].setSub(self.name)

        self.formset[1]['subscribe'].append('_' + topic[1:])
        exec('self._' + topic[1:] + '= ControlButton("' + topic + '")')
        exec('self._' + topic[1:] + '.value = Topic.names["' + topic + '"].button.click')




class Topic(BaseWidget) :
    names = {}
    def __init__ (self, name, form) :
        super(Topic,self).__init__()
        self.dform = form
        self.name = name
        Topic.names[self.name] = self
        self.pubs = {}
        self.subs = {}
        self.button = ControlButton(name, checkable = True)
        self.button.value = self.__buttonAction
        self._publishData = ControlButton('publish data to ' + self.name, checkable = True)
        self._publishData.value = self.__startPubThread
        self._publishRate = ControlSlider('publish rate')
        self._publishRate.min = 1
        self._publishRate.max = 100
        self._autoFillTime = ControlCheckBox('stamp time and seq automatically')
        self.formset = [('_publishData', '_publishRate', '_autoFillTime'),[], {'publisher' : [], 'subscriber' : []}]
        self.setData()

        self.pub = rospy.Publisher(self.name, self.dform, queue_size=10)
        if self.dform == Image :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackImage)
        elif self.dform == MissionState :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackMissionState)
        elif self.dform == LightState :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackLightState)
        elif self.dform == MotionState :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackMotionState)
        elif self.dform == OccupancyGrid :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackOccupancyGrid)
        elif self.dform == PoseStamped :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackPoseStamped)
        elif self.dform == PoseWithCovarianceStamped :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackPoseWithCovarianceStamped)
        elif self.dform == TFMessage :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackTFMessage)
        elif self.dform == VelocityLevel :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackVelocityLevel)
        elif self.dform == Path :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackPath)
        elif self.dform == Control :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackControl)
        elif self.dform == ImuSpeed :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackImuSpeed)
        elif self.dform == VehicleState :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackVehicleState)
        elif self.dform == Curvature :
            self.sub = rospy.Subscriber(self.name, self.dform, self.__callbackCurvature)
#callbackfunctions
    def __callbackImage(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._height.value = data.height
        self._width.value = data.width
        self._encoding.value = data.encoding
        self._is_bigendian.value = data.is_bigendian
        self._step.value = data.step
        self._img = CvBridge().imgmsg_to_cv2(data, data.encoding)
        if self.button.checked == True :
            cv2.imshow('data of ' + self.name, self._img)
    def __callbackMissionState(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._mission_state.value = data.mission_state
    def __callbackLightState(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._light_found.value = data.light_found
        self._red.value = data.red
        self._yellow.value = data.yellow
        self._green.value = data.green
        self._left.value = data.left
    def __callbackMotionState(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._motion_state.value = data.motion_state
    def __callbackOccupancyGrid(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._info_map_load_time_secs.value = data.info.map_load_time.secs
        self._info_map_load_time_nsecs.value = data.info.map_load_time.nsecs
        self._info_resolution.value = data.info.resolution
        self._info_width.value = data.info.width
        self._info_height.value = data.info.height
        self._info_origin_position_x.value = data.info.origin.position.x
        self._info_origin_position_y.value = data.info.origin.position.y
        self._info_origin_position_z.value = data.info.origin.position.z
        self._info_origin_orientation_x.value = data.info.origin.orientation.x
        self._info_origin_orientation_y.value = data.info.origin.orientation.y
        self._info_origin_orientation_z.value = data.info.origin.orientation.z
        self._info_origin_orientation_w.value = data.info.origin.orientation.w
        self._data = np.array(data.data).reshape(data.info.height, data.info.width).astype(np.uint8)
        self.__dataToImg()
    def __callbackPoseStamped(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._pose_position_x.value = data.pose.position.x
        self._pose_position_y.value = data.pose.position.y
        self._pose_position_z.value = data.pose.position.z
        self._pose_orientation_x.value = data.pose.orientation.x
        self._pose_orientation_y.value = data.pose.orientation.y
        self._pose_orientation_z.value = data.pose.orientation.z
        self._pose_orientation_w.value = data.pose.orientation.w
    def __callbackPoseWithCovarianceStamped(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._pose_pose_position_x.value = data.pose.pose.position.x
        self._pose_pose_position_y.value = data.pose.pose.position.y
        self._pose_pose_position_z.value = data.pose.pose.position.z
        self._pose_pose_orientation_x.value = data.pose.pose.orientation.x
        self._pose_pose_orientation_y.value = data.pose.pose.orientation.y
        self._pose_pose_orientation_z.value = data.pose.pose.orientation.z
        self._pose_pose_orientation_w.value = data.pose.pose.orientation.w
    def __callbackTFMessage(self, data) :
        for i in data.transforms :
            tag = i.header.frame_id + " to " + i.child_frame_id
            if not (i.child_frame_id in self._data.keys()) :
                self._data[i.child_frame_id] = {}
            self._data[i.child_frame_id][i.header.frame_id] = i
            self._transforms += (tag, [i.child_frame_id,i.header.frame_id])
            if self._transforms.value == [i.child_frame_id,i.header.frame_id] :
                self._transforms_header_seq.value = i.header.seq
                self._transforms_header_stamp_secs.value = i.header.stamp.secs
                self._transforms_header_stamp_nsecs.value = i.header.stamp.nsecs
                self._transforms_header_frame_id.value = i.header.frame_id
                self._transforms_child_frame_id.value = i.child_frame_id
                self._transforms_transform_translation_x.value = i.transform.translation.x
                self._transforms_transform_translation_y.value = i.transform.translation.y
                self._transforms_transform_translation_z.value = i.transform.translation.z
                self._transforms_transform_rotation_x.value = i.transform.rotation.x
                self._transforms_transform_rotation_y.value = i.transform.rotation.y
                self._transforms_transform_rotation_z.value = i.transform.rotation.z
                self._transforms_transform_rotation_w.value = i.transform.rotation.w
    def __callbackVelocityLevel(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._velocity_level.value = data.velocity_level
    def __callbackPath(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._poses = data.poses
        self._img_path_type.max = len(self._poses)
        self.__dataToImg()
    def __callbackControl(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._control_mode.value = data.control_mode
        self._is_auto.value = data.is_auto
        self._estop.value = data.estop
        self._gear.value = data.gear
        self._brake.value = data.brake
        self._speed.value = data.speed
        self._steer.value = data.steer
    def __callbackImuSpeed(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._imu_speed.value = data.imu_speed
    def __callbackVehicleState(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._is_auto.value = data.is_auto
        self._estop.value = data.estop
        self._gear.value = data.gear
        self._brake.value = data.brake
        self._speed.value = data.speed
        self._steer.value = data.steer
        self._encoder.value = data.encoder
        self._alive.value = data.alive
    def __callbackCurvature(self, data) :
        self._header_seq.value = data.header.seq
        self._header_stamp_secs.value = data.header.stamp.secs
        self._header_stamp_nsecs.value = data.header.stamp.nsecs
        self._header_frame_id.value = data.header.frame_id
        self._curvature.value = data.curvature
    def __callbackMouse(self, event, x, y, flags, param) :
        for i in self.pubs.keys() :
            if self.pubs[i]._deactivate.checked == False :
                return
        if self.dform == OccupancyGrid :
            if event == cv2.EVENT_LBUTTONDOWN :
                self._img_drawing = True
                if self._img_brush_type.value == 'brush' :
                    cv2.circle(self._img,(x,y),self._img_brush_radius.value,[155,155,155],-1)
                elif self._img_brush_type.value == 'eraser' :
                    cv2.circle(self._img,(x,y),self._img_brush_radius.value,[255,255,255],-1)
                self.__imgToData()
                self.__dataToImg()
            elif event == cv2.EVENT_MOUSEMOVE :
                if self._img_drawing == True :
                    if self._img_brush_type.value == 'brush' :
                        cv2.circle(self._img,(x,y),self._img_brush_radius.value,[155,155,155],-1)
                    elif self._img_brush_type.value == 'eraser' :
                        cv2.circle(self._img,(x,y),self._img_brush_radius.value,[255,255,255],-1)
                    self.__imgToData()
                    self.__dataToImg()
            elif event == cv2.EVENT_LBUTTONUP :
                self._img_drawing = False
                self.__imgToData()
                self.__dataToImg()
        elif self.dform == Path :
            if self._img_add_pose.checked == True and self._img_path_type.value != 0:
                if event == cv2.EVENT_LBUTTONDOWN :
                    self._img_drawing = (x/2,y/2)
                elif event == cv2.EVENT_LBUTTONUP :
                    if self._img_drawing != None :
                        pose = PoseStamped()
                        if self._autoFillTime.value :
                            head = Header()
                            now = rospy.Time.now()
                            head.stamp = now
                            pose.header = head
                        pose.pose.position.x = (self._img_drawing[0] - self._width/2) * self._res
                        pose.pose.position.y = (self._height - self._img_drawing[1]) * self._res
                        headx = (x/2 - self._width/2) * self._res
                        heady = (self._height - y/2) * self._res
                        theta = np.arctan2(heady-pose.pose.position.y, headx-pose.pose.position.x) / 2
                        pose.pose.orientation.z = np.sin(theta)
                        pose.pose.orientation.w = np.cos(theta)
                        self._poses.insert(self._img_path_type.value, pose)
                        self.__dataToImg()

    def __dataToImg(self) :
        if self.dform == OccupancyGrid :
            img = 255-self._data
            img = cv2.flip(img,0)
            img = np.expand_dims(img, axis = 2)
            self._img = np.concatenate((img, img, img), axis = 2).astype(np.uint8)
        elif self.dform == Path :
            self._img_path_type.max = len(self._poses)
            img = np.zeros([self._height, self._width, 3])
            tmp = None
            tr = np.array([[1/self._res,0,self._width/2],
                            [0,-1/self._res,self._height]])

            if self._img_path_type.value == 0 :
                for pose in self._poses :
                    yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                    pt1 = tuple(np.dot(tr,np.array([pose.pose.position.x, pose.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                    pt2 = tuple(np.dot(tr,np.array([pose.pose.position.x + self._delta*np.cos(yaw), pose.pose.position.y + self._delta*np.sin(yaw),1])).reshape(1,-1)[0].astype('int'))
                    cv2.arrowedLine(img, pt1, pt2, (0,0,255), 1)
            else :
                pose = self._poses[self._img_path_type.value - 1]
                yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                pt1 = tuple(np.dot(tr,np.array([pose.pose.position.x, pose.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                pt2 = tuple(np.dot(tr,np.array([pose.pose.position.x + self._delta*np.cos(yaw), pose.pose.position.y + self._delta*np.sin(yaw),1])).reshape(1,-1)[0].astype('int'))
                cv2.arrowedLine(img, pt1, pt2, (0,0,255), 1)
            for pose in self._poses :
                if tmp!=None :
                    pt1 = tuple(np.dot(tr,np.array([tmp.pose.position.x, tmp.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                    pt2 = tuple(np.dot(tr,np.array([pose.pose.position.x, pose.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                    cv2.line(img, pt1,pt2, (255,255,255),1)
                tmp = pose
            self._img = cv2.resize(img, None, fx = 2, fy = 2, interpolation = cv2.INTER_NEAREST)

        if self.button.checked == True :
            cv2.imshow('data of ' + self.name, self._img)
    def __imgToData(self) :
        if self.dform == OccupancyGrid :
            data = np.delete(self._img,(1,2),2)
            data = data.reshape(data.shape[0],data.shape[1])
            data = cv2.flip(data,0)
            self._data = 255-data
    def __deletePose(self) :
        if self.dform == Path :
            if self._img_path_type.value != 0 :
                del self._poses[self._img_path_type.value - 1]
                self.__dataToImg()
    def __changeTfmessage(self) :
        val = self._transforms.value
        if val == None :
            return
        i = self._data[val[0]][val[1]]
        if self._transforms.value == [i.child_frame_id,i.header.frame_id] :
            self._transforms_header_seq.value = i.header.seq
            self._transforms_header_stamp_secs.value = i.header.stamp.secs
            self._transforms_header_stamp_nsecs.value = i.header.stamp.nsecs
            self._transforms_header_frame_id.value = i.header.frame_id
            self._transforms_child_frame_id.value = i.child_frame_id
            self._transforms_transform_translation_x.value = i.transform.translation.x
            self._transforms_transform_translation_y.value = i.transform.translation.y
            self._transforms_transform_translation_z.value = i.transform.translation.z
            self._transforms_transform_rotation_x.value = i.transform.rotation.x
            self._transforms_transform_rotation_y.value = i.transform.rotation.y
            self._transforms_transform_rotation_z.value = i.transform.rotation.z
            self._transforms_transform_rotation_w.value = i.transform.rotation.w
    def __setState(self) :
        if self.dform == MissionState :
            self._mission_state.value = self._setState.value
        elif self.dform == MotionState :
            self._motion_state.value = self._setState.value

    def __buttonAction(self) :
        if self.button.checked == True :
            if self.dform in [Image, OccupancyGrid, Path]:
                cv2.imshow('data of ' + self.name, self._img)
                cv2.setMouseCallback('data of ' + self.name, self.__callbackMouse)
            self.show()
        else :
            if self.dform in [Image, OccupancyGrid, Path] :
                cv2.destroyWindow('data of ' + self.name)
            self.close()
    def __startPubThread(self) :
        pub = threading.Thread(target = self.__publishData)
        pub.start()
    def __publishData(self) :
        while self._publishData.checked and (not rospy.is_shutdown()):
            rate = rospy.Rate(self._publishRate.value)
            head = Header()
            if self._autoFillTime.value :
                now = rospy.Time.now()
                head.stamp = now
#publishfunctions
            if self.dform == Image :
                head.frame_id = self._header_frame_id.value
                pubData = CvBridge().cv2_to_imgmsg(self._data, self._encoding.value)
                pubData.header = head
                self.pub.publish(pubData)
            elif self.dform == MissionState :
                head.frame_id = self._header_frame_id.value
                pubData = MissionState()
                pubData.header = head
                pubData.mission_state = self._mission_state.value
                self.pub.publish(pubData)
            elif self.dform == LightState :
                head.frame_id = self._header_frame_id.value
                pubData = LightState()
                pubData.header = head
                pubData.light_found = self._light_found.value
                pubData.red = self._red.value
                pubData.yellow = self._yellow.value
                pubData.green = self._green.value
                pubData.left = self._left.value
                self.pub.publish(pubData)
            elif self.dform == MotionState :
                head.frame_id = self._header_frame_id.value
                pubData = MotionState()
                pubData.header = head
                pubData.motion_state = self._motion_state.value
                self.pub.publish(pubData)
            elif self.dform == OccupancyGrid :
                head.frame_id = self._header_frame_id.value
                pubData = OccupancyGrid()
                pubData.header = head
                pubData.info.map_load_time.secs = self._info_map_load_time_secs.value
                pubData.info.map_load_time.nsecs = self._info_map_load_time_nsecs.value
                pubData.info.resolution = self._info_resolution.value
                pubData.info.width = int(self._info_width.value)
                pubData.info.height = int(self._info_height.value)
                pubData.info.origin.position.x = self._info_origin_position_x.value
                pubData.info.origin.position.y = self._info_origin_position_y.value
                pubData.info.origin.position.z = self._info_origin_position_z.value
                qnorm = np.linalg.norm([self._info_origin_orientation_x.value, self._info_origin_orientation_y.value, self._info_origin_orientation_z.value, self._info_origin_orientation_w.value])
                pubData.info.origin.orientation.x = self._info_origin_orientation_x.value / qnorm
                pubData.info.origin.orientation.y = self._info_origin_orientation_y.value / qnorm
                pubData.info.origin.orientation.z = self._info_origin_orientation_z.value / qnorm
                pubData.info.origin.orientation.w = self._info_origin_orientation_w.value / qnorm
                self.__imgToData()
                pubData.data = self._data.flatten().tolist()
                self.pub.publish(pubData)
            elif self.dform == PoseStamped :
                head.frame_id = self._header_frame_id.value
                pubData = PoseStamped()
                pubData.header = head
                pubData.pose.position.x = self._pose_position_x.value
                pubData.pose.position.y = self._pose_position_y.value
                pubData.pose.position.z = self._pose_position_z.value
                qnorm = np.linalg.norm([self._pose_orientation_x.value, self._pose_orientation_y.value, self._pose_orientation_z.value, self._pose_orientation_w.value])
                pubData.pose.orientation.x = self._pose_orientation_x.value / qnorm
                pubData.pose.orientation.y = self._pose_orientation_y.value / qnorm
                pubData.pose.orientation.z = self._pose_orientation_z.value / qnorm
                pubData.pose.orientation.w = self._pose_orientation_w.value / qnorm
                self.pub.publish(pubData)
            elif self.dform == PoseWithCovarianceStamped :
                head.frame_id = self._header_frame_id.value
                pubData = PoseWithCovarianceStamped()
                pubData.header = head
                pubData.pose.pose.position.x = self._pose_pose_position_x.value
                pubData.pose.pose.position.y = self._pose_pose_position_y.value
                pubData.pose.pose.position.z = self._pose_pose_position_z.value
                qnorm = np.linalg.norm([self._pose_pose_orientation_x.value, self._pose_pose_orientation_y.value, self._pose_pose_orientation_z.value, self._pose_pose_orientation_w.value])
                pubData.pose.pose.orientation.x = self._pose_pose_orientation_x.value / qnorm
                pubData.pose.pose.orientation.y = self._pose_pose_orientation_y.value / qnorm
                pubData.pose.pose.orientation.z = self._pose_pose_orientation_z.value / qnorm
                pubData.pose.pose.orientation.w = self._pose_pose_orientation_w.value / qnorm
            elif self.dform == TFMessage :
                head.frame_id = self._transforms_header_frame_id.value
                pubData = TFMessage()
                tfm = TransformStamped()
                tfm.header = head
                tfm.child_frame_id = self._transforms_child_frame_id.value
                tfm.transform.translation.x = self._transforms_transform_translation_x.value
                tfm.transform.translation.y = self._transforms_transform_translation_y.value
                tfm.transform.translation.z = self._transforms_transform_translation_z.value
                qnorm = np.linalg.norm([self._transforms_transform_rotation_x.value, self._transforms_transform_rotation_y.value, self._transforms_transform_rotation_z.value, self._transforms_transform_rotation_w.value])
                tfm.transform.rotation.x = self._transforms_transform_rotation_x.value / qnorm
                tfm.transform.rotation.y = self._transforms_transform_rotation_y.value / qnorm
                tfm.transform.rotation.z = self._transforms_transform_rotation_z.value / qnorm
                tfm.transform.rotation.w = self._transforms_transform_rotation_w.value / qnorm
                pubData.transforms.append(tfm)
                self.pub.publish(pubData)
            elif self.dform == VelocityLevel :
                head.frame_id = self._header_frame_id.value
                pubData = VelocityLevel()
                pubData.header = head
                pubData.velocity_level = self._velocity_level.value
                self.pub.publish(pubData)
            elif self.dform == Path :
                head.frame_id = self._header_frame_id.value
                pubData = Path()
                pubData.header = head
                pubData.poses = self._poses
                self.pub.publish(pubData)
            elif self.dform == Control :
                head.frame_id = self._header_frame_id.value
                pubData = Control()
                pubData.header = head
                pubData.control_mode = self._control_mode.value
                pubData.is_auto = self._is_auto.value
                pubData.estop = self._estop.value
                pubData.gear = self._gear.value
                pubData.brake = self._brake.value
                pubData.speed = self._speed.value
                pubData.steer = self._steer.value
                self.pub.publish(pubData)
            elif self.dform == ImuSpeed :
                head.frame_id = self._header_frame_id.value
                pubData = ImuSpeed()
                pubData.header = head
                pubData.imu_speed = self._imu_speed.value
                self.pub.publish(pubData)
            elif self.dform == VehicleState :
                head.frame_id = self._header_frame_id.value
                pubData = VehicleState()
                pubData.header = head
                pubData.is_auto = self._is_auto.value
                pubData.estop = self._estop.value
                pubData.gear = self._gear.value
                pubData.brake = self._brake.value
                pubData.speed = self._speed.value
                pubData.steer = self._steer.value
                pubData.encoder = self._encoder.value
                pubData.alive = self._alive.value
                self.pub.publish(pubData)
            elif self.dform == Curvature :
                head.frame_id = self._header_frame_id.value
                pubData = Curvature()
                pubData.header = head
                pubData.curvature = self._curvature.value
                self.pub.publish(pubData)
            
            rate.sleep()

    def setPub(self, nodename) :
        self.pubs[nodename] = Node.names[nodename]
        self.formset[2]['publisher'].append('_' + nodename)
        exec('self._' + nodename + '= ControlButton("' + nodename + '")')
        exec('self._' + nodename + '.value = Node.names["' + nodename + '"].button.click')

    def setSub(self, nodename) :
        self.subs[nodename] =  Node.names[nodename]
        self.formset[2]['subscriber'].append('_' + nodename)
        exec('self._' + nodename + '= ControlButton("' + nodename + '")')
        exec('self._' + nodename + '.value = Node.names["' + nodename + '"].button.click')
#initializefunctions
    def setData(self) :
        if self.dform != TFMessage :
            self._header_seq = ControlNumber('header.seq')
            self._header_seq.max = 1000000
            self.formset[1].append('_header_seq')
            self._header_stamp_secs = ControlNumber('header.stamp_secs')
            self._header_stamp_secs.max = 1000000000000
            self._header_stamp_nsecs = ControlNumber('nsecs')
            self._header_stamp_nsecs.max = 100000000000
            self.formset[1].append( ('_header_stamp_secs', '_header_stamp_nsecs'))
            self._header_frame_id = ControlText('header.frame_id')
            self.formset[1].append('_header_frame_id')

        if self.dform == Image :
            self._height = ControlNumber('height')
            self._height.max = 10000
            self.formset[1].append('_height')
            self._width = ControlNumber('width')
            self._width.max = 10000
            self.formset[1].append('_width')
            self._encoding = ControlText('encoding')
            self.formset[1].append('_encoding')
            self._is_bigendian = ControlSlider('is_bigendian')
            self._is_bigendian.max = 1
            self.formset[1].append('_is_bigendian')
            self._step = ControlNumber('step')
            self._step.max = 1000000
            self.formset[1].append('_step')
            self._img = np.zeros([200,200])
        elif self.dform == MissionState :
            self._mission_state = ControlText('mission_state')
            self.formset[1].append('_mission_state')
            self._setState = ControlCombo('set mission state as')
            self._setState.add_item('DRIVING_SECTION')
            self._setState.add_item('INTERSECTION_LEFT')
            self._setState.add_item('INTERSECTION_RIGHT')
            self._setState.add_item('INTERSECTION_STRAIGHT')
            self._setState.add_item('OBSTACLE_STATIC')
            self._setState.add_item('OBSTACLE_SUDDEN')
            self._setState.add_item('CROSSWALK')
            self._setState.add_item('SCHOOL_ZONE')
            self._setState.add_item('SPEED_BUST')
            self._setState.add_item('PARKING')
            self._setStateButton = ControlButton('Set')
            self._setStateButton.value = self.__setState
            self.formset[1].append(('_setState', '_setStateButton'))
        elif self.dform == LightState :
            self._light_found = ControlSlider('light_found')
            self._light_found.max = 1
            self.formset[1].append('_light_found')
            self._red = ControlSlider('red')
            self._red.max = 1
            self.formset[1].append('_red')
            self._yellow = ControlSlider('yellow')
            self._yellow.max = 1
            self.formset[1].append('_yellow')
            self._green = ControlSlider('green')
            self._green.max = 1
            self.formset[1].append('_green')
            self._left = ControlSlider('left')
            self._left.max = 1
            self.formset[1].append('_left')
        elif self.dform == MotionState :
            self._motion_state = ControlText('motion_state')
            self.formset[1].append('_motion_state')
            self._setState = ControlCombo('set motion state as')
            self._setState.add_item('FORWARD_MOTION')
            self._setState.add_item('FORWARD_MOTION_SLOW')
            self._setState.add_item('HALT')
            self._setState.add_item('LEFT_MOTION')
            self._setState.add_item('RIGHT_MOTION')
            self._setState.add_item('PARKING')
            self._setStateButton = ControlButton('Set')
            self._setStateButton.value = self.__setState
            self.formset[1].append(('_setState', '_setStateButton'))
        elif self.dform == OccupancyGrid :
            self._header_frame_id.value = 'car_frame'
            self._info_map_load_time_secs = ControlNumber('info.map_load_time.secs')
            self._info_map_load_time_secs.max = 1000000000000
            self._info_map_load_time_nsecs = ControlNumber('nsecs')
            self._info_map_load_time_nsecs.max = 100000000000
            self.formset[1].append(('_info_map_load_time_secs', '_info_map_load_time_nsecs'))
            self._info_resolution = ControlNumber('info.map_resolution')
            self._info_resolution.decimals = 3
            self._info_resolution.value = 0.03
            self.formset[1].append('_info_resolution')
            self._info_width = ControlNumber('info.width')
            self._info_width.max = 10000
            self._info_width.value = 200
            self.formset[1].append('_info_width')
            self._info_height = ControlNumber('info.height')
            self._info_height.max = 10000
            self._info_height.value = 200
            self.formset[1].append('_info_height')
            self._info_origin_position_x = ControlNumber('info.origin.position.x:')
            self._info_origin_position_x.min = -100
            self._info_origin_position_x.value = -3
            self._info_origin_position_x.decimals = 3
            self._info_origin_position_y = ControlNumber('y:')
            self._info_origin_position_y.min = -100
            self._info_origin_position_y.decimals = 3
            self._info_origin_position_z = ControlNumber('z:')
            self._info_origin_position_z.min = -100
            self._info_origin_position_z.decimals = 3
            self.formset[1].append( ('_info_origin_position_x', '_info_origin_position_y', '_info_origin_position_z') )
            self._info_origin_orientation_x = ControlNumber('info.origin.orientation.x:')
            self._info_origin_orientation_x.min = -100
            self._info_origin_orientation_x.decimals = 3
            self._info_origin_orientation_y = ControlNumber('y:')
            self._info_origin_orientation_y.min = -100
            self._info_origin_orientation_y.decimals = 3
            self._info_origin_orientation_z = ControlNumber('z:')
            self._info_origin_orientation_z.min = -100
            self._info_origin_orientation_z.decimals = 3
            self._info_origin_orientation_w = ControlNumber('w:')
            self._info_origin_orientation_w.min = -100
            self._info_origin_orientation_w.value = 1
            self._info_origin_orientation_w.decimals = 3
            self.formset[1].append( ('_info_origin_orientation_x', '_info_origin_orientation_y', '_info_origin_orientation_z', '_info_origin_orientation_w') )
            self._data = np.full([int(self._info_height.value),int(self._info_width.value)],0, dtype = np.uint8)
            self.formset[1].append('drawing tools')
            self._img = np.zeros([200,200,3], dtype = np.uint8)
            self.__dataToImg()
            self._img_brush_type = ControlCombo('brush')
            self._img_brush_type.add_item('brush', 'brush')
            self._img_brush_type.add_item('eraser', 'eraser')
            self.formset[1].append('_img_brush_type')
            self._img_drawing = False
            self._img_brush_radius = ControlSlider('circle radius:')
            self._img_brush_radius.min = 1
            self.formset[1].append('_img_brush_radius')
        elif self.dform == PoseStamped :
            self._header_frame_id.value = "car_frame"
            self._pose_position_x = ControlNumber('pose.position.x:')
            self._pose_position_x.min = -100
            self._pose_position_x.decimals = 3
            self._pose_position_y = ControlNumber('y:')
            self._pose_position_y.min = -100
            self._pose_position_y.value = 5
            self._pose_position_y.decimals = 3
            self._pose_position_z = ControlNumber('z:')
            self._pose_position_z.min = -100
            self._pose_position_z.decimals = 3
            self.formset[1].append( ('_pose_position_x','_pose_position_y','_pose_position_z' ))
            self._pose_orientation_x = ControlNumber('pose.orientation.x')
            self._pose_orientation_x.min = -100
            self._pose_orientation_x.decimals = 3
            self._pose_orientation_y = ControlNumber('y:')
            self._pose_orientation_y.min = -100
            self._pose_orientation_y.decimals = 3
            self._pose_orientation_z = ControlNumber('z:')
            self._pose_orientation_z.value = 1
            self._pose_orientation_z.min = -100
            self._pose_orientation_z.decimals = 3
            self._pose_orientation_w = ControlNumber('w:')
            self._pose_orientation_w.min = -100
            self._pose_orientation_w.value = 1
            self._pose_orientation_w.decimals = 3
            self.formset[1].append( ('_pose_orientation_x', '_pose_orientation_y', '_pose_orientation_z', '_pose_orientation_w') )
        elif self.dform == PoseWithCovarianceStamped :
            self._pose_pose_position_x = ControlNumber('pose.pose.position.x:')
            self._pose_pose_position_x.min = -100
            self._pose_pose_position_x.decimals = 3
            self._pose_pose_position_y = ControlNumber('y:')
            self._pose_pose_position_y.min = -100
            self._pose_pose_position_y.decimals = 3
            self._pose_pose_position_z = ControlNumber('z:')
            self._pose_pose_position_z.min = -100
            self._pose_pose_position_z.decimals = 3
            self.formset[1].append( ('_pose_pose_position_x','_pose_pose_position_y','_pose_pose_position_z' ))
            self._pose_pose_orientation_x = ControlNumber('pose.pose.orientation.x')
            self._pose_pose_orientation_x.min = -100
            self._pose_pose_orientation_x.decimals = 3
            self._pose_pose_orientation_y = ControlNumber('y:')
            self._pose_pose_orientation_y.min = -100
            self._pose_pose_orientation_y.decimals = 3
            self._pose_pose_orientation_z = ControlNumber('z:')
            self._pose_pose_orientation_z.min = -100
            self._pose_pose_orientation_z.decimals = 3
            self._pose_pose_orientation_w = ControlNumber('w:')
            self._pose_pose_orientation_w.min = -100
            self._pose_pose_orientation_w.decimals = 3
            self.formset[1].append( ('_pose_pose_orientation_x', '_pose_pose_orientation_y', '_pose_pose_orientation_z', '_pose_pose_orientation_w') )
        elif self.dform == TFMessage :
            self._data = {}
            self._transforms = ControlCombo('transforms')
            self._transforms.changed_event = self.__changeTfmessage
            self.formset[1].append( ('_transforms'))
            self._transforms_header_seq = ControlNumber('header.seq')
            self._transforms_header_seq.max = 1000000
            self.formset[1].append('_transforms_header_seq')
            self._transforms_header_stamp_secs = ControlNumber('header.stamp_secs')
            self._transforms_header_stamp_secs.max = 1000000000000
            self._transforms_header_stamp_nsecs = ControlNumber('nsecs')
            self._transforms_header_stamp_nsecs.max = 100000000000
            self.formset[1].append(('_transforms_header_stamp_secs', '_transforms_header_stamp_nsecs'))
            self._transforms_header_frame_id = ControlText('header.frame_id')
            self.formset[1].append('_transforms_header_frame_id')
            self._transforms_child_frame_id = ControlText('child_frame_id')
            self.formset[1].append('_transforms_child_frame_id')
            self._transforms_transform_translation_x = ControlNumber('transform.translation.x:')
            self._transforms_transform_translation_x.min = -100
            self._transforms_transform_translation_x.decimals = 3
            self._transforms_transform_translation_y = ControlNumber('y:')
            self._transforms_transform_translation_y.min = -100
            self._transforms_transform_translation_y.decimals = 3
            self._transforms_transform_translation_z = ControlNumber('z:')
            self._transforms_transform_translation_z.min = -100
            self._transforms_transform_translation_z.decimals = 3
            self.formset[1].append(('_transforms_transform_translation_x','_transforms_transform_translation_y','_transforms_transform_translation_z'))
            self._transforms_transform_rotation_x = ControlNumber('transform.rotation.x:')
            self._transforms_transform_rotation_x.min = -100
            self._transforms_transform_rotation_x.decimals = 3
            self._transforms_transform_rotation_y = ControlNumber('y:')
            self._transforms_transform_rotation_y.min = -100
            self._transforms_transform_rotation_y.decimals = 3
            self._transforms_transform_rotation_z = ControlNumber('z:')
            self._transforms_transform_rotation_z.min = -100
            self._transforms_transform_rotation_z.decimals = 3
            self._transforms_transform_rotation_w = ControlNumber('w:')
            self._transforms_transform_rotation_w.min = -100
            self._transforms_transform_rotation_w.decimals = 3
            self.formset[1].append(('_transforms_transform_rotation_x','_transforms_transform_rotation_y','_transforms_transform_rotation_z','_transforms_transform_rotation_w'))
        elif self.dform == VelocityLevel :
            self._velocity_level = ControlNumber('velocity_level')
            self._velocity_level.decimals = 3
            self.formset[1].append('_velocity_level')
        elif self.dform == Path :
            self._height = 200
            self._width = 200
            self._res = 0.03
            self._delta = 0.5
            self._poses = []
            self._img = np.zeros([200,200])
            self._img_path_type = ControlSlider('show path with nth(index+1) pose')
            self._img_path_type.max = len(self._poses)
            self._img_path_type.changed_event = self.__dataToImg
            self._img_add_pose = ControlButton('add pose after current pose', checkable = True)
            self._img_delete_pose = ControlButton('delete current pose')
            self._img_delete_pose.value = self.__deletePose
            self.formset[1].append(('_img_add_pose', '_img_delete_pose'))
            self.formset[1].append('_img_path_type')
            self._img_drawing = None
        elif self.dform == Control :
            self._control_mode = ControlText('control_mode')
            self.formset[1].append('_control_mode')
            self._is_auto = ControlSlider('is_auto')
            self._is_auto.max = 1
            self.formset[1].append('_is_auto')
            self._estop = ControlSlider('estop')
            self._estop.max = 1
            self.formset[1].append('_estop')
            self._gear = ControlSlider('gear')
            self._gear.max = 2
            self.formset[1].append('_gear')
            self._brake = ControlNumber('brake')
            self._brake.max = 150
            self.formset[1].append('_brake')
            self._speed = ControlNumber('speed')
            self._speed.decimals = 3
            self.formset[1].append('_speed')
            self._steer = ControlNumber('steer')
            self._steer.decimals = 3
            self.formset[1].append('_steer')
        elif self.dform == ImuSpeed :
            self._imu_speed = ControlNumber('imu_speed')
            self._imu_speed.decimals = 3
            self.formset[1].append('_imu_speed')
        elif self.dform == VehicleState :        
            self._is_auto = ControlSlider('is_auto')
            self._is_auto.max = 1
            self.formset[1].append('_is_auto')
            self._estop = ControlSlider('_estop')
            self._estop.max = 1
            self.formset[1].append('_estop')
            self._gear = ControlSlider('gear')
            self._gear.max = 2
            self.formset[1].append('_gear')
            self._brake = ControlNumber('brake')
            self._brake.max = 150
            self.formset[1].append('_brake')
            self._speed = ControlNumber('speed')
            self._speed.decimals = 3
            self.formset[1].append('_speed')
            self._steer = ControlNumber('steer')
            self._steer.decimals = 3
            self.formset[1].append('_steer')
            self._encoder = ControlNumber('encoder')
            self._encoder.max = 10000000000
            self._encoder.min = -10000000000
            self.formset[1].append('_encoder')
            self._alive = ControlNumber('alive')
            self._alive.max = 255
            self.formset[1].append('_alive')
        elif self.dform == Curvature :
            self._curvature = ControlNumber('curvature')
            self._curvature.max = 1000
            self._curvature.decimals = 3
            self.formset[1].append('_curvature')

class MainMonitor(BaseWidget) :
    def __init__(self) :
        BaseWidget.__init__(self)
        Node.activeNodes.active_nodes.append('zero_monitor')
        rospy.init_node('zero_monitor')
        self._logo_path = rospy.get_param('/logo_path')
        Topic('/occupancy_map', Image)
        self._topicOccupancyMap = Topic.names['/occupancy_map'].button
        Topic('/lane_data', Image)
        self._topicLaneData = Topic.names['/lane_data'].button
        Topic('/mission_state', MissionState)
        self._topicMissionState = Topic.names['/mission_state'].button
        Topic('/light_state', LightState)
        self._topicLightState = Topic.names['/light_state'].button
        Topic('/task', MissionState)
        self._topicTask = Topic.names['/task'].button
        Topic('/motion_state', MotionState)
        self._topicMotionState = Topic.names['/motion_state'].button
        Topic('/local_map', OccupancyGrid)
        self._topicLocalMap = Topic.names['/local_map'].button
        Topic('/goal_pose', PoseStamped)
        self._topicGoalPose = Topic.names['/goal_pose'].button
        Topic('/tf', TFMessage)
        self._topicTF = Topic.names['/tf'].button
        Topic('/vehicle_pose', PoseWithCovarianceStamped)
        self._topicVehiclePose = Topic.names['/vehicle_pose'].button
        Topic('/global_map', OccupancyGrid)
        self._topicGlobalMap = Topic.names['/global_map'].button
        Topic('/velocity_level', VelocityLevel)
        self._topicVelocityLevel = Topic.names['/velocity_level'].button
        Topic('/path', Path)
        self._topicPath = Topic.names['/path'].button
        Topic('/curvature', Curvature)
        self._topicCurvature = Topic.names['/curvature'].button
        Topic('/imu_speed', ImuSpeed)
        self._topicImuSpeed = Topic.names['/imu_speed'].button
        Topic('/calibrated_control', Control)
        self._topicCalibratedControl = Topic.names['/calibrated_control'].button
        Topic('/vehicle_state', VehicleState)
        self._topicVehicleState = Topic.names['/vehicle_state'].button
        #########################################################################
        Node('lidar')
        self._nodeLidar = Node.names['lidar'].button
        Node.names['lidar'].setPub('/occupancy_map')
        Node('forward_camera')
        self._nodeForwardCamera = Node.names['forward_camera'].button
        Node.names['forward_camera'].setPub('/lane_data')
        Node('gps')
        self._nodeGPS = Node.names['gps'].button
        Node.names['gps'].setPub('/mission_state')
        Node('traffic_light')
        self._nodeTrafficLight = Node.names['traffic_light'].button
        Node.names['traffic_light'].setPub('/light_state')
        Node('sign_camera')
        self._nodeSignCamera = Node.names['sign_camera'].button
        Node.names['sign_camera'].setPub('/task')
        Node('imu_encoder')
        self._nodeIMUEncoder = Node.names['imu_encoder'].button
        Node.names['imu_encoder'].setPub('/imu_speed')
        Node('mission_master')
        self._nodeMissionMaster = Node.names['mission_master'].button
        Node.names['mission_master'].setSub('/mission_state')
        Node.names['mission_master'].setSub('/light_state')
        Node.names['mission_master'].setSub('/task')
        Node.names['mission_master'].setPub('/motion_state')
        Node('map_merger')
        self._nodeMapMerger = Node.names['map_merger'].button
        Node.names['map_merger'].setSub('/occupancy_map')
        Node.names['map_merger'].setSub('/lane_data')
        Node.names['map_merger'].setSub('/motion_state')
        Node.names['map_merger'].setPub('/goal_pose')
        Node.names['map_merger'].setPub('/local_map')
        Node.names['map_merger'].setPub('/velocity_level')
        Node('hector_slam')
        self._nodeHectorSlam = Node.names['hector_slam'].button
        Node.names['hector_slam'].setPub('/tf')
        Node.names['hector_slam'].setPub('/vehicle_pose')
        Node.names['hector_slam'].setPub('/global_map')
        Node('path_planner')
        self._nodePathPlanner = Node.names['path_planner'].button
        Node.names['path_planner'].setSub('/local_map')
        Node.names['path_planner'].setSub('/goal_pose')
        Node.names['path_planner'].setSub('/tf')
        Node.names['path_planner'].setSub('/vehicle_pose')
        Node.names['path_planner'].setSub('/global_map')
        Node.names['path_planner'].setPub('/path')
        Node('path_tracker')
        self._nodePathTracker = Node.names['path_tracker'].button
        Node.names['path_tracker'].setSub('/velocity_level')
        Node.names['path_tracker'].setSub('/path')
        Node.names['path_tracker'].setPub('/curvature')
        Node('model_estimator')
        self._nodeModelEstimator = Node.names['model_estimator'].button
        Node.names['model_estimator'].setSub('/curvature')
        Node.names['model_estimator'].setSub('/velocity_level')
        Node.names['model_estimator'].setSub('/imu_speed')
        Node.names['model_estimator'].setPub('/calibrated_control')
        Node('serial_communicator')
        self._nodeSerialCommunicator = Node.names['serial_communicator'].button
        Node.names['serial_communicator'].setSub('/calibrated_control')
        Node.names['serial_communicator'].setPub('/vehicle_state')

        self._overallImage = ControlButton('overall image', checkable = True)
        self._overallImage.value = self.__showOverAll
        img = cv2.imread(self._logo_path + 'architecture.jpeg',1)
        height, width, depth = img.shape
        imgScale = 1024.0/height
        newX,newY = img.shape[1]*imgScale,img.shape[0]*imgScale
        img = cv2.resize(img,(int(newX),int(newY)))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('architecture', img)

        self._closeAll = ControlButton('Close All')
        self._closeAll.value = self.__closeAll
        self._deactivateAll = ControlButton('deactivate all')
        self._deactivateAll.value = self.__deactivateAll
        self._activateAll = ControlButton('activate all')
        self._activateAll.value = self.__activateAll
        self.formset = [[('-----------------------Nodes-----------------------'),
                        ('_nodeLidar', '_nodeForwardCamera', '_nodeGPS', '_nodeTrafficLight', '_nodeSignCamera', '_nodeIMUEncoder'),
                        ('_nodeMissionMaster', '_nodeMapMerger', '_nodeHectorSlam', '_nodePathPlanner'),
                        ('_nodePathTracker', '_nodeModelEstimator', '_nodeSerialCommunicator'),
                        ('-----------------------Topics----------------------'),
                        ('_topicOccupancyMap', '_topicLaneData', '_topicMissionState', '_topicLightState', '_topicTask','_topicImuSpeed'),
                        ('_topicMotionState', '_topicVelocityLevel', '_topicLocalMap', '_topicGoalPose', '_topicTF', '_topicVehiclePose', '_topicGlobalMap'),
                        ('_topicPath', '_topicCurvature', '_topicCalibratedControl','_topicVehicleState'),
                        ('_---------------------------------------------------'),
                        ('_closeAll'),
                        ('_deactivateAll', '_activateAll'),
                        ('_overallImage')
                        ]]
    def __deactivateAll(self) :
        for name in Node.names.keys() :
            if Node.names[name]._deactivate.checked == False :
                Node.names[name]._deactivate.checked = True
                Node.activeNodes.active_nodes.remove(Node.names[name].name)
        Node.pubSignal()
    def __activateAll(self) :
        for name in Node.names.keys() :
            if Node.names[name]._deactivate.checked == True :
                Node.names[name]._deactivate.checked = False
                Node.activeNodes.active_nodes.append(Node.names[name].name)
        Node.pubSignal()
    def __closeAll(self) :
        for name in Node.names :
            Node.names[name].close()
        for name in Topic.names :
            Topic.names[name].close()
            Topic.names[name]._publishData.checked = False
        cv2.destroyAllWindows()
        Node.activeNodes.active_nodes = []
        Node.pubSignal()
        self.close()
        exit()
    def __showOverAll(self) :
        if self._overallImage.checked == True :
            update = threading.Thread(target = self.__updateOverAll)
            update.start()
    def __updateOverAll(self) :
        while self._overallImage.checked == True and (not rospy.is_shutdown()):
            img = 255-Topic.names['/local_map']._data
            img = cv2.flip(img,0)
            img = np.expand_dims(img, axis = 2)
            img = np.concatenate((img, img, img), axis = 2)

            tmp = None
            
            pathf2mapf = self.__getTFMatrix(Topic.names['/path']._header_frame_id.value,Topic.names['/local_map']._header_frame_id.value)
            yaw = 2 * np.arctan2(Topic.names['/local_map']._info_origin_orientation_z.value, Topic.names['/local_map']._info_origin_orientation_w.value)
            dim2rot = np.array([[np.cos(yaw),np.sin(yaw)], [-np.sin(yaw),np.cos(yaw)]] )
            pt = np.array([[ -Topic.names['/local_map']._info_origin_position_x.value], [ -Topic.names['/local_map']._info_origin_position_y.value]])
            pt = np.dot(dim2rot,pt)
            mapf2map = np.identity(3)
            mapf2map[:2,:2] = dim2rot
            mapf2map[:2,2:3] = pt
            map2img = np.array([[1/Topic.names['/path']._res,0,0],
                            [0,-1/Topic.names['/path']._res,Topic.names['/path']._height]])
            tr = np.dot(mapf2map,pathf2mapf)
            tr = np.dot(map2img,tr)
            for pose in Topic.names['/path']._poses :
                yaw = 2 * np.arctan2(pose.pose.orientation.z, pose.pose.orientation.w)
                pt1 = tuple(np.dot(tr,np.array([[pose.pose.position.x], [pose.pose.position.y],[1]])).astype('int'))
                pt2 = tuple(np.dot(tr,np.array([[pose.pose.position.x + Topic.names['/path']._delta*np.cos(yaw)], [pose.pose.position.y + Topic.names['/path']._delta*np.sin(yaw)],[1]])).astype('int'))
                cv2.arrowedLine(img, pt1, pt2, (0,0,255), 1)
            for pose in Topic.names['/path']._poses :
                if tmp!=None :
                    pt1 = tuple(np.dot(tr,np.array([tmp.pose.position.x, tmp.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                    pt2 = tuple(np.dot(tr,np.array([pose.pose.position.x, pose.pose.position.y,1])).reshape(1,-1)[0].astype('int'))
                    cv2.line(img, pt1,pt2, (0,0,0),1)
                tmp = pose
            img = cv2.resize(img, None, fx = 4, fy = 4, interpolation = cv2.INTER_NEAREST)

            cv2.imshow('overall data', img)
        
        cv2.destroyWindow('overall data')
        
    def __getTFMatrix(self, child_frame, mother_frame) :
        matrix = np.zeros([3,3])
        #print 'mother : ' + mother_frame
        #print 'child : ' + child_frame
        if child_frame == mother_frame :
            matrix = np.identity(3)
        #print Topic.names['/tf']._data.keys()
        if child_frame in Topic.names['/tf']._data.keys() :
        #    print 'child frame in first keys'
        #    print Topic.names['/tf']._data[child_frame].keys()
            if mother_frame in Topic.names['/tf']._data[child_frame].keys() :
        #        print 'child frame in second keys'
                tf = Topic.names['/tf']._data[child_frame][mother_frame]
                yaw = 2 * np.arctan2(tf.transform.rotation.z, tf.transform.rotation.w)
                matrix = np.array([ [np.cos(yaw), -np.sin(yaw), tf.transform.translation.x],
                                    [np.sin(yaw), np.cos(yaw), tf.transform.translation.y],
                                    [0, 0, 1] ])
        if mother_frame in Topic.names['/tf']._data.keys() :
        #    print 'mother frame in first keys'
        #    print Topic.names['/tf']._data[mother_frame].keys()
            if child_frame in Topic.names['/tf']._data[mother_frame].keys() :
        #        print 'mother frame in second keys'
                tf = Topic.names['/tf']._data[mother_frame][child_frame]
                yaw = 2 * np.arctan2(tf.transform.rotation.z, tf.transform.rotation.w)
                tmp = np.array([[np.cos(yaw),np.sin(yaw)], [-np.sin(yaw),np.cos(yaw)]] )
                pt = np.array([[ -tf.transform.translation.x], [ -tf.transform.translation.y]])
                pt = np.dot(tmp,pt)
                matrix = np.identity(3)
                matrix[:2,:2] = tmp
                matrix[:2,2:3] = pt
        if matrix[2][2] == 0 :
        #    print "no tf matrix"
            return np.identity(3)
        else :
            return matrix




if __name__ == '__main__':
    try:
        pyforms.start_app(MainMonitor)
    except rospy.ROSInterruptException:
        pass