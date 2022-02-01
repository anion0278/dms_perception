#!/usr/bin/env python3

import cv2
import rospy
import config
import ros_numpy
import numpy as np
import pyrealsense2 as rs
import recognizer as rec
from geometry_msgs.msg import Point
from jetson_camera_node.msg import CameraData, HandData, MultiHandData
from pympler.asizeof import asizeof
from std_msgs.msg import String
from hand_data import Hand
from typing import List
import utils_ip as ip
import sys

ros_cam_data_msg_size = 3575480

class HandRecognizer():
    def __init__(self, debug = True):
        node_suffix = ip.get_eth_ip_address().replace(".","_")
        rospy.init_node(config.hands_tracker_node_name + str(node_suffix))
        print("Started node: %s" % rospy.get_name())
        self.recognizer = rec.MPRecognizer(max_num_hands = 2, debug = debug)
        self.subscriber = rospy.Subscriber("cam_data_processor_"+str(node_suffix)+"/camera_data_" + str(node_suffix), CameraData, self.__process_topic_data, queue_size = 1, 
                                            buff_size= ros_cam_data_msg_size * 2) # this fixes latency problem: https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        self.hands_pub = rospy.Publisher(rospy.get_name() + config.hands_data_topic, MultiHandData, queue_size = 1)

    def run(self):
        print("Started listening...")
        rospy.spin()
    
    def __process_topic_data(self, cameraData):
        print("Recieved camera data")
        #print("Size of ROS message (min buff size): " + str(asizeof(cameraData))) 
        cv_color_img = ros_numpy.numpify(cameraData.color)
        cv_depth_img = np.array(cameraData.depth,dtype=float).reshape(cv_color_img.shape[0],cv_color_img.shape[1])
        intrinsics = self.__get_intrinsics(cameraData.cameraInfo)
        extrinsics = self.__get_extrinsics(cameraData.extRotationMatrix, cameraData.extTranslationVector)
        scale = cameraData.depthScale
        recognized_hands = self.recognizer.recognize_hand(cv_color_img, cv_depth_img, intrinsics, scale, extrinsics)
        self.__publish_hands(recognized_hands)

    def __publish_hands(self, recognized_hands_list:List[Hand]):
        multi_hand_msg = MultiHandData()
        print("============== Recognized hands: ")
        for hand in recognized_hands_list:
            hand_msg = HandData() 
            hand_msg.handSide = String(hand.side.name)
            hand_msg.gestureType = hand.gesture
            hand_msg.confidence = hand.confidence
            for xyz_point in hand.landmarks_3d:
                hand_msg.landmarks.append(Point(x=xyz_point[0],y=xyz_point[1],z=xyz_point[2]))
            multi_hand_msg.recognizedHands.append(hand_msg)
            print("Hand %s %s" % (hand.gesture, hand.side.name))
        self.hands_pub.publish(multi_hand_msg)
        
    def __get_intrinsics(self, camera_info_msg):
        intrinsics = rs.intrinsics()
        intrinsics.width = camera_info_msg.width
        intrinsics.height = camera_info_msg.height
        intrinsics.ppx = camera_info_msg.K[2]
        intrinsics.ppy = camera_info_msg.K[5]
        intrinsics.fx = camera_info_msg.K[0]
        intrinsics.fy = camera_info_msg.K[4] 
        intrinsics.model = rs.distortion.brown_conrady
        intrinsics.coeffs = [i for i in camera_info_msg.D]
        return intrinsics

    def __get_extrinsics(self, extrinsics_rotation, extrinsics_translation):
        extrinsics = rs.extrinsics()
        extrinsics.rotation = extrinsics_rotation
        extrinsics.translation = extrinsics_translation
        #print("Rotation %s" % extrinsics.rotation)
        #print("Translation %s" % extrinsics.translation)
        return extrinsics

if __name__ == '__main__':
    tracker = HandRecognizer()
    tracker.run()




