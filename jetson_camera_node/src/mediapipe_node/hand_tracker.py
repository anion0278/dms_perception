#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import rospy
import config
import ros_numpy
import numpy as np
import pyrealsense2 as rs
import jetson_media as jm
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from jetson_camera_node.srv import ImageRec, ImageRecRequest, ImageRecResponse
from jetson_camera_node.msg import CameraData, HandData, MultiHandData
from pympler.asizeof import asizeof

ros_cam_data_msg_size = 421248

class HandRecognizer():
    def __init__(self):
        rospy.init_node("hand_tracker")
        self.recognizer = jm.MPRecognizer(debug = True)
        self.subscriber = rospy.Subscriber("camera_data", CameraData, self.__process_topic_data, queue_size = 1, 
                                            buff_size= ros_cam_data_msg_size * 2) # fixes latency problem: https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
        self.hands_pub = rospy.Publisher(rospy.get_name() + config.hands_data_topic, MultiHandData, queue_size = 1)

    def run(self):
        rospy.spin()

    def __process_topic_data(self, cameraData):
        #print("Size of ROS message (min buff size): " + str(asizeof(cameraData))) # currently 421248
        cv_color_img = ros_numpy.numpify(cameraData.color)
        cv_depth_img = cv2.resize(ros_numpy.numpify(cameraData.depth), (cv_color_img.shape[1], cv_color_img.shape[0]))
        #cv2.imshow(
        #    "Processed RGB image", 
        #    np.concatenate((cv2.cvtColor(cv_color_img, cv2.COLOR_RGB2BGR), cv2.cvtColor(cv_depth_img, cv2.COLOR_GRAY2RGB)),
        #    axis=1))
        #cv2.waitKey(2)
        cv_depth_img = cv_depth_img / 255.0
        intrinsics = self.__get_intrinsics(cameraData.cameraInfo)
        extrinsics = self.__get_extrinsics(cameraData.extRotationMatrix, cameraData.extTranslationVector)
        scale = cameraData.depthScale
        recognized_hands = self.recognizer.recognize_hand(cv_color_img, cv_depth_img, intrinsics, scale, extrinsics)
        self.__publish_hands(recognized_hands)

    def __publish_hands(self, recognized_hands_list):
        multi_hand_msg = MultiHandData()
        for hand in recognized_hands_list:
            hand_msg = HandData() 
            hand_msg.handSide = config.HandSide.LEFT.value
            hand_msg.gestureType = hand.gest
            landmarks = hand.pos3D
            for i in range(len(landmarks)):
                xyz_point = landmarks[i]
                hand_msg.landmarks.append(Point(x=xyz_point[0],y=xyz_point[1],z=xyz_point[1]))
            multi_hand_msg.recognizedHands.append(hand_msg)
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
        #extrinsics.rotation = [-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0]
        #extrinsics.translation = [0.0,-0.37,0.94]
        extrinsics.rotation = [1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,-1.0]
        extrinsics.translation = [0.062,-0.37,0.94]
        return extrinsics

if __name__ == '__main__':
    tracker = HandRecognizer()
    tracker.run()




