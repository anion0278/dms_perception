#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import rospy
import config
import ros_numpy
from std_msgs.msg import Header
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from jetson_camera_node.srv import ImageRec, ImageRecRequest, ImageRecResponse
from jetson_camera_node.msg import CameraData, HandData, MultiHandData


def get_test_data():
    multi_hand_data = MultiHandData()
    for hand in range(3):
        landmarks = []
        for i in range(21):
            landmarks.append(Point(x=1,y=2,z=3))
        gesture = 1
        hand_data = HandData()
        hand_data.landmarks = landmarks
        hand_data.gestureType = gesture
        hand_data.handSide = config.HandSide.LEFT.value
        multi_hand_data.recognizedHands.append(hand_data)
        multi_hand_data.header = Header()
        multi_hand_data.header.stamp = rospy.Time.now()
        multi_hand_data.header.frame_id = "world"
    return multi_hand_data

rospy.init_node("hand_tracker") # name is overriden during launching from roslaunch
hands_pub = rospy.Publisher(rospy.get_name() + config.hands_data_topic, MultiHandData, queue_size = 1)
while not rospy.is_shutdown():
    data = get_test_data()
    hands_pub.publish(data)
    rospy.sleep(0.2)