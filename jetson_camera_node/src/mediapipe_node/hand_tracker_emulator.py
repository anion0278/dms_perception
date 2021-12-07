#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import rospy
import config
from std_msgs.msg import Header
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from jetson_camera_node.msg import CameraData, HandData, MultiHandData
from std_msgs.msg import String

def get_test_data():
    multi_hand_data = MultiHandData()
    for hand in range(2):
        landmarks = []
        for i in range(21):
            hand_offset = hand * 0.05
            landmarks.append(Point(x=1 + hand_offset,y=2 + hand_offset,z=3 + hand_offset))
        gesture = 1
        hand_data = HandData() # TODO extract from hand_tracker !!!
        hand_data.landmarks = landmarks
        hand_data.gestureType = gesture
        hand_data.handSide = String(config.HandSide.LEFT.name)
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
    print("published")
    rospy.sleep(0.2)