#!/usr/bin/env python3

import time

from numpy.lib.function_base import average
import rospy
import numpy as np
import ros_numpy
from geometry_msgs.msg import Point
from jetson_camera_node.msg import CameraData, HandData, MultiHandData
import message_filters
from udp import MainPcCommunication
from collections import Counter
import rosnode
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from typing import List
import struct
import config
# import tf2_ros as tf
# import tf_conversions
from hand_data import Hand
import sys

pcl_fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1)]

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

max_hand_projection_dist_m = 0.1

class DataAggregateProcessor():
    def __init__(self, node_names, debug = True):
        rospy.init_node('hand_aggregation_processor')
        self.__init_subscribers(node_names)
        self.pc_communication = MainPcCommunication("192.168.1.20") 
        self.left_pcl_publisher = rospy.Publisher("hands_point_clouds/left", PointCloud2, queue_size=1)
        self.right_pcl_publisher = rospy.Publisher("hands_point_clouds/right", PointCloud2, queue_size=1)
        # self.tf_pub = tf.TransformBroadcaster()

    def __init_subscribers(self, node_names):
        if len(node_names) == 0: ValueError("Check node names!")
        subs = []
        for node_name in node_names:
            topic_name = node_name + config.hands_data_topic
            subs.append(message_filters.Subscriber(topic_name, MultiHandData, queue_size=1))
            print("Added source topic: %s" % topic_name)
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=1, slop=0.5)
        self.sync.registerCallback(self.on_sync_data)

    def run(self):
        rospy.spin()

    def on_sync_data(self, *msgs_from_all_trackers: List[MultiHandData]):
        left_hands : List[Hand] = []
        right_hands : List[Hand] = []
        pcl_right = []
        pcl_left = []

        override_gesture_right = None
        override_gesture_left = None

        for multi_hand_msg, color, index in zip(msgs_from_all_trackers, colors, range(len(msgs_from_all_trackers))): # for now we assume that the max number of hands in a single msg is 2
            for hand_data_msg in multi_hand_msg.recognizedHands:
                hand = self.convert_hand_data(hand_data_msg, index)
            
                if hand.side.value == config.HandSide.LEFT: # TODO possible to replace with for and custom class
                    left_hands.append(hand)
                    pcl_left += self.get_hand_rgba_point_array(hand, color)
                    if hand.cam_index == 0:
                        override_gesture_left = hand.gesture

                if hand.side.value == config.HandSide.RIGHT:
                    right_hands.append(hand)
                    pcl_right += self.get_hand_rgba_point_array(hand, color)
                    if hand.cam_index == 0:
                        override_gesture_right = hand.gesture

        print("Left hands: %s, Right Hands: %s" % (len(left_hands), len(right_hands)))
        left_hands = self.remove_outlier_hands(left_hands)
        right_hands = self.remove_outlier_hands(right_hands)
        print("Filtered left hands: %s, Filtered right Hands: %s" % (len(left_hands), len(right_hands)))
        left_hand_final = self.merge_hands(left_hands, override_gesture_left)
        right_hand_final = self.merge_hands(right_hands, override_gesture_right)

        self.pc_communication.send_hands(left_hand_final, right_hand_final)
        self.left_pcl_publisher.publish(self.create_point_cloud(pcl_left))
        # self.tf_pub.sendTransform(left_hand_final.cached_centroid,
        #                     tf_conversions.quaternion_from_euler(0,0,0),
        #                     rospy.Time.now(),
        #                     "left_hand"
        #                     "world")
        self.right_pcl_publisher.publish(self.create_point_cloud(pcl_right))
        # self.tf_pub.sendTransform(right_hand_final.cached_centroid,
        #                     tf_conversions.quaternion_from_euler(0,0,0),
        #                     rospy.Time.now(),
        #                     "left_hand"
        #                     "world")

    def remove_outlier_hands(self, hands: List[Hand]) -> List[Hand]:
        # hands should belong to the same side! TODO check
        filtered_hands = []
        for evaluated_hand in hands:
            if not self.__is_outlier(evaluated_hand, hands):
                filtered_hands.append(evaluated_hand)
        return filtered_hands

    def __is_outlier(self,evaluated_hand:Hand, all_hand_projections:List[Hand]):
        mean = np.mean([h.cached_centroid for h in all_hand_projections])
        std = np.std([h.cached_centroid for h in all_hand_projections])
        z_score = (evaluated_hand.cached_centroid - mean) / std
        z_score = np.abs(np.linalg.norm(z_score)) 
        if z_score > 2:
            print("Outlier: %s" % z_score)
            return True
        return False

    def convert_hand_data(self, hand_data_msg, cam_index):
        points3d = []
        for point in hand_data_msg.landmarks:
            points3d.append(ros_numpy.numpify(point))
        hand = Hand([], points3d, config.HandSide[hand_data_msg.handSide.data.upper()], hand_data_msg.confidence, hand_data_msg.gestureType)
        hand.cam_index = cam_index
        hand.update_centroid()
        return hand
    
    def get_hand_rgba_point_array(self, hand: Hand, color):
        pcl_data: List[List[float]] = []
        for point3d in hand.landmarks_3d:
            rgba = struct.unpack('I', struct.pack('BBBB', *color, int(hand.confidence * 255)))[0]
            pcl_data.append([*point3d, rgba])
        return pcl_data

    def create_point_cloud(self, cloud_points: List[List[float]]) -> PointCloud2:
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        return pcl2.create_cloud(header, pcl_fields, cloud_points)
    
    def merge_hands(self, merge_hands: List[Hand], override_gesture = None) -> Hand: 
        if merge_hands is None or len(merge_hands) == 0: return None

        # hands should belong to the same side! TODO check
        hand_side = merge_hands[0].side 
        most_common_gesture = self.filter_gesture(merge_hands)
        average_confidence = np.mean([h.confidence for h in merge_hands])
        filtered_landmarks = []
        for i in range(len(merge_hands[0].landmarks_3d)):
            singleLandmarkData = []
            for hand in merge_hands:
                singleLandmarkData.append(hand.landmarks_3d[i])
            filtered_landmarks.append(np.median(singleLandmarkData, axis=0))

        if override_gesture is not None:
            most_common_gesture = override_gesture

        return Hand([], filtered_landmarks, hand_side.name.capitalize(), average_confidence, most_common_gesture)

    def filter_gesture(self, hands: List[Hand]): 
        gestures = [h.gesture for h in hands]

        if len(self.__get_unique_items(gestures)) == len(hands):
            most_stable_hand = max(hands, key = lambda h:h.confidence)
            return most_stable_hand.gesture

        # TODO if both multiple gestures have same number of occurences, then select the one with the highest confidence
        return self.__get_most_frequent_category_or_first(gestures)

    def __get_unique_items(self, gestures):
        return list(set(gestures))

    def __get_most_frequent_category_or_first(self, list):
        occurence_count = Counter(list)
        return occurence_count.most_common(1)[0][0]

def find_all_hand_tracker_nodes():
    return sorted(list(filter(lambda t: t.startswith("/"+ config.hands_tracker_node_name), rosnode.get_node_names())))

if __name__ == "__main__":
    proc = DataAggregateProcessor(find_all_hand_tracker_nodes())
    proc.run()

