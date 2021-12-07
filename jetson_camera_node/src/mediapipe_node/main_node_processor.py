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
from hand_data import Hand

pcl_fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1)]

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

max_hand_projection_dist_m = 0.1

class DataAggregateProcessor():
    def __init__(self, node_names):
        rospy.init_node('hand_aggregation_processor')
        self.__init_subscribers(node_names)
        self.pc_communication = MainPcCommunication("192.168.0.149") #"192.168.1.20" PO
        self.left_pcl_publisher = rospy.Publisher("hands_point_clouds/left", PointCloud2, queue_size=1)
        self.right_pcl_publisher = rospy.Publisher("hands_point_clouds/right", PointCloud2, queue_size=1)

    def __init_subscribers(self, node_names):
        if len(node_names) == 0: ValueError("Check node names!")
        subs = []
        for node_name in node_names:
            subs.append(message_filters.Subscriber(node_name + config.hands_data_topic, MultiHandData, queue_size=1))
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=1, slop=0.1)
        self.sync.registerCallback(self.on_sync_data)

    def __get_most_frequent_category_or_first(self, list):
        occurence_count = Counter(list)
        return occurence_count.most_common(1)[0][0]

    def run(self):
        rospy.spin()

    def on_sync_data(self, *msgs_from_all_trackers: List[MultiHandData]):
        left_hands : List[Hand] = []
        right_hands : List[Hand] = []
        pcl_right = []
        pcl_left = []

        for multi_hand_msg, color in zip(msgs_from_all_trackers, colors): # for now we assume that the max number of hands in a single msg is 2
            for hand_data_msg in multi_hand_msg.recognizedHands:
                hand = self.convert_hand_data(hand_data_msg)

                if hand.side.value == config.HandSide.LEFT: # TODO possible to replace with for and custom class
                    left_hands.append(hand)
                    pcl_left += self.get_hand_rgba_point_array(hand, color)

                if hand.side.value == config.HandSide.RIGHT:
                    right_hands.append(hand)
                    pcl_right += self.get_hand_rgba_point_array(hand, color)

        print("Left hands: %s, Right Hands: %s" % (len(left_hands), len(right_hands)))
        left_hands = self.remove_outlier_hands(left_hands)
        right_hands = self.remove_outlier_hands(right_hands)
        print("Filtered left hands: %s, Filtered right Hands: %s" % (len(left_hands), len(right_hands)))

        self.pc_communication.send_hands(self.merge_hands(left_hands), self.merge_hands(right_hands))

    def remove_outlier_hands(self, hands: List[Hand]) -> List[Hand]:
        # hands should belong to the same side! TODO check
        filtered_hands = []
        for evaluated_hand in hands:
            if not self.__is_outlier(evaluated_hand, hands):
                filtered_hands.append(evaluated_hand)
        return filtered_hands

    def __is_outlier(self,evaluated_hand:Hand, all_hand_projections:List[Hand]):
        other_hand_projections = [h for h in all_hand_projections if h != evaluated_hand]
        projections_centroid = np.median([h.cached_centroid for h in other_hand_projections], axis=0)
        centroid_dist = np.linalg.norm(evaluated_hand.cached_centroid - projections_centroid)
        if centroid_dist > max_hand_projection_dist_m:
            return True
        return False

    def convert_hand_data(self, hand_data_msg):
        points3d = []
        for point in hand_data_msg.landmarks:
            points3d.append(ros_numpy.numpify(point))
        hand = Hand([], points3d, hand_data_msg.handSide.data.capitalize(), hand_data_msg.confidence, hand_data_msg.gestureType)
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
    
    def merge_hands(self, merge_hands: List[Hand]) -> Hand: 
        if merge_hands is None or len(merge_hands) == 0: return None

        # hands should belong to the same side! TODO check
        hand_side = merge_hands[0].side 

        # TODO return gesture of most "confident" hand if all categories have same number of occurences
        most_common_gesture = self.__get_most_frequent_category_or_first([h.gesture for h in merge_hands])
        average_confidence = np.mean([h.confidence for h in merge_hands])
        filtered_landmarks = []
        for i in range(len(merge_hands[0].landmarks_3d)):
            singleLandmarkData = []
            for hand in merge_hands:
                singleLandmarkData.append(hand.landmarks_3d[i])
            filtered_landmarks.append(np.median(singleLandmarkData, axis=0))

        return Hand([], filtered_landmarks, hand_side.name.capitalize(), average_confidence, most_common_gesture)

def find_all_hand_tracker_nodes():
    return list(filter(lambda t: t.startswith("/"+ config.hands_tracker_node_name), rosnode.get_node_names()))

if __name__ == "__main__":
    proc = DataAggregateProcessor(find_all_hand_tracker_nodes())
    proc.run()

