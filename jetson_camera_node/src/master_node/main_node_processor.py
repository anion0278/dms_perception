#!/usr/bin/env python3

import time
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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from typing import List
 
class DataAggregateProcessor():
    def __init__(self, node_names):
        rospy.init_node('hand_aggregation_processor')
        self.__init_cam_subs(node_names)
        self.pc_communication = MainPcCommunication("192.168.0.149") #"192.168.1.20" PO
        self.left_pcl_publisher = rospy.Publisher("hands_point_clouds/left", PointCloud2, queue_size=1)
        self.right_pcl_publisher = rospy.Publisher("hands_point_clouds/right", PointCloud2, queue_size=1)

    def __init_cam_subs(self, node_names):
        if len(node_names) == 0: ValueError("Check node names!")
        subs = []
        for node_name in node_names:
            subs.append(message_filters.Subscriber(node_name + "/hands_data", MultiHandData, queue_size=1))
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=1, slop=0.1)
        self.sync.registerCallback(self.on_sync_data)

    def __get_most_frequent_category(self, list):
        occurence_count = Counter(list)
        return occurence_count.most_common(1)[0][0]

    def run(self):
        rospy.spin()

    def on_sync_data(self, *sensors_msgs: List[MultiHandData]):
        hands = sensors_msgs[0].recognizedHands
        # TVORIT POINT CLOUD
        if len(hands) > 0:
            index_point = ros_numpy.numpify(hands[0].landmarks[8])
            filtered_hands_data = [*index_point, float(hands[0].gestureType)]
            print(filtered_hands_data)
            self.pc_communication.send_hand_data(filtered_hands_data, filtered_hands_data)
            return
        print("recieved empty data msg")
        
    def create_point_cloud(self, cloud_points: List[List[float]]) -> PointCloud2:
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        return pcl2.create_cloud_xyz32(header, cloud_points)
    
    def merge_hand_data(self, merge_data_list: List[HandData]) -> HandData: # unused for now
        gestureTypeList = []
        handSideList = []
        landmarksList = []
        for i in range(21):
            singlePointList = []
            for hand in merge_data_list:
                if i == 8:
                    gestureTypeList.append(hand.gestureType)
                    handSideList.append(hand.handSide)
                singlePointList.append(ros_numpy.numpify(hand.landmarks[i]))
            landmarksList.append(np.median(singlePointList, axis=0))
        # TODO handle empty lists
        gestureType = self.__get_most_frequent_category(gestureTypeList)
        handSide = self.__get_most_frequent_category(handSideList)
        return handSide, gestureType, landmarksList

if __name__ == "__main__":
    hand_tracker_nodes = list(filter(lambda t: t.startswith("/hands_tracker"), rosnode.get_node_names()))
    proc = DataAggregateProcessor(hand_tracker_nodes)
    proc.run()

