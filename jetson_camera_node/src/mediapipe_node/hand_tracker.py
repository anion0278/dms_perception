#!/usr/bin/env python3

import cv2
import mediapipe as mp
import time
import rospy
import config
import ros_numpy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetson_camera_node.srv import ImageRec, ImageRecRequest, ImageRecResponse
from jetson_camera_node.msg import CameraData


class HandRecognizer():
    def __init__(self):
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=False, max_num_hands=4,  min_detection_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils
        # Frame rate statistics
        self.pTime = 0
        self.cTime = 0
        self.__cv_bridge = CvBridge()

    def run(self):
        rospy.init_node('hand_tracker')
        self.service = rospy.Service(config.hand_recognition_servise, ImageRec, self.process_service_request)
        self.subscriber = rospy.Subscriber("camera_data", CameraData, self.process_topic_data)
        rospy.spin()

    def process_topic_data(self, cameraData):
        cv_color_img = ros_numpy.numpify(cameraData.color)
        cv_depth_img = ros_numpy.numpify(cameraData.depth)
        cv_depth_img = cv2.cvtColor(cv2.resize(cv_depth_img, (320, 240)), cv2.COLOR_GRAY2RGB)
        result_img = self.__image_processing(cv_color_img, cv_depth_img)
        cv2.imshow("Processed RGB image", np.concatenate(
            (cv2.cvtColor(result_img, cv2.COLOR_RGB2BGR), cv_depth_img), 
            axis=1))
        cv2.waitKey(2)
        #print("Processed camera data")

    def process_service_request(self, request):
        print("Recieved request")
        #img = __cv_bridge.imgmsg_to_cv2(request.input) # works only for Python2 !!!
        cv_color_img = ros_numpy.numpify(request.input)
        cv_depth_img = None
        result_img = self.__image_processing(cv_color_img, cv_depth_img)
        print("Responding...")
        return ImageRecResponse(self.__cv_bridge.cv2_to_imgmsg(result_img))
        

    def __image_processing(self, cv_color_img, cv_depth_img):
        results = self.hands.process(cv_color_img)
        #print(results.multi_hand_landmarks)
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    #print(id, lm)
                    # Get finger joint points
                    h, w, c = cv_color_img.shape
                    cx, cy = int(lm.x*w), int(lm.y*h)
                    #cv2.putText(img, str(int(id)), (cx+10, cy+10), cv2.FONT_HERSHEY_PLAIN,1, (0, 0, 255), 2)
                self.mpDraw.draw_landmarks(cv_color_img, handLms, self.mpHands.HAND_CONNECTIONS)
        # Count screen frame rate
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        cv2.putText(cv_color_img, "FPS: " + str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        return cv_color_img

if __name__ == '__main__':
    tracker = HandRecognizer()
    tracker.run()




