#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from jetson_camera_node.srv import ImageRec, ImageRecRequest, ImageRecResponse
from cv_bridge import CvBridge
#import ros_numpy  #sudo apt-get install ros-melodic-ros-numpy
import config

class CameraGrabber:
    def __init__(self):
        rospy.init_node('camera_grabber') # multiple nodes with the same name may exist
        self.__hand_recognition_service = rospy.ServiceProxy(config.hand_recognition_servise, ImageRec)
        #self.__pub = rospy.Publisher('', String)
        self.__capture = cv2.VideoCapture(2)
        self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)
        self.__cv_bridge = CvBridge()

    def run(self):
        print("Waiting for service...")
        rospy.wait_for_service(config.hand_recognition_servise)
        rospy.sleep(1.0)
        #print("Got service!")
        while not rospy.is_shutdown():
            success, img = self.__capture.read()
            
            imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_msg = self.__cv_bridge.cv2_to_imgmsg(imgRGB)
            print("Sending request")
            recognized_img_msg = self.__hand_recognition_service(ImageRecRequest(img_msg))
            recognized_img = self.__cv_bridge.imgmsg_to_cv2(recognized_img_msg.output)
            print("Recieved response")
            imgBGR = cv2.cvtColor(recognized_img, cv2.COLOR_RGB2BGR)
            cv2.imshow("Processed image", imgBGR)
            if cv2.waitKey(2) & 0xFF == 27:
                cap.release()
                break
            #rospy.sleep(1.0)


if __name__ == '__main__':
    cam = CameraGrabber()
    cam.run()
