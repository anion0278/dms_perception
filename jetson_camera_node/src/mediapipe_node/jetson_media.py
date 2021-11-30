import numpy as np
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import hand_data as hd
import csv
from model import KeyPointClassifier #tensorflow https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html

class MPRecognizer:
    def __init__(self,model_complexity = 0,max_num_hands = 1,min_detection_confidence = 0.5,min_tracking_confidence = 0.5,debug = False):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(True,max_num_hands,min_detection_confidence,min_tracking_confidence)
        self.keypoint_clasifier = KeyPointClassifier()
        self.debug = debug

        if self.debug:   
            self.keypoint_classifier_labels = ["Open","Close","Pointer","OK"]
            

    def __recognize(self,image,depth,intrinsics,scale,extrinsics):
        height,width,_ = image.shape
        result = self.hands.process(image)
        hands = []

        if result.multi_hand_landmarks is not None:
            for landmarks,handedness in zip(result.multi_hand_landmarks,result.multi_handedness):
                hand_coordinates = []
                hand_3Dcoordinates = []
                relative_landmarks = []
                hand_base_point = self.__clip(landmarks.landmark[0].x, landmarks.landmark[0].y,width,height)

                depth_buffer = []
                #landmark coordinates
                for landmark in landmarks.landmark:
                    #upper clip
                    (x,y) = self.__clip(landmark.x, landmark.y,width,height)
                    relative_landmarks.append(x-hand_base_point[0])
                    relative_landmarks.append(y-hand_base_point[1])
                    hand_coordinates.append((x,y))
                    #3D coordinates
                    depth_buffer.append(depth[y,x])

                depth_median = np.median(depth_buffer, axis=0)
                for x,y in hand_coordinates:
                    pos3D = self.__get_depth_in_WCS(x,y,depth_median,intrinsics,scale,extrinsics)
                    hand_3Dcoordinates.append(pos3D)

                #normalized relative coordinates for gesture recognition
                max_value = max(list(map(abs,relative_landmarks)))
                normalized_relative_landmarks = list(map(lambda n: n/max_value,relative_landmarks))    
                #hand additional info
                side = handedness.classification[0].label
                confidence = handedness.classification[0].score
                gesture = self.keypoint_clasifier(normalized_relative_landmarks)

                hand = hd.HandData(hand_coordinates,hand_3Dcoordinates,side,confidence,gesture)
                hands.append(hand)
        return hands

    def __clip(self,x,y,width,height):
        cx = min(int(x*width),width-1)
        cy = min(int(y*height),height-1)
        return cx,cy

    def __get_depth_in_WCS(self,x,y,depth_value,intrinsics,scale,extrinsics):
        point_rel_to_camera = rs.rs2_deproject_pixel_to_point(intrinsics, (x,y), depth_value) #depth value multiply with scale!!
        (x3D,y3D,z3D) = rs.rs2_transform_point_to_point(extrinsics,point_rel_to_camera)
        return x3D,y3D,z3D

    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics):
        hands = self.__recognize(color,depth,intrinsics,scale,extrinsics)
        
        if self.debug:
            depth_tf = cv2.cvtColor((depth * 255).astype("uint8"), cv2.COLOR_GRAY2RGB)
        
            for hand in hands:
                index_point = hand.landmark[8]
                cv2.circle(depth_tf, index_point, 2, (255,255,0), thickness=2, lineType=8, shift=0)
                cv2.circle(color, index_point, 2, (255,255,0), thickness=2, lineType=8, shift=0)
                cv2.putText(color,self.keypoint_classifier_labels[hand.gest],index_point,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
                #cv2.putText(color,hand.side.name,index_point,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
            stack = np.concatenate((cv2.cvtColor(color, cv2.COLOR_RGB2BGR), depth_tf), axis=1)
            cv2.imshow("Processed RGB + depth", stack)
            cv2.waitKey(2)
            #print("depth: %s" % depth_value)
        return hands


