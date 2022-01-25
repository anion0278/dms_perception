import numpy as np
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import hand_data as hd
from typing import List
import config as c
from model import KeyPointClassifier #tensorflow https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html

class MPRecognizer:
    def __init__(self, model_complexity = 0, max_num_hands = 2,min_detection_confidence = 0.5,min_tracking_confidence = 0.5,debug = False):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity = model_complexity, 
                                        static_image_mode = False,
                                        max_num_hands = max_num_hands,
                                        min_detection_confidence = min_detection_confidence,
                                        min_tracking_confidence = min_tracking_confidence)
        self.keypoint_clasifier = KeyPointClassifier()
        self.debug = debug

        if self.debug:   
            self.keypoint_classifier_labels = ["Open","Close","Pointer","OK"]
            
    def __recognize(self, cv_rgb_image, cv_depth_image, intrinsics, scale: float, extrinsics) -> List[hd.Hand]:
        height,width,_ = cv_rgb_image.shape
        result = self.hands.process(cv_rgb_image)
        hands = []

        if result.multi_hand_landmarks is not None:
            for landmarks,handedness in zip(result.multi_hand_landmarks,result.multi_handedness):
                hand_2d_coordinates = []
                relative_landmarks = []
                points_in_cam_frame = []

                hand_base_point = self.__upper_clip(landmarks.landmark[0].x, landmarks.landmark[0].y,width,height)
                #smecko
                hand_depth_history = [cv_depth_image[hand_base_point[1],hand_base_point[0]]]
                #endsmecko
                # all (!) landmarks are always available, because MP assumes their positions even when they are not visible on the camera image  
                for i,landmark in enumerate(landmarks.landmark): #enumerate kvuli smecka
                    (x,y) = self.__upper_clip(landmark.x, landmark.y,width,height)
                    relative_landmarks.append(x-hand_base_point[0])
                    relative_landmarks.append(y-hand_base_point[1])
                    hand_2d_coordinates.append((x,y))

                    #smecko
                    if i in [5, 9, 13, 17]:
                        if abs(hand_depth_history-cv_depth_image[y,x])<0.1:
                            hand_depth_history = [cv_depth_image[y,x]]
                    #endsmecko

                    points_in_cam_frame.append(self.__get_point_in_camera_frame(x,y,hand_depth_history,cv_depth_image, intrinsics))

                hand_3d_coordinates = self.get_points_in_robot_frame(extrinsics, points_in_cam_frame)

                #normalized relative coordinates for gesture recognition
                max_value = max(list(map(abs,relative_landmarks)))
                normalized_relative_landmarks = list(map(lambda n: n/max_value,relative_landmarks))    
                #hand additional info
                side = {"Right":c.HandSide.LEFT,"Left":c.HandSide.RIGHT}.get(handedness.classification[0].label,c.HandSide.UNKNOWN) 
                confidence = handedness.classification[0].score
                gesture = self.keypoint_clasifier(normalized_relative_landmarks)
                hand = hd.Hand(hand_2d_coordinates,hand_3d_coordinates,side,confidence,gesture)
                hands.append(hand)
        return hands

    def get_points_in_robot_frame(self, extrinsics, points_in_cam_frame, use_z_filter=False):
        hand_3d_coordinates = []
        z_axis_index = 2
        depth_median = np.median(points_in_cam_frame, axis=0)[z_axis_index] 
        for point_in_cam_frame in points_in_cam_frame:
            if use_z_filter:
                point_in_cam_frame[z_axis_index] = depth_median # set filtered Z value
            point_in_robot_frame = rs.rs2_transform_point_to_point(extrinsics, point_in_cam_frame)
            hand_3d_coordinates.append(point_in_robot_frame)
        return hand_3d_coordinates

    def __get_point_in_camera_frame(self, x, y,hand_depth_history, depth_img, intrinsics):
        depth_in_point = depth_img[y,x] # YX is the correct sequence!
        #smecko
        if abs(hand_depth_history[0]-depth_in_point)>0.02:
            depth_in_point = hand_depth_history[0]
        else:
            hand_depth_history[0] = depth_in_point
        #endsmecko
        return rs.rs2_deproject_pixel_to_point(intrinsics, (x,y), depth_in_point) 

    def __upper_clip(self,x,y,width,height):
        cx = min(int(x*width),width-1)
        cy = min(int(y*height),height-1)
        return cx,cy

    def __norm_depth(self,depth):
        depth[depth>c.camera_max_distance] = c.camera_max_distance #crop to max distance
        norm = lambda n: n/c.camera_max_distance
        depth = norm(depth)
        return (depth*255).astype("uint8")


    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics):
        hands = self.__recognize(color,depth,intrinsics,scale,extrinsics)
        
        if self.debug:
            depth_tf = cv2.cvtColor(self.__norm_depth(depth), cv2.COLOR_GRAY2RGB)
        
            for hand in hands:
                for i,landmark in enumerate(hand.landmarks_2d):
                    cv2.circle(depth_tf, landmark, 1, (255,255,255), thickness=2, lineType=8, shift=0)
                    cv2.circle(color, landmark, 1, (255,255,255), thickness=2, lineType=8, shift=0)
                    if i == 8:
                        cv2.circle(color, landmark, 2, (255,255,0), thickness=2, lineType=8, shift=0)
                        cv2.putText(color,self.keypoint_classifier_labels[hand.gesture],landmark,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
                #cv2.putText(color,hand.side.name,index_point,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
            stack = np.concatenate((cv2.cvtColor(color, cv2.COLOR_RGB2BGR), depth_tf), axis=1)
            cv2.imshow("[AS] Processed RGB + depth", stack)
            cv2.waitKey(2)
            #print("depth: %s" % depth_value)
        return hands


