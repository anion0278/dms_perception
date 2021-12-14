import numpy as np
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import hand_data as hd
from typing import List
from model import KeyPointClassifier #tensorflow https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html

class MPRecognizer:
    def __init__(self, model_complexity = 1, max_num_hands = 2,min_detection_confidence = 0.5,min_tracking_confidence = 0.5,debug = False):
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
                
                # all (!) landmarks are always available, because MP assumes their positions even when they are not visible on the camera image  
                for landmark in landmarks.landmark: 
                    (x,y) = self.__upper_clip(landmark.x, landmark.y,width,height)
                    relative_landmarks.append(x-hand_base_point[0])
                    relative_landmarks.append(y-hand_base_point[1])
                    hand_2d_coordinates.append((x,y))
                    points_in_cam_frame.append(self.__get_point_in_camera_frame(x,y, cv_depth_image, intrinsics))

                hand_3d_coordinates = self.get_points_in_robot_frame(extrinsics, points_in_cam_frame)

                #normalized relative coordinates for gesture recognition
                max_value = max(list(map(abs,relative_landmarks)))
                normalized_relative_landmarks = list(map(lambda n: n/max_value,relative_landmarks))    
                #hand additional info
                side = handedness.classification[0].label
                confidence = handedness.classification[0].score
                gesture = self.keypoint_clasifier(normalized_relative_landmarks)
                hand = hd.Hand(hand_2d_coordinates,hand_3d_coordinates,side,confidence,gesture)
                hands.append(hand)
        return hands

    def get_points_in_robot_frame(self, extrinsics, points_in_cam_frame):
        hand_3d_coordinates = []
        z_axis_index = 2
        depth_median = np.median(points_in_cam_frame, axis=0)[z_axis_index] 
        for point_in_cam_frame in points_in_cam_frame:
            point_in_cam_frame[z_axis_index] = depth_median # set filtered Z value
            point_in_robot_frame = rs.rs2_transform_point_to_point(extrinsics, point_in_cam_frame)
            hand_3d_coordinates.append(point_in_robot_frame)
        return hand_3d_coordinates

    def __get_point_in_camera_frame(self, x, y, depth_img, intrinsics):
        depth_in_point = depth_img[y,x] # YX is the correct sequence!
        return rs.rs2_deproject_pixel_to_point(intrinsics, (x,y), depth_in_point)

    def __upper_clip(self,x,y,width,height):
        cx = min(int(x*width),width-1)
        cy = min(int(y*height),height-1)
        return cx,cy

    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics):
        hands = self.__recognize(color,depth,intrinsics,scale,extrinsics)
        
        if self.debug:
            depth_tf = cv2.cvtColor((depth * 255).astype("uint8"), cv2.COLOR_GRAY2RGB)
        
            for hand in hands:
                index_point = hand.landmarks_2d[8]
                cv2.circle(depth_tf, index_point, 2, (255,255,0), thickness=2, lineType=8, shift=0)
                cv2.circle(color, index_point, 2, (255,255,0), thickness=2, lineType=8, shift=0)
                cv2.putText(color,self.keypoint_classifier_labels[hand.gesture],index_point,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
                #cv2.putText(color,hand.side.name,index_point,cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
            stack = np.concatenate((cv2.cvtColor(color, cv2.COLOR_RGB2BGR), depth_tf), axis=1)
            cv2.imshow("[AS] Processed RGB + depth", stack)
            cv2.waitKey(2)
            #print("depth: %s" % depth_value)
        return hands


