import numpy as np
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs

class MPRecognizer:
    def __init__(self,model_complexity = 0,max_num_hands = 1,min_detection_confidence = 0.5,min_tracking_confidence = 0.5):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(True,max_num_hands,min_detection_confidence,min_tracking_confidence)
        self.mpDraw = mp.solutions.drawing_utils

    def __recognize(self,image):
        self.result = self.hands.process(image)
        hand_coordinates = []
        if self.result.multi_hand_landmarks:
            rows, cols, _ = image.shape
            for landmarks in self.result.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(image, landmarks, self.mp_hands.HAND_CONNECTIONS)
                for landmark in landmarks.landmark:
                    hand_coordinates.append(self.mp_drawing._normalized_to_pixel_coordinates(landmark.x,landmark.y,rows,cols))
        else:
            return hand_coordinates
        return hand_coordinates

    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics, debug = False):
        hand_coordinates = self.__recognize(color)

        if debug:
            cv2.imshow("MediaPipe image", cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
            cv2.waitKey(2)

        recognized_hands = []
        landmarks = []
        for i in range(len(hand_coordinates)):
            if hand_coordinates[i] is not None:  
                color_img_pixel = [np.clip(int(hand_coordinates[i][0]), 0, color.shape[1] - 1), 
                                np.clip(int(hand_coordinates[i][1]), 0, color.shape[0] - 1)]
                depth_value = depth[color_img_pixel[1], color_img_pixel[0]]
                if i == 8:
                    depth_tf = cv2.cvtColor((depth * 255).astype("uint8"), cv2.COLOR_GRAY2RGB)
                    cv2.circle(depth_tf, color_img_pixel, 2, (255,0,0), thickness=2, lineType=8, shift=0)
                    stack = np.concatenate((cv2.cvtColor(color, cv2.COLOR_RGB2BGR), depth_tf), axis=1)
                    cv2.imshow("Processed RGB + depth", stack)
                    cv2.waitKey(2)
                    print("depth: %s" % depth_value)
                point_rel_to_camera = rs.rs2_deproject_pixel_to_point(intrinsics, color_img_pixel, depth_value)
                point_rel_to_robot = rs.rs2_transform_point_to_point(extrinsics,point_rel_to_camera)
                landmarks.append(point_rel_to_robot)
            else:
                landmarks.append([0,0,0]) # if landmark is not recognized it still needs to be published

        if len(landmarks) > 0:
            gesture = 1
            recognized_hands.append([landmarks, gesture])
            
        return recognized_hands

    





