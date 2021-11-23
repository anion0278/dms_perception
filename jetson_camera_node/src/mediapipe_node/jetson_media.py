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

    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics):
        hand_coordinates = self.__recognize(color)

        cv2.imshow("MediaPipe image", cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
        cv2.waitKey(2)

        recognized_hands = []
        landmarks = []
        for i in range(len(hand_coordinates)):
            if hand_coordinates[i] is not None:  
                u = rs.rs2_deproject_pixel_to_point(intrinsics,
                        [int(hand_coordinates[i][0]), int(hand_coordinates[i][1])],
                        depth[int(hand_coordinates[i][1]),int(hand_coordinates[i][0])])
                point = rs.rs2_transform_point_to_point(extrinsics,u)
                landmarks.append(point)
            else:
                landmarks.append([0,0,0]) # if landmark is not recognized it still needs to be published

        if len(landmarks) > 0:
            gesture = 1
            recognized_hands.append([landmarks, gesture])
            
        return recognized_hands

    





