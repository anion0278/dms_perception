import numpy as np
import cv2
import mediapipe as mp
import rs_utils2 as rs
import numpy as np

class MPRecognizer:
    def __init__(self,model_complexity = 0,max_num_hands = 1,min_detection_confidence = 0.5,min_tracking_confidence = 0.5):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(True,max_num_hands,model_complexity,min_detection_confidence,min_tracking_confidence)

    def __recognize(self,image):
        self.result = self.hands.process(image)

        if self.result.multi_hand_landmarks:
            rows, cols, _ = image.shape
            hand_coordinates = []
            for landmarks in self.result.multi_hand_landmarks:
               for landmark in landmarks.landmark:
                   hand_coordinates.append(self.mp_drawing._normalized_to_pixel_coordinates(landmark.x,landmark.y,rows,cols))
  
        else:
            return None;
        return hand_coordinates

    def recognize_hand(self,color,depth,intrinsics,scale,extrinsics):
        self.intrinsics = intrinsics
        self.extrinsics = extrinsics
        self.image = image
        self.depth = depth
        self.scale = scale

        self.image.flags.writeable = False
        hand_coordinates = __recognize(image)
        image.flags.writeable = True

        if hand_coordinates:
            for i in range(len(hand_coordinates)):
                if i == 8:
                    u = rs_utils2.transform_px_to_pt(self.intrinsics,[int(hand_coordinates[i][0]),int(hand_coordinates[i][1])],depth[int(hand_coordinates[i][1]),int(hand_coordinates[i][0])],self.scale)
                    coords = rs_utils2.transform_pt_to_base(self.extrinsics,u)
                else:
                    coords = [0,0,0]
            return coords
        else:
            return None

    





