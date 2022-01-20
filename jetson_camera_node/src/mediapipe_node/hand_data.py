import numpy as np
import config as c

class Hand():
    def __init__(self,landmarks_2d,landmarks_3d,side,confidence,gesture):
        self.landmarks_2d = landmarks_2d
        self.landmarks_3d = landmarks_3d
        self.side = side
        self.gesture = int(gesture)
        self.confidence = confidence
        self.cached_centroid = None 
        self.cam_index = None
    
    def filter_depth(self):
        depth = max(self.landmarks_3d, axis=0)[2]

    def update_centroid(self):
        # all (!) landmarks are always available, because MP assumes their positions even when they are not visible on the camera image  
        self.cached_centroid = np.median(self.landmarks_3d, axis=0)