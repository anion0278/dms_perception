import numpy as np

class HandData():
    def __init__(self,landmark = np.zeros((21,2)),pos3D = np.zeros((21,3)),side = "Right",confidence = 0.0,gest = 2):
        self.landmark = landmark
        self.pos3D = pos3D
        self.side = side
        self.gest = gest
        self.confidence = confidence
    
    def get_aggregated_depth(self):
        depth = max(self.pos3D,axis=0)[2]