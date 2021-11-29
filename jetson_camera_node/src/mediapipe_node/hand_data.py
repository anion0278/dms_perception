import numpy as np
import config as c

class HandData():
    def __init__(self,landmark,pos3D,side,confidence,gest):
        self.landmark = landmark
        self.pos3D = pos3D
        self.side = {"Left":c.HandSide.LEFT,"Right":c.HandSide.RIGHT}.get(side,c.HandSide.UNKNOWN)
        self.gest = gest
        self.confidence = confidence
    
    def get_aggregated_depth(self):
        depth = max(self.pos3D,axis=0)[2]