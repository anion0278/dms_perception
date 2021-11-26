import numpy as np

class HandData():
    def __init__(self,pos = np.zeros(21),side = "Right",gest = 2,confidence = 0.0):
        self.pos = pos
        self.side = side
        self.gest = gest
        self.confidence = confidence