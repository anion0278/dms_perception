import numpy as np

class HandData():
    def __init__(self,pos = np.zeros(21),side = "Right",confidence = 0.0,gest = 2):
        self.pos = pos
        self.side = side
        self.gest = gest
        self.confidence = confidence