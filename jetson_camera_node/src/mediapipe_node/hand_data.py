import numpy as np

class HandData():
    def __init__(self,landmark = np.zeros(21),side = "Right",confidence = 0.0,gest = 2):
        self.landmark = landmark
        self.side = side
        self.gest = gest
        self.confidence = confidence