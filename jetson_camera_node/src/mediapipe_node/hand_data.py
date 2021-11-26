import numpy as np

class HandData():
    def __init__(pos = np.zeros(21),side = 0,gest = 2,confidence = 0.0f):
        self.pos = pos
        self.side = side
        self.gest = gest
        self.confidence = confidence