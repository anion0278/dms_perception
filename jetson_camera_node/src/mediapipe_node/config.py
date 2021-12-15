from enum import IntEnum
class HandSide(IntEnum):
    UNKNOWN = 0
    RIGHT = 1
    LEFT = 2

hand_recognition_servise = "hand_recognition"
hands_data_topic = "/hands_data"
hands_tracker_node_name = "hands_tracker_"