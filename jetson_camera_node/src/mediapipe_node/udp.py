import socket
import struct
from typing import List

from hand_data import Hand

class UDPClient:
    def __init__(self,addr,port):
        self.addr = (addr,port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_floats(self, buffer):
        buf = struct.pack('%sf' % len(buffer), *buffer)
        self.socket.sendto(buf,self.addr)

    def close(self):
        self.socket.detach()
        self.socket.close()

class MainPcCommunication():
    def __init__(self, ip):
        self.udp_com = UDPClient(ip, 4023) 
    
    def send_hands(self, left_hand:Hand, right_hand:Hand):
        udp_msg = [*self.__format_hand_data(left_hand), *self.__format_hand_data(right_hand)]
        self.udp_com.send_floats(udp_msg)

    def __format_hand_data(self, hand: Hand) -> List[float]:
        if hand is None: return [0,0,0,-1] # empty hand TK convention
        index_finger_counter = 8
        return [*hand.landmarks_3d[index_finger_counter], hand.gesture]
