import socket
import struct

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
    
    def send_hand_data(self, left_hand_data, right_hand_data):
        udp_msg = [*left_hand_data, *right_hand_data]
        self.udp_com.send_floats(udp_msg)