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
    def __init__(self):
        #self.udp_com = UDPClient("192.168.0.2", 4023)
        self.udp_com = UDPClient("169.254.59.148", 4023)
    
    def send_hand_data(self, hand_data):
        udp_msg = hand_data[0] 
        self.udp_com.send_floats(udp_msg)