#!/usr/bin/env python3

import socket
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', bytes(ifname[:15], 'utf-8'))
    )[20:24])

def get_eth_ip_address():
    try:
        return get_ip_address('eth0')
    except:
        return get_ip_address('lo')
    #except Exception as ex:
        #default_ip = "127.0.0.0" # for some reason localhost IP is different on PO side
        #print("Could not get IP, setting defaut IP: %s" % default_ip)
        #return default_ip
