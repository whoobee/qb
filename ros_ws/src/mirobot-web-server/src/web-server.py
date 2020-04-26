#!/usr/bin/env python

import SimpleHTTPServer 
import SocketServer 
import os

import socket
import fcntl
import struct
import netifaces

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

def get_lan_ip_address():
    lan_ip = ""
    ip = ""
    for interface in netifaces.interfaces():
        try:
            ip = get_ip_address(interface)
            #ignore localhost
            if (ip != "127.0.0.1"):
                lan_ip = ip
                break
        except:
            continue
    return (lan_ip)

PORT = 8000
HOST = get_lan_ip_address()

web_dir = os.path.join(os.path.dirname(__file__), 'web/')
os.chdir(web_dir)

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler 

httpd = SocketServer.TCPServer((HOST, PORT), Handler) 

print "serving at address:", HOST 
print "serving at port:", PORT 
print "web dir:", web_dir 
httpd.serve_forever()