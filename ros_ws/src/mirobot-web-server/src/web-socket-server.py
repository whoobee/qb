#!/usr/bin/env python3

# WS server example

import threading
import os
import signal
import subprocess

from http.server import CGIHTTPRequestHandler, HTTPServer
from websocket_server import WebsocketServer
import json

import socket
import fcntl
import struct
import netifaces


class NET_UTILS:
    @staticmethod
    def get_ip_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])

    @staticmethod
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


class ROS_NODE_TRIGGER:
    node_process = None
    shpath = ""

    def __init__ (self, _shpath):
        self.shpath = _shpath

    def start(self):
        print("called -> ROS_NODE_TRIGGER.start())")
        if(self.shpath != ""):
            print("path is " + self.shpath)
            self.node_process = subprocess.Popen([self.shpath],
                shell=True, 
                preexec_fn=os.setsid
                )
            return True
        else:
            print("Invalid process handle")
            return False

    def stop(self):
        print("called -> ROS_NODE_TRIGGER.stop())")
        if(self.node_process != None):
            print("valid process handle")
            os.killpg(os.getpgid(self.node_process.pid), signal.SIGTERM)
            return True
        else:
            print("Invalid process handle")
            return False


class ROS_NODE_STATE:
    starting = "starting"
    stopping = "stopping"
    engaged = "engaged"
    disengaged = "disengaged"

class ROS_NODE:
    state = ROS_NODE_STATE.disengaged
    trigger = None

    def __init__(self, state, shpath):
        self.state = state
        self.trigger = ROS_NODE_TRIGGER(shpath)

    def set_state(self, state):
        self.state = state

    def start(self):
        print("called -> ROS_NODE.start())")
        if(self.trigger != None and self.trigger.start() == True):
            self.state = ROS_NODE_STATE.engaged
            return True
        else:
            return False

    def stop(self):
        print("called -> ROS_NODE.stop())")
        if(self.trigger != None and self.trigger.stop() == True):
            self.state = ROS_NODE_STATE.disengaged
            return True
        else:
            return False


class MIRO_SERVER:
    # master ros handler
    master_node = ROS_NODE(ROS_NODE_STATE.disengaged,
        '/home/whoobee/mirobot/ros_ws/src/mirobot-web-server/src/scripts/start-ros-master.sh')

    wss_thread = None
    wss_server = None
    http_thread = None
    http_server = None

    # init server
    def __init__(self, _host, _http_port, _wss_port):
        self.wss_thread = threading.Thread(target = self.start_ws_server, args = (_host, _wss_port))
        self.http_thread = threading.Thread(target = self.start_http_server, args = (_host, _http_port))

    # start server
    def start(self):
        self.wss_thread.start()
        self.http_thread.start()

    # stop server
    def stop(self):
        self.wss_thread.stop()
        self.http_thread.stop()

    # Called for every client connecting (after handshake)
    def new_client(self, client, server):
        self.broadcast_status()

    # Called for every client disconnecting
    def client_left(self, client, server):
        self.broadcast_status()

    # Called when a client sends a message
    def message_received(self, client, server, json_message):
        message = ""
        message = json.loads(json_message)
        print("Client(%d) said: %s" % (client['id'], message))
        if(message["command"] == "engage"):
            if(message["arg"] == "all"):
                self.master_node.start()
        elif(message["command"] == "disengage"):
            if(message["arg"] == "all"):
                self.master_node.stop()
        self.broadcast_status()

    # broadcast status to all clients
    def broadcast_status(self):
        response = {"response":self.master_node.state, "arg":"all"}
        json_response = json.dumps(response)
        self.wss_server.send_message_to_all(json_response)

    def start_ws_server(self, _host, _port):
        self.wss_server = WebsocketServer(_port, _host)
        self.wss_server.set_fn_new_client(self.new_client)
        self.wss_server.set_fn_client_left(self.client_left)
        self.wss_server.set_fn_message_received(self.message_received)
        self.wss_server.run_forever()
    
    def start_http_server(self, _host, _port):
        http_handler = CGIHTTPRequestHandler
        http_handler.cgi_directories = ['/cgi-bin', '/htbin']  # this is the default
        self.http_server = HTTPServer(((_host, _port)), http_handler)
        self.http_server.serve_forever()


def main():
    WSS_PORT = 8090
    HTTP_PORT = 8080
    HOST = NET_UTILS.get_lan_ip_address()

    web_dir = os.path.join(os.path.dirname(__file__), 'web/')
    os.chdir(web_dir)

    server = MIRO_SERVER(HOST, HTTP_PORT, WSS_PORT)
    server.start()

if __name__ == "__main__":
    os.environ["DEBUSSY"] = "1"
    main()