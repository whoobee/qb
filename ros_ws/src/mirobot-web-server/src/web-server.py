#!/usr/bin/env python

import SimpleHTTPServer 
import SocketServer 
import os

HOST, PORT = "192.168.0.67", 8000

web_dir = os.path.join(os.path.dirname(__file__), 'web/')
os.chdir(web_dir)

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler 

httpd = SocketServer.TCPServer((HOST, PORT), Handler) 

print "serving at address:", HOST 
print "serving at port:", PORT 
print "web dir:", web_dir 
httpd.serve_forever()