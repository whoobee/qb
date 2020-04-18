#!/usr/bin/env python

import socketserver

PORT = 80

Handler = http.server.SimpleHTTPRequestHandler

with socketserver.TCPServer(("~/mirobot/scripts/webpages/index.html", PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()