#!/usr/bin/env python3

import http.server
import socketserver
import os
from urllib.parse import urlparse

path = os.path.abspath(__file__).replace('scripts/start_webserver.py', 'debug_ui')
os.chdir(path)

PORT = 7954
INDEXFILE = 'index.html'

class MyHandler(http.server.SimpleHTTPRequestHandler):

    def do_GET(self):

        parsedParams = urlparse(self.path)

        if os.access('.' + os.sep + parsedParams.path, os.R_OK):            
            http.server.SimpleHTTPRequestHandler.do_GET(self)

        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()

            with open(INDEXFILE, 'r') as fin:
                self.wfile.write(fin.read().encode())

Handler = MyHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()
        