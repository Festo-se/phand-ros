#!/usr/bin/env python3

__author__ = "Timo Schwarzer"
__copyright__ = "Copyright 2020, Festo Coperate Bionic Projects"
__credits__ = ["Timo Schwarzer"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Timo Schwarzer"
__email__ = "timo.schwarzer@festo.com"
__status__ = "Experimental"

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

httpd = socketserver.TCPServer(("", PORT), Handler)
print("serving at port", PORT)
httpd.serve_forever()
        