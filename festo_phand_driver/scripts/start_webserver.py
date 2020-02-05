import http.server
import socketserver
import os
from urllib.parse import urlparse

print (os.getcwd())

os.chdir(os.getcwd() + '/dist/hand-test-app')
print(os.getcwd())


PORT = 8000
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

