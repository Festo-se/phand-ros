import http.server
import socketserver
import os

PORT = 7954
INDEXFILE = '../debug_ui/index.html'

class MyHandler(http.server.SimpleHTTPRequestHandler):

    def do_GET(self):   

        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        
        with open(INDEXFILE, 'r') as fin:
            self.wfile.write(fin.read().encode())            

Handler = MyHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.serve_forever()

