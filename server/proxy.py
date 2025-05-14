#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer

class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type','text/html')
        self.end_headers()
        message = "Hello, World! Here is a GET response"
        self.wfile.write(bytes(message, "utf8"))
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        print(self.path + " " + body.decode("UTF-8"))
        self.send_response(200)
        self.end_headers()

with HTTPServer(('', 4500), handler) as server:
    server.serve_forever()
