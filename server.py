
from http.server import BaseHTTPRequestHandler, HTTPServer
import base64

fileName =''

#encode
with open(fileName, "rb") as image_file:
    encoded_string = base64.b64encode(image_file.read())

class RequestHandler_httpd(BaseHTTPRequestHandler):
    def do_GET(self):
        message_to_send = encoded_string
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', len(message_to_send))
        self.end_headers()
        self.wfile.write(message_to_send)
        return

#create server connection and send encoded string
server_address_httpd = ('192.168.68.118', 8080)
httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
print('Starting server: ')
httpd.serve_forever()