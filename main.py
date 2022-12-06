#!/usr/bin/python
import os, sys
import kipr as k
import json
import SocketServer
import SimpleHTTPServer
import httplib


class Robot:
	def __init__(self):
		self.a = 0


class Handler(SimpleHTTPServer.SimpleHTTPRequestHandler):

    def __init__(self, request, client_address, server):
        super().__init__(request, client_address, server)

    def api_response(self):
        return json.dumps({"message": "Hello world"}).encode()

    def do_GET(self):
		print("1")
		try:
			if self.path == '/':
				print("1")
				self.send_response(httplib.OK)
				self.send_header("Content-Type", "application/json")
				self.end_headers()
				self.wfile.write(bytes(self.api_response()))
		except Exception as e:
			print("Error " + str(e))


def main():
	print("here " + str(sys.version_info))
	PORT = 8005
    # Create an object of the above class
	#Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
	my_server = SocketServer.TCPServer(("0.0.0.0", PORT), Handler)
	# Star the server
	print("Server started at " + str(PORT))
	my_server.serve_forever()
	while True:
		i = 0

if __name__== "__main__":
	sys.stdout = os.fdopen(sys.stdout.fileno(),"w",0)
	print("here")
	try:
		main()
	except Exception as e:
		print("error " + str(e))



