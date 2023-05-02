import socket
import sys

HOST, PORT = "192.168.125.1", 8100
data = " ".join(sys.argv[1:])
print(f"Sending:\t{data}")

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.sendall(str.encode(data + "\n"))

received = sock.recv(1024)
sock.close()

print(f"Received:\t{received}")