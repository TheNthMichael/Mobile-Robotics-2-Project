import socket
import sys
import time

HOST, PORT = "192.168.125.1", 8100
data = " ".join(sys.argv[1:])

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to server and send data
    sock.connect((HOST, PORT))
    sock.sendall(str.encode(data + "\n"))

    # Receive data from the server and shut down
    received = sock.recv(1024)
finally:
    time.sleep(5)
    sock.close()

print(f"Sent:\t{data}")
print(f"Received:\t{received}")