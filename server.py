import socket
import os

s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

port = 8400

s.bind(("", port))


while True:
    c, addr = s.accept()
    command = s.recv(1024).decode()
    print(command)
    s.close()
