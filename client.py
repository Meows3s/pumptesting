import socket

SERVER_IP = "192.168.89.252"  # revel box IP
PORT = 1969

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((SERVER_IP, PORT))
    s.sendall(b"hello over the LAN")
    print(s.recv(1024).decode())
