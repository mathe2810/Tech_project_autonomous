# lidar_socket.py
"""
Socket server pour transmettre les scans du lidar simulé
Usage : importer et envoyer les ranges (distances) à chaque cycle
"""
import socket
import pickle

HOST = '127.0.0.1'
PORT = 5005

class LidarSocketServer:
    def __init__(self, host=HOST, port=PORT):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"LidarSocketServer listening on {self.host}:{self.port}")
        self.conn, _ = self.sock.accept()
        print("LidarSocketServer: client connected")

    def send_scan(self, ranges):
        data = pickle.dumps(ranges)
        self.conn.sendall(data)

    def close(self):
        self.conn.close()
        self.sock.close()

# Exemple d'utilisation :
# server = LidarSocketServer()
# server.send_scan([1.0, 2.0, ...])
# server.close()
