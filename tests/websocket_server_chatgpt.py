import socket
import json

# HOST = '0.0.0.0'  # Listen on all available interfaces
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 5555         # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen()

    print(f"Listening on {HOST}:{PORT}")

    connection, address = server_socket.accept()
    with connection:
        print(f"Connected by {address}")

        while True:
            data = connection.recv(1024)
            if not data:
                break

            sensor_data = json.loads(data.decode())
            print(f"Received data: {sensor_data}")