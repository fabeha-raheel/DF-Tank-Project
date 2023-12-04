import socket
import json
import time

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 5555

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))

    while True:
        # Replace with your actual sensor data retrieval logic
        sensor_data = {"temperature": 25.5, "humidity": 50.2}

        # Convert data to JSON format
        payload = json.dumps(sensor_data)

        # Send data to the server
        client_socket.sendall(payload.encode())
        time.sleep(1)  # Adjust as needed