import websocket
import json
import time
import random

# Replace these with your WebSocket server details
websocket_url = "ws://localhost:8000/"
sensor_id = "sensor_001"

def on_message(ws, message):
    print(f"Received message: {message}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print(f"Closed with status code {close_status_code}: {close_msg}")

def on_open(ws):
    print("WebSocket connection opened")
    
    while True:
        # Simulate sensor data (replace this with actual sensor data)
        sensor_data = {
            "sensor_id": sensor_id,
            "timestamp": int(time.time()),
            "value": random.uniform(0, 100)
        }

        # Convert sensor data to JSON
        json_data = json.dumps(sensor_data)

        # Send the sensor data over the WebSocket connection
        ws.send(json_data)

        # Wait for a short period before sending the next data (adjust as needed)
        time.sleep(1)

if __name__ == "__main__":
    # Create a WebSocket connection
    ws = websocket.WebSocketApp(websocket_url,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    # Set the on_open callback to handle the opening of the connection
    ws.on_open = on_open

    # Start the WebSocket connection (this will run the on_open callback)
    ws.run_forever()
