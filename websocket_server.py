import asyncio
import json
import websockets

async def handle_client(websocket, path):
    print(f"Client connected from {websocket.remote_address}")

    try:
        while True:
            # Wait for messages from the client
            message = await websocket.recv()
            print(f"Received message: {message}")

            # Process the received message (you can implement your logic here)
            # For example, you can broadcast the message to all connected clients
            # or store the data in a database.

    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed by client {websocket.remote_address}")

if __name__ == "__main__":
    # Replace the host and port with your desired values
    # server_host = "localhost"
    server_host = "0.0.0.0"
    server_port = 8000

    server = websockets.serve(handle_client, server_host, server_port)

    print(f"WebSocket server listening on ws://{server_host}:{server_port}")

    try:
        # Start the WebSocket server
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    except KeyboardInterrupt:
        print("WebSocket server stopped.")