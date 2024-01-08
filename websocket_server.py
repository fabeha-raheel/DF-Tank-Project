import asyncio
import json
import websockets
import pickle

def write_to_log(data):

    f = open('/home/ugv/DF-Tank-Project/log.pickle', 'wb')
    pickle.dump(data, f)
    f.close()

async def handle_client(websocket, path):
    print(f"Client connected from {websocket.remote_address}")

    try:
        while True:
            # Wait for messages from the client
            message = await websocket.recv()
            data = json.loads(message)
            # print(f"Received message: {message}")
            print(data)
            # write_to_log(data)

    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed by client {websocket.remote_address}")

if __name__ == "__main__":
    # Replace the host and port with your desired values
    server_host = "localhost"
    # server_host = "0.0.0.0"
    server_port = 8000

    server = websockets.serve(handle_client, server_host, server_port)

    print(f"WebSocket server listening on ws://{server_host}:{server_port}")

    try:
        # Start the WebSocket server
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    except KeyboardInterrupt:
        print("WebSocket server stopped.")