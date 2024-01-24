#!/usr/bin/python3

import asyncio
import json
import websockets
import pickle

# Store connected clients in a set
connected_clients = set()

async def handle_client(websocket, path):
    print(f"Client connected from {websocket.remote_address}")

    # Add the new client to the set of connected clients
    connected_clients.add(websocket)

    try:
        while True:
            # Wait for messages from the client
            message = await websocket.recv()
            data = json.loads(message)
            print(data)

            # Broadcast the received message to all other connected clients
            await broadcast(message, sender=websocket)

    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed by client {websocket.remote_address}")

        # Remove the disconnected client from the set
        connected_clients.remove(websocket)

async def broadcast(message, sender):
    # Broadcast the message to all other connected clients
    for client in connected_clients:
        if client != sender:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                # Remove the disconnected client from the set
                connected_clients.remove(client)

if __name__ == "__main__":
    # Replace the host and port with your desired values
    # server_host = "localhost"
    server_host = "0.0.0.0"
    server_port = 9090

    server = websockets.serve(handle_client, server_host, server_port)

    print(f"WebSocket server listening on ws://{server_host}:{server_port}")

    try:
        # Start the WebSocket server
        asyncio.get_event_loop().run_until_complete(server)
        asyncio.get_event_loop().run_forever()

    except KeyboardInterrupt:
        print("WebSocket server stopped.")
