#!/usr/bin/python3

from DF_System import *

# websocket_url = "ws://192.168.0.154:9090/" # UGVBS
# websocket_url = "ws://192.168.0.112:9090/" # Lab
websocket_url = "ws://localhost:9090/"   # Testing

if __name__ == '__main__':

    df_system = DF_System(ws_url=websocket_url)

    df_system.initialize()