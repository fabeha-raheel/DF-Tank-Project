#!/usr/bin/python3

from DF_System import *

websocket_url = "ws://192.168.0.154:9090/"

if __name__ == '__main__':

    df_system = DF_System(ws_url=websocket_url)

    df_system.initialize()