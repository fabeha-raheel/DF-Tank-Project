#!/usr/bin/python3

from DF_System import *

# websocket_url = "ws://192.168.0.116:8000/"

if __name__ == '__main__':

    df_system = DF_System(test=True)
    # df_system.ws_url = websocket_url

    df_system.initialize()

    while True:
        time.sleep(1)