#!/usr/bin/env python3

# %%
# Importing Libraries
import os
from xilinx import *
from flask import Flask, request, jsonify

# %%
# Execution
fpgaPort = os.environ.get('fpgaPort', '/dev/ttyUSB0')
fpgaBAUD = int(os.environ.get('fpgaBaud', '115200'))
print("Connecting to FPGA...")
fpga = Xilinx_Antenna(port=fpgaPort, baud=fpgaBAUD)
fpga.connect()

app = Flask(__name__)

@app.get(os.environ.get('apiRoute', '/fetch'))
def getData():
    if fpga.is_connected() == True:
        print("Reading Antenna data....")
        data = fpga.read_data()
        return jsonify(fpga.return_data()), 200
    else:
        return 'Yikes', 500
    


app.run(host='0.0.0.0', port=int(os.environ.get('inferencePort', '8080')))

