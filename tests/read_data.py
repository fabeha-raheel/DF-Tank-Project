
## TEST 1 - PLOT DATA FROM FILE

# import matplotlib.pyplot as plt

# amplitudes = []

# with open('fpga_data.txt', 'r') as file:
#     for line in file:
#         line = line.strip()
#         if line.isnumeric():
#             amplitudes.append(int(line))
# print(amplitudes)
# plt.plot(amplitudes)
# plt.show()

## TEST 2 - LIVE PLOTTING

import matplotlib.pyplot as plt 
import numpy as np

amplitudes = []

plt.ion() 
fig = plt.figure() 
# x = range(1, 10, 1)
x = list(range(400000000, 5900000000, 959860))
y = np.zeros([len(x)])
ax = fig.add_subplot(111)

# line1, = ax.plot(x, y, 'b-')
# ax.plot(x[:len(amplitudes)], amplitudes)

while True:
    with open('fpga_data.txt', 'r') as file:
        for line in file:
            line = line.strip()
            if line.isnumeric():
                amplitudes.append(int(line))
                y[len(amplitudes)-1] = int(line)
                ax.plot(x, y, 'b-')
                ax.set_ylim([0, 8000000])
                # ax.plot(x[:len(amplitudes)], amplitudes)
                fig.canvas.draw()
                fig.canvas.flush_events()
    amplitudes = []


# for phase in np.linspace(0, 10*np.pi, 100): 
# 	line1.set_ydata(np.sin(0.5 * x + phase)) 
# 	fig.canvas.draw() 
# 	fig.canvas.flush_events() 
