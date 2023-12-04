

amplitudes = []

with open(r'tests\fpga_data.txt', 'r') as file:
    for line in file:
        line = line.strip()
        if line.isnumeric():
            amplitudes.append(int(line))
