import random

amplitudes = random.sample(range(0, 10), 10)

print("Amplitudes")
print(amplitudes)

N = 2

averaged_amplitudes = []

for i in range(int(len(amplitudes)/N)):
    print("i = ", i)
    print("Amplitudes set")
    print(amplitudes[i*N:(i+1)*N])

    average = sum(amplitudes[i*N:(i+1)*N])/N
    print("Average: " , average)
    averaged_amplitudes.append(average)

print(averaged_amplitudes)
