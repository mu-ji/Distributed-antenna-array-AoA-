import time
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.animation as animation

fig = plt.figure()
line1, = plt.plot([], [], lw=2)
line2, = plt.plot([], [], lw=2)
plt.xlabel('samples number')
plt.ylabel('phase')
plt.ylim(-201,201)
plt.xlim(0,160)
plt.legend()

start_time = time.time()
while(1):

    current_time = time.time()
    elapsed_time = current_time - start_time
    
    if elapsed_time >= 1:
        start_time = current_time

        ser1_phase_data = np.load('COM4.npy')
        ser2_phase_data = np.load('COM11.npy')

        line1.set_data(range(len(ser1_phase_data)), ser1_phase_data)
        line2.set_data(range(len(ser2_phase_data)), ser2_phase_data)

        #plt.show()

        #print(ser2_phase_data-ser1_phase_data)
        phase_diff = ser2_phase_data-ser1_phase_data
        for i in range(len(phase_diff)):
            if abs(phase_diff[i]) > 201:
                phase_diff[i] = phase_diff[i] - 402

        #print(np.mean(phase_diff))

        mean_phase_diff = np.mean(phase_diff)
        wave_length = 12.5 # cm
        antanna_interval = 6.6 # cm
        angle = np.arccos((abs(mean_phase_diff)/402)*wave_length/antanna_interval)
        print(np.rad2deg(angle))
        time.sleep(0.1)