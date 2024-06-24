import serial
import numpy as np
import math
import struct
import matplotlib.pyplot as plt
import binascii
import matplotlib.animation as animation

import threading

ser1 = serial.Serial('COM4', 115200)
rawFrame1 = []

ser2 = serial.Serial('COM11', 115200)
rawFrame2 = []


def nRF5340_receiver(ser, rawFrame):
    global ser1_update
    global ser2_update
    while True:
        byte = ser.read(1)        
        rawFrame += byte

        if rawFrame[-4:]==[255, 255, 255, 255]:
            if len(rawFrame) == 648:
                received_data = rawFrame[:640]
                num_samples = 160

                phase_data = np.zeros(num_samples, dtype=np.int16)
                for i in range(num_samples):
                    (phase) = struct.unpack('>h', bytes(received_data[4*i+2:4*i+4]))
                    phase_data[i] = phase[0]

                phase_data = phase_data.astype(np.float32)
                np.save('{}.npy'.format(ser.port), phase_data)

                try:
                    response_rssi = bytes(rawFrame[-8:-4])
                    response_rssi = int(response_rssi.decode('utf-8'))
                    #print(iteration)
                    print(ser.port,response_rssi)
                    #print('-------------------------------')

                except:
                    rawFrame = []
                    continue
                
            rawFrame = []


thread1 = threading.Thread(target=nRF5340_receiver, args=(ser1, rawFrame1))
thread1.start()
thread2 = threading.Thread(target=nRF5340_receiver, args=(ser2, rawFrame2))
thread2.start()

'''
def dirc_find(ser1_update,ser2_update):
    while(1):
        ser1_phase_data = np.load('COM4.npy')
        ser2_phase_data = np.load('COM11.npy')

        phase_diff = ser2_phase_data-ser1_phase_data
        for i in range(len(phase_diff)):
            if abs(phase_diff[i]) > 201:
                phase_diff[i] = phase_diff[i] - 402

        mean_phase_diff = np.mean(phase_diff)
        wave_length = 12.5 # cm
        antanna_interval = 6.6 # cm
        angle = np.arccos((abs(mean_phase_diff)/402)*wave_length/antanna_interval)

        print("angle:",np.rad2deg(angle))

        ser1_update = False
        ser2_update = False


thread3 = threading.Thread(target=dirc_find)
thread3.start()
'''

ser1_phase_data = np.load('COM4.npy')
ser2_phase_data = np.load('COM11.npy')

# 初始化两个 line 对象,给它们一些初始数据
ser1_phase_data = np.zeros(100)
ser2_phase_data = np.zeros(100)

fig = plt.figure()
line1, = plt.plot(range(len(ser1_phase_data)), ser1_phase_data, lw=2, label = 'ser1')
line2, = plt.plot(range(len(ser2_phase_data)), ser2_phase_data, lw=2, label = 'ser2')
plt.xlabel('samples number')
plt.ylabel('phase')
plt.ylim(-201,201)
plt.legend()

def update(frame):
    ser1_phase_data = np.load('COM4.npy')
    ser2_phase_data = np.load('COM11.npy')  

    line1.set_data(range(len(ser1_phase_data)), ser1_phase_data)
    line2.set_data(range(len(ser2_phase_data)), ser2_phase_data)

    return line1, line2

ani = animation.FuncAnimation(fig, update, frames=1000, interval=50, blit=True)
plt.show()
