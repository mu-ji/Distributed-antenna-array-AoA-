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

        if rawFrame[-3:]==[255, 255, 255]:
            if len(rawFrame) == 648:
                received_data = rawFrame[:640]
                num_samples = 160

                phase_data = np.zeros(num_samples, dtype=np.int16)
                for i in range(num_samples):
                    (phase) = struct.unpack('>h', bytes(received_data[4*i+2:4*i+4]))
                    phase_data[i] = phase[0]

                phase_data = phase_data.astype(np.float32)

                packet_number = np.expand_dims(rawFrame[-4], axis=0)

                response_rssi = bytes(rawFrame[-8:-4])
                response_rssi = int(response_rssi.decode('utf-8'))
                response_rssi = np.expand_dims(response_rssi, axis=0)

                data = np.concatenate([phase_data, response_rssi], axis=0)
                data = np.concatenate([data, packet_number], axis=0)
                np.save('{}node.npy'.format(ser.port), data)

                try:
                    response_rssi = bytes(rawFrame[-8:-4])
                    response_rssi = int(response_rssi.decode('utf-8'))
                    #print(iteration)
                    #print('packet_numer:',rawFrame[-4])
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


def dirc_find(ser1_data,ser2_data):

    ser1_phase_data = ser1_data[:-2]
    ser2_phase_data = ser2_data[:-2]

    ser1_rssi = ser1_data[-2]
    ser1_packet_number = ser1_data[-1]

    ser2_rssi = ser2_data[-2]
    ser2_packet_number = ser2_data[-1]

    phase_diff = ser2_phase_data - ser1_phase_data

    for i in range(len(phase_diff)):
        if abs(phase_diff[i]) > 201:
            phase_diff[i] = phase_diff[i] - 402

    mean_phase_diff = np.mean(phase_diff)
    wave_length = 12.5 # cm
    antanna_interval = 6 # cm

    if ser1_rssi > ser2_rssi:
        mean_phase_diff = abs(mean_phase_diff)
    else:
        mean_phase_diff = -abs(mean_phase_diff)
    angle = np.arccos((mean_phase_diff/402)*wave_length/antanna_interval)

    print("angle:",np.rad2deg(angle))

    ser1_update = False
    ser2_update = False
    return angle


#thread3 = threading.Thread(target=dirc_find)
#thread3.start()

'''
ser1_data = np.load('COM4node.npy')
ser2_data = np.load('COM11node.npy')
ser1_phase_data = ser1_data[:-2]
ser2_phase_data = ser2_data[:-2]

ser1_rssi = ser1_data[-2]
ser1_packet_number = ser1_data[-1]

ser2_rssi = ser2_data[-2]
ser2_packet_number = ser2_data[-1]
'''

ser1_phase_data = np.zeros(100)
ser2_phase_data = np.zeros(100)
ser1_rssi = 0
ser2_rssi = 0

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))
line1, = ax1.plot(range(len(ser1_phase_data)), ser1_phase_data, lw=2, label = 'ser1')
line2, = ax1.plot(range(len(ser2_phase_data)), ser2_phase_data, lw=2, label = 'ser2')
ax1.set_xlabel('samples number')
ax1.set_ylabel('phase')
ax1.set_ylim(-201,201)
ax1.set_xlim(0,160)
ax1.legend()
ax1.set_position([0.1,0.6, 0.8,0.3])

ser1_rssi_line, = ax2.plot(range(len(ser1_phase_data)), [ser1_rssi]*len(ser1_phase_data), lw=2, label = 'ser1')
ser2_rssi_line, = ax2.plot(range(len(ser2_phase_data)), [ser2_rssi]*len(ser1_phase_data), lw=2, label = 'ser2')
ax2.set_ylim(0,-50)
ax2.set_xlim(0,160)
ax2.legend()
ax2.set_position([0.1,0.1, 0.3,0.3])


angle = np.pi / 4  # 45度角
direction_line, = ax3.plot([0, np.cos(angle)], [0, np.sin(angle)], color='red')
ax3.set_position([0.6,0.1, 0.3,0.3])
ax3.set_ylim(-1,1)
ax3.set_xlim(-1,1)
ax3.legend()



def update(frame):
    ser1_data = np.load('COM4node.npy')
    ser2_data = np.load('COM11node.npy')
    ser1_phase_data = ser1_data[:-2]
    ser2_phase_data = ser2_data[:-2]

    ser1_rssi = ser1_data[-2]
    ser1_packet_number = ser1_data[-1]

    ser2_rssi = ser2_data[-2]
    ser2_packet_number = ser2_data[-1]

    line1.set_data(range(len(ser1_phase_data)), ser1_phase_data)
    line2.set_data(range(len(ser2_phase_data)), ser2_phase_data)

    ser1_rssi_line.set_data(range(len(ser1_phase_data)), [ser1_rssi]*len(ser1_phase_data))
    ser2_rssi_line.set_data(range(len(ser2_phase_data)), [ser2_rssi]*len(ser2_phase_data))

    angle = dirc_find(ser1_data, ser2_data)

    direction_line.set_data([0, np.cos(angle)], [0, np.sin(angle)])

    return line1, line2, ser1_rssi_line, ser2_rssi_line, direction_line

ani = animation.FuncAnimation(fig, update, frames=1000, interval=500, blit=True)
plt.show()
