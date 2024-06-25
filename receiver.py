import serial
import numpy as np
import math
import struct
import matplotlib.pyplot as plt
import binascii

import matplotlib.pyplot as plt

ser = serial.Serial('COM4', 115200)

SPEED_OF_LIGHT  = 299792458
frequency = 16000000

rawFrame = []

diff_list = []

times = 10
iteration = 0
#while iteration < times:
while True:
    byte  = ser.read(1)        
    rawFrame += byte

    if rawFrame[-3:]==[255, 255, 255]:
        if len(rawFrame) == 648:
            received_data = rawFrame[:640]
            num_samples = 160

            phase_data = np.zeros(num_samples, dtype=np.int16)
            for i in range(num_samples):
                (phase) = struct.unpack('>h', bytes(received_data[4*i+2:4*i+4]))
                #print(phase)
                #print((received_data[4*i+2] << 8) | received_data[4*i+3])
                #phase_data[i] = (received_data[4*i+2] << 8) | received_data[4*i+3]
                phase_data[i] = phase[0]

            phase_data = phase_data.astype(np.float32)
            
            #phase_data_diff = np.diff(phase_data)

            iteration = iteration + 1

            #fig = plt.figure()
            #fig.add_subplot(121)
            #plt.plot([i for i in range(160)],phase)
            #fig.add_subplot(122)
            #plt.plot([i for i in range(160)],phase_2pi)
            #plt.show()

            fig = plt.figure()
            plt.plot([i for i in range(160)],phase_data)
            plt.xlabel('samples number')
            plt.ylabel('phase')
            plt.show()
            

            try:
                response_rssi = bytes(rawFrame[-8:-4])
                response_rssi = int(response_rssi.decode('utf-8'))
                #print(iteration)
                print(response_rssi)
                print('packet_number:',rawFrame[-4])
                #print('-------------------------------')

            except:
                rawFrame = []
                continue
            
        rawFrame = []

