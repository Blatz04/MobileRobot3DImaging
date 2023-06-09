import serial
import numpy as np
#import keyboard

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)



data = np.zeros(8, dtype=np.uint8)

buzz = 0x00

#hex of 'm' 'r' 'i' 0x6D 0x72 0x69
data[0] = 0x6D
data[1] = 0x72
data[2] = 0x69

speedL = 0
speedR = 0

data[4] = speedL >> 8
data[3] = speedL & 0xFF
data[6] = speedR >> 8
data[5] = speedR & 0xFF
data[7] = buzz

speedL = 100
speedR = -100

data[4] = speedL >> 8
data[3] = speedL & 0xFF
data[6] = speedR >> 8
data[5] = speedR & 0xFF
data[7] = buzz

ser.write(data)
print(ser.readline())    

ser.close()