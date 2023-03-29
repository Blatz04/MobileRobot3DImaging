import serial
import numpy as np
import time
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

ser.write(data)

encRight = 0
encLeft  = 0
pulsePostSamplingRight = 0
samplingRight = 0 
initialEncRight = 0
takeInitialEnc = 0

takenData   = ser.readline().decode("utf-8").rstrip()
while len(takenData) > 10:   
    encRight    = int(takenData.split(',')[7])
    encLeft     = int(takenData.split(',')[6])
    imu         = takenData.split(',')[5]
        
while pulsePostSamplingRight< 34:#32.766:
    speedL = 0
    speedR = 100

    data[4] = speedL >> 8
    data[3] = speedL & 0xFF
    data[6] = speedR >> 8
    data[5] = speedR & 0xFF
    data[7] = buzz

    ser.write(data)
    takenData   = ser.readline().decode("utf-8").rstrip()
    if len(takenData) > 10:
        encRight    = int(takenData.split(',')[7])
        encLeft     = int(takenData.split(',')[6])
        imu         = takenData.split(',')[5]
    if takeInitialEnc < 1:
        initialEncRight = encRight
        takeInitialEnc = 1

    samplingRight           = encRight - initialEncRight - 65535 
    encRight                = 65535
    pulsePostSamplingRight  = pulsePostSamplingRight + samplingRight *-1/65535
    samplingRight           = 0
    #print(pulsePostSamplingRight)    
    print(initialEncRight)

    
speedR = 0
data[4] = speedL >> 8
data[3] = speedL & 0xFF
data[6] = speedR >> 8
data[5] = speedR & 0xFF
data[7] = buzz

ser.write(data)
ser.close()
