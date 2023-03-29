# Motor speed 300mm/s


import serial
import numpy as np
#import keyboard
import cv2
import time

# initialize the camera
cam = cv2.VideoCapture(0)
#ret, image = cam.read()


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
x=1
while True:
    speedL = 50
    speedR = 100

    data[4] = speedL >> 8
    data[3] = speedL & 0xFF
    data[6] = speedR >> 8
    data[5] = speedR & 0xFF
    data[7] = buzz

    ser.write(data)
    print(ser.readline())   

    time.sleep(5.235991666666667)

    speedL = 0
    speedR = 0

    data[4] = speedL >> 8
    data[3] = speedL & 0xFF
    data[6] = speedR >> 8
    data[5] = speedR & 0xFF
    data[7] = buzz

    ser.write(data)
    print(ser.readline())
    time.sleep(1)
    ret, image = cam.read()
    cv2.imwrite('/home/pi/takenPictures/'+str(x)+'.jpg',image)
    print('/home/pi/takenPictures/'+str(x)+'.jpg')
    time.sleep(1)
    x += 1
    if x>6 :
        break
    

ser.close()
cam.release()