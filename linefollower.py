import serial
import numpy as np
import time

ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
mobileRobotSer  = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  

def moveMobileRobot(speedL,speedR):
    data = np.zeros(8, dtype=np.uint8)

    data[0] = 0x6D
    data[1] = 0x72
    data[2] = 0x69

    data[4] = speedL >> 8
    data[3] = speedL & 0xFF
    data[6] = speedR >> 8
    data[5] = speedR & 0xFF
    data[7] = 0

    mobileRobotSer.write(data)


P=0
D=0
PD=0
lastError=0
Kp=8
Kd=0

def errorMapping(sensorState):
    errorState=0
    if sensorState == 0b10000000:
        errorState = 7
    if sensorState == 0b11000000:
        errorState = 6
    if sensorState == 0b11100000:
        errorState = 6
    if sensorState == 0b01000000:
        errorState = 5
    if sensorState == 0b01100000:
        errorState = 4
    if sensorState == 0b01110000:
        errorState = 4
    if sensorState == 0b00100000:
        errorState = 3
    if sensorState == 0b00110000:
        errorState = 2
    if sensorState == 0b00111000:
        errorState = 2
    if sensorState == 0b00010000:
        errorState = 1
    if sensorState == 0b00011000:
        errorState = 0
    if sensorState == 0b00001000:
        errorState = -1
    if sensorState == 0b00011100:
        errorState = -2
    if sensorState == 0b00001100:
        errorState = -2
    if sensorState == 0b00000100:
        errorState = -3
    if sensorState == 0b00001110:
        errorState = -4
    if sensorState == 0b00000110:
        errorState = -4
    if sensorState == 0b00000010:
        errorState = -5
    if sensorState == 0b00000111:
        errorState = -6
    if sensorState == 0b00000011:
        errorState = -6
    if sensorState == 0b00000001:
        errorState = -7
    return errorState
    
moveMobileRobot(0,0)
time.sleep(1)
moveMobileRobot(100,-100)
time.sleep(1)
moveMobileRobot(0,0)
time.sleep(1)
moveMobileRobot(-100,100)
time.sleep(2)  
moveMobileRobot(0,0)
time.sleep(1)  
moveMobileRobot(100,-100)
time.sleep(1)
moveMobileRobot(0,0)
time.sleep(10)  
    

while True:
    value=ser.readline()
    error=0
    value=value.decode("utf-8").rstrip()
    if value.isnumeric():
        error   =   int(value)
        error   =   errorMapping(error)
        P       =   Kp*error
        D       =   Kd*(error-lastError)
        PD      =   P+D 
        moveMobileRobot(100-PD,100+PD)
        lastError=error

    print(PD)
    
